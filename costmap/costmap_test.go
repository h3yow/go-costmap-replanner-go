package costmap

import (
	"math"
	"sync"
	"testing"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/geom"
)

func testGrid(t *testing.T) *Grid {
	t.Helper()
	g, err := NewGrid(DefaultConfig())
	if err != nil {
		t.Fatalf("NewGrid: %v", err)
	}
	g.RegisterSource("lidar")
	g.RegisterSource("sonar")
	return g
}

func tSec(sec float64) time.Time {
	return time.Unix(0, 0).Add(time.Duration(sec * float64(time.Second)))
}

func TestRejectReasons(t *testing.T) {
	g := testGrid(t)
	now := tSec(10)

	cases := []struct {
		name     string
		r        Reading
		now      time.Time
		want     RejectReason
		accept   bool
		degraded bool
	}{
		{"正常读数", Reading{"lidar", now, []geom.Vec{{X: 1, Y: 1}}, 1}, now, RejectNone, true, false},
		{"未注册源", Reading{"radar", now, []geom.Vec{{X: 1, Y: 1}}, 1}, now, RejectUnknownSensor, false, false},
		{"空读数", Reading{"lidar", now, nil, 1}, now, RejectEmpty, false, false},
		{"NaN坐标", Reading{"lidar", now, []geom.Vec{{X: math.NaN(), Y: 1}}, 1}, now, RejectCorrupt, false, false},
		{"Inf坐标", Reading{"lidar", now, []geom.Vec{{X: 1, Y: math.Inf(1)}}, 1}, now, RejectCorrupt, false, false},
		{"零时间戳", Reading{"lidar", time.Time{}, []geom.Vec{{X: 1, Y: 1}}, 1}, now, RejectCorrupt, false, false},
		{"非法置信度", Reading{"lidar", now, []geom.Vec{{X: 1, Y: 1}}, 1.5}, now, RejectCorrupt, false, false},
		{"过期读数", Reading{"lidar", now.Add(-3 * time.Second), []geom.Vec{{X: 1, Y: 1}}, 1}, now, RejectStale, false, false},
		{"未来时间戳", Reading{"lidar", now.Add(time.Second), []geom.Vec{{X: 1, Y: 1}}, 1}, now, RejectFuture, false, false},
		{"全部越界", Reading{"lidar", now, []geom.Vec{{X: 100, Y: 100}}, 1}, now, RejectOutOfBounds, false, false},
		{"部分越界降级", Reading{"lidar", now, []geom.Vec{{X: 1, Y: 1}, {X: 100, Y: 100}}, 1}, now, RejectOutOfBounds, true, true},
		{"低置信度降级", Reading{"sonar", now, []geom.Vec{{X: 2, Y: 2}}, 0.2}, now, RejectLowConfidence, true, true},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			res := g.Ingest(c.r, c.now)
			t.Logf("输入 source=%q ts=%v pts=%d conf=%.2f => reason=%s degraded=%v detail=%q",
				c.r.Source, c.r.Timestamp, len(c.r.Points), c.r.Confidence, res.Reason, res.Degraded, res.Detail)
			if res.Reason != c.want {
				t.Fatalf("reason=%s, want %s", res.Reason, c.want)
			}
			if res.Accepted() != c.accept || res.Degraded != c.degraded {
				t.Fatalf("accepted=%v degraded=%v, want accepted=%v degraded=%v",
					res.Accepted(), res.Degraded, c.accept, c.degraded)
			}
		})
	}
}

func TestOutOfOrder(t *testing.T) {
	g := testGrid(t)
	now := tSec(10)
	r1 := g.Ingest(Reading{"lidar", now, []geom.Vec{{X: 1, Y: 1}}, 1}, now)
	r2 := g.Ingest(Reading{"lidar", now.Add(-time.Second), []geom.Vec{{X: 1, Y: 1}}, 1}, now)
	t.Logf("顺序读数 => %s；乱序读数 => %s (%s)", r1.Reason, r2.Reason, r2.Detail)
	if r1.Reason != RejectNone || r2.Reason != RejectOutOfOrder {
		t.Fatalf("乱序判定错误: %s / %s", r1.Reason, r2.Reason)
	}
	// 不同传感器互不影响乱序判定
	r3 := g.Ingest(Reading{"sonar", now.Add(-500 * time.Millisecond), []geom.Vec{{X: 1, Y: 1}}, 1}, now)
	if r3.Reason != RejectNone {
		t.Fatalf("不同源不应受乱序影响, got %s", r3.Reason)
	}
}

func TestBlockedAndInflation(t *testing.T) {
	g := testGrid(t)
	now := tSec(10)
	if g.IsBlocked(1, 1) {
		t.Fatal("融合前不应有障碍")
	}
	g.Ingest(Reading{"lidar", now, []geom.Vec{{X: 1, Y: 1}}, 1}, now)
	g.Ingest(Reading{"lidar", now.Add(10 * time.Millisecond), []geom.Vec{{X: 1, Y: 1}}, 1}, now.Add(10*time.Millisecond))
	g.Ingest(Reading{"lidar", now.Add(20 * time.Millisecond), []geom.Vec{{X: 1, Y: 1}}, 1}, now.Add(20*time.Millisecond))
	if !g.IsBlocked(1, 1) {
		t.Fatal("三次命中后该格应致命")
	}
	// 膨胀半径内（0.3m）应被阻塞
	if !g.IsBlocked(1.2, 1.0) {
		t.Fatal("膨胀半径内应阻塞")
	}
	if g.IsBlocked(3, 3) {
		t.Fatal("远处不应阻塞")
	}
	t.Logf("障碍点(1,1)致命，膨胀半径%.2fm内判定一致，地图版本=%d", g.cfg.InflationRadius, g.Version())
}

func TestIngestManyAtomicity(t *testing.T) {
	g := testGrid(t)
	now := tSec(10)
	vBefore := g.Version()
	// 一批中：一个有效、一个过期、一个损坏——无效的绝不进图，有效的生效，版本只 +1
	rs := []Reading{
		{"lidar", now, []geom.Vec{{X: 1, Y: 1}, {X: 1, Y: 1}, {X: 1, Y: 1}}, 1},
		{"lidar", now.Add(-3 * time.Second), []geom.Vec{{X: 2, Y: 2}}, 1},
		{"sonar", now, []geom.Vec{{X: math.NaN(), Y: 0}}, 1},
	}
	res := g.IngestMany(rs, now)
	for _, rr := range res {
		t.Logf("批量项 source=%q => %s accepted=%v", rr.Reading.Source, rr.Reason, rr.Accepted())
	}
	if g.Version() != vBefore+1 {
		t.Fatalf("一次批量成功融合后版本应 +1, got %d", g.Version()-vBefore)
	}
	if !g.IsBlocked(1, 1) {
		t.Fatal("有效读数应已生效")
	}
	if c, _ := g.CostAt(2, 2); c != 0 {
		t.Fatalf("过期读数不得入图, cost=%v", c)
	}
	// 全部无效时地图必须完全不变
	vBefore = g.Version()
	res = g.IngestMany([]Reading{
		{"lidar", now.Add(-3 * time.Second), []geom.Vec{{X: 3, Y: 3}}, 1},
		{"unknown", now, []geom.Vec{{X: 3, Y: 3}}, 1},
	}, now)
	if g.Version() != vBefore {
		t.Fatalf("全部无效时地图版本不应变化, got %d", g.Version())
	}
	t.Logf("全无效批量结果: %s, %s；地图保持版本 %d", res[0].Reason, res[1].Reason, vBefore)
}

func TestDecay(t *testing.T) {
	g := testGrid(t)
	t0 := tSec(10)
	g.Ingest(Reading{"lidar", t0, []geom.Vec{{X: 1, Y: 1}, {X: 1, Y: 1}, {X: 1, Y: 1}}, 1}, t0)
	if !g.IsBlocked(1, 1) {
		t.Fatal("命中后应致命")
	}
	later := t0.Add(5 * time.Second) // 衰减 30/s * 5s = 150，清零
	snap := g.Decay(later)
	if c := snap.Raw[snap.Cols/2+10][snap.Cols/2+10]; c != 0 {
		t.Fatalf("充分衰减后代价应为 0, got %v", c)
	}
	t.Logf("衰减 5s 后原障碍格代价=%v, 版本=%d", 0, snap.Version)
}

func TestConcurrentConsistency(t *testing.T) {
	g := testGrid(t)
	base := tSec(100)
	var wg sync.WaitGroup
	stop := make(chan struct{})

	// 多个生产者并发写入不同源/同源读数
	for w := 0; w < 4; w++ {
		wg.Add(1)
		go func(id int) {
			defer wg.Done()
			i := 0
			for {
				select {
				case <-stop:
					return
				default:
				}
				ts := base.Add(time.Duration(i) * time.Millisecond)
				src := []string{"lidar", "sonar"}[id%2]
				g.IngestMany([]Reading{{
					Source: src, Timestamp: ts,
					Points: []geom.Vec{{X: float64(id) * 0.2, Y: 0.5}}, Confidence: 1,
				}}, ts)
				i++
			}
		}(w)
	}
	// 并发读者：快照必须行/列数一致、每行长度一致、每个代价有界
	for r := 0; r < 4; r++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < 2000; j++ {
				s := g.Snapshot()
				if len(s.Cells) != s.Rows || s.Cols <= 0 {
					t.Errorf("快照尺寸不一致 rows=%d len=%d", s.Rows, len(s.Cells))
					return
				}
				for _, row := range s.Cells {
					if len(row) != s.Cols {
						t.Errorf("行长度不一致: %d vs %d", len(row), s.Cols)
						return
					}
					for _, c := range row {
						if math.IsNaN(c) || c < 0 || c > s.Config.LethalCost {
							t.Errorf("越界代价 %v", c)
							return
						}
					}
				}
			}
		}()
	}
	time.Sleep(100 * time.Millisecond)
	close(stop)
	wg.Wait()
	t.Logf("并发融合结束，最终地图版本=%d，快照结构与代价始终一致", g.Version())
}

func TestNewGridInvalidConfig(t *testing.T) {
	if _, err := NewGrid(Config{Width: 0}); err == nil {
		t.Fatal("非法配置应返回错误")
	}
}

func TestRejectFuture(t *testing.T) {
	g := testGrid(t)
	now := tSec(10)
	r := g.Ingest(Reading{
		Source: "lidar", Timestamp: now.Add(time.Hour),
		Points: []geom.Vec{{X: 1, Y: 1}}, Confidence: 1,
	}, now)
	t.Logf("超前 1h => %s (%s)", r.Reason, r.Detail)
	if r.Reason != RejectFuture {
		t.Fatalf("期望 future_timestamp, got %s", r.Reason)
	}
}

func TestSnapshotIsImmutableCopy(t *testing.T) {
	g := testGrid(t)
	now := tSec(10)
	s1 := g.Snapshot()
	s1.Cells[0][0] = 999 // 篡改返回值
	s2 := g.Snapshot()
	if s2.Cells[0][0] != 0 {
		t.Fatal("快照必须是深拷贝，外部篡改不得影响地图")
	}
	g.Ingest(Reading{"lidar", now, []geom.Vec{{X: 1, Y: 1}, {X: 1, Y: 1}, {X: 1, Y: 1}}, 1}, now)
	s3 := g.Snapshot()
	if s3.Version <= s2.Version {
		t.Fatal("融合后版本应递增")
	}
	t.Logf("快照深拷贝隔离正常，版本 %d -> %d", s2.Version, s3.Version)
}
