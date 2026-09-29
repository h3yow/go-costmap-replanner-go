package costmap

import (
	"fmt"
	"math"
	"sync"
	"testing"
	"time"
)

func testConfig() Config {
	return Config{
		Width: 50, Height: 50, Resolution: 0.1,
		OriginX: 0, OriginY: 0,
		MaxAge: 500 * time.Millisecond, ObstacleCost: 254, InflateRadius: 0.3,
	}
}

// TestRejectReasons 验证各类异常读数给出可区分的拒绝原因，且不进地图。
func TestRejectReasons(t *testing.T) {
	now := time.Now()
	cases := []struct {
		name string
		r    Reading
		want RejectReason
	}{
		{"过期", Reading{SensorID: "a", Stamp: now.Add(-time.Second), X: 1, Y: 1, Range: 1, Intensity: 1}, RejectStale},
		{"越界", Reading{SensorID: "a", Stamp: now, X: 100, Y: 1, Range: 1, Intensity: 1}, RejectOutOfBounds},
		{"负坐标越界", Reading{SensorID: "a", Stamp: now, X: -1, Y: 1, Range: 1, Intensity: 1}, RejectOutOfBounds},
		{"NaN坐标", Reading{SensorID: "a", Stamp: now, X: math.NaN(), Y: 1, Range: 1, Intensity: 1}, RejectCorrupt},
		{"Inf坐标", Reading{SensorID: "a", Stamp: now, X: math.Inf(1), Y: 1, Range: 1, Intensity: 1}, RejectCorrupt},
		{"负距离", Reading{SensorID: "a", Stamp: now, X: 1, Y: 1, Range: -0.5, Intensity: 1}, RejectCorrupt},
		{"置信度越界", Reading{SensorID: "a", Stamp: now, X: 1, Y: 1, Range: 1, Intensity: 1.5}, RejectCorrupt},
		{"未来时间戳", Reading{SensorID: "a", Stamp: now.Add(time.Hour), X: 1, Y: 1, Range: 1, Intensity: 1}, RejectFutureStamped},
	}
	for _, tc := range cases {
		m := New(testConfig())
		dec := m.Ingest(now, []Reading{tc.r})[0]
		t.Logf("case=%s input=%+v -> accepted=%v reason=%q detail=%s",
			tc.name, tc.r, dec.Accepted, dec.Reason, dec.Detail)
		if dec.Accepted || dec.Reason != tc.want {
			t.Fatalf("case=%s 期望拒绝原因 %q，实际 accepted=%v reason=%q", tc.name, tc.want, dec.Accepted, dec.Reason)
		}
		if v := m.Snapshot().Version; v != 0 {
			t.Fatalf("case=%s 被拒绝的读数不应推进版本号，实际 version=%d", tc.name, v)
		}
	}
}

// TestOutOfOrder 验证乱序读数被拒绝，且不影响已更新的状态。
func TestOutOfOrder(t *testing.T) {
	m := New(testConfig())
	now := time.Now()
	d1 := m.Ingest(now, []Reading{{SensorID: "a", Stamp: now, X: 1.05, Y: 1.05, Range: 1, Intensity: 1}})[0]
	d2 := m.Ingest(now, []Reading{{SensorID: "a", Stamp: now.Add(-time.Millisecond), X: 2.05, Y: 2.05, Range: 1, Intensity: 1}})[0]
	t.Logf("第一条 accepted=%v，乱序第二条 accepted=%v reason=%q", d1.Accepted, d2.Accepted, d2.Reason)
	if !d1.Accepted || d2.Accepted || d2.Reason != RejectOutOfOrder {
		t.Fatalf("乱序判定错误: d1=%+v d2=%+v", d1, d2)
	}
	// 不同传感器互不影响。
	d3 := m.Ingest(now, []Reading{{SensorID: "b", Stamp: now.Add(-time.Millisecond), X: 2.05, Y: 2.05, Range: 1, Intensity: 1}})[0]
	if !d3.Accepted {
		t.Fatalf("不同传感器的旧时间戳不应被判乱序: %+v", d3)
	}
}

// TestBatchAtomic 验证混合批次：接受的生效、拒绝的不生效，版本号只前进一次。
func TestBatchAtomic(t *testing.T) {
	m := New(testConfig())
	now := time.Now()
	batch := []Reading{
		{SensorID: "a", Stamp: now, X: 1.05, Y: 1.05, Range: 1, Intensity: 1}, // 接受
		{SensorID: "a", Stamp: now, X: 100, Y: 1, Range: 1, Intensity: 1},     // 越界
		{SensorID: "b", Stamp: now, X: 2.05, Y: 2.05, Range: 1, Intensity: 1}, // 接受
	}
	decs := m.Ingest(now, batch)
	for i, d := range decs {
		t.Logf("batch[%d] accepted=%v reason=%q", i, d.Accepted, d.Reason)
	}
	if !decs[0].Accepted || decs[1].Accepted || !decs[2].Accepted {
		t.Fatalf("批次结论错误: %+v", decs)
	}
	snap := m.Snapshot()
	if snap.Version != 1 {
		t.Fatalf("含接受读数的批次应只推进一次版本号，实际 %d", snap.Version)
	}
	if c := snap.Cells[10*50+10]; c != 254 {
		t.Fatalf("接受读数的命中栅格应为 254，实际 %d", c)
	}
	if c := snap.Cells[20*50+20]; c != 254 {
		t.Fatalf("接受读数的命中栅格应为 254，实际 %d", c)
	}
}

// TestConcurrentIngestSnapshot 并发读写：每个批次写一组独占栅格，
// 任何快照中同批次栅格必须全有或全无（无半提交状态），版本号单调。
func TestConcurrentIngestSnapshot(t *testing.T) {
	m := New(testConfig())
	now := time.Now()
	const batches = 32
	const perBatch = 4

	// 批次 b 的栅格区域：行 = b，列 0..perBatch-1（互不重叠）。
	makeBatch := func(b int) []Reading {
		var rs []Reading
		for i := 0; i < perBatch; i++ {
			rs = append(rs, Reading{
				SensorID: fmt.Sprintf("s%d", b), Stamp: now,
				X: (float64(i) + 0.5) * 0.1, Y: (float64(b) + 0.5) * 0.1,
				Range: 1, Intensity: 1,
			})
		}
		return rs
	}

	var wg sync.WaitGroup
	stop := make(chan struct{})
	var readErr error
	var rmu sync.Mutex

	// 读者：持续取快照，校验一致性。
	for r := 0; r < 4; r++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			lastVersion := uint64(0)
			for {
				select {
				case <-stop:
					return
				default:
				}
				snap := m.Snapshot()
				if snap.Version < lastVersion {
					rmu.Lock()
					readErr = fmt.Errorf("版本号回退: %d -> %d", lastVersion, snap.Version)
					rmu.Unlock()
					return
				}
				lastVersion = snap.Version
				for b := 0; b < batches; b++ {
					set := 0
					for i := 0; i < perBatch; i++ {
						if snap.Cells[b*snap.Width+i] == 254 {
							set++
						}
					}
					if set != 0 && set != perBatch {
						rmu.Lock()
						readErr = fmt.Errorf("批次 %d 出现半提交状态: %d/%d 栅格已写", b, set, perBatch)
						rmu.Unlock()
						return
					}
				}
			}
		}()
	}

	// 写者：并发提交批次。
	var wwg sync.WaitGroup
	for b := 0; b < batches; b++ {
		wwg.Add(1)
		go func(b int) {
			defer wwg.Done()
			m.Ingest(now, makeBatch(b))
		}(b)
	}
	wwg.Wait()
	close(stop)
	wg.Wait()

	if readErr != nil {
		t.Fatalf("并发一致性校验失败: %v", readErr)
	}
	final := m.Snapshot()
	t.Logf("最终版本=%d（%d 个批次各含 %d 条）", final.Version, batches, perBatch)
	if final.Version != batches {
		t.Fatalf("每个批次都含接受读数，版本号应为 %d，实际 %d", batches, final.Version)
	}
}

// TestDeterministic 同样的读数序列重放两次，地图逐字节一致。
func TestDeterministic(t *testing.T) {
	now := time.Now()
	run := func() Snapshot {
		m := New(testConfig())
		for b := 0; b < 10; b++ {
			var rs []Reading
			for i := 0; i < 5; i++ {
				rs = append(rs, Reading{
					SensorID: "a", Stamp: now.Add(time.Duration(b) * time.Millisecond),
					X: float64(b)*0.4 + 0.05, Y: float64(i)*0.4 + 0.05,
					Range: 1, Intensity: 0.8,
				})
			}
			m.Ingest(now, rs)
		}
		return m.Snapshot()
	}
	s1, s2 := run(), run()
	if s1.Version != s2.Version {
		t.Fatalf("版本不一致: %d vs %d", s1.Version, s2.Version)
	}
	for i := range s1.Cells {
		if s1.Cells[i] != s2.Cells[i] {
			t.Fatalf("栅格 %d 不一致: %d vs %d", i, s1.Cells[i], s2.Cells[i])
		}
	}
	t.Logf("两次重放得到一致地图，version=%d", s1.Version)
}
