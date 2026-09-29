// Package integration 端到端验证：多路传感器并发接入 -> 代价地图融合
// -> Hybrid A* 规划 -> 去抖/滞回重规划 的整条链路。
package integration

import (
	"sync"
	"testing"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
	"github.com/h3yow/go-costmap-replanner-go/geom"
	"github.com/h3yow/go-costmap-replanner-go/internal/testutil"
	"github.com/h3yow/go-costmap-replanner-go/kinematics"
	"github.com/h3yow/go-costmap-replanner-go/planner"
	"github.com/h3yow/go-costmap-replanner-go/replanner"
	simpkg "github.com/h3yow/go-costmap-replanner-go/sim"
)

// TestConcurrentPipelineConsistency 多写者并发融合 + 多读者并发快照，
// 在 -race 下断言地图结构/代价始终一致，且所有异常读数都有可区分的处置。
func TestConcurrentPipelineConsistency(t *testing.T) {
	grid, err := costmap.NewGrid(costmap.DefaultConfig())
	if err != nil {
		t.Fatal(err)
	}
	for _, s := range simpkg.DefaultSources() {
		grid.RegisterSource(s.Name)
	}
	grid.RegisterSource("probe")

	sc := simpkg.Scenario{
		Sources: simpkg.DefaultSources(),
		Static:  simpkg.WallPoints(geom.Vec{X: 0, Y: -2}, geom.Vec{X: 0, Y: 2}, 0.15),
		Dynamic: func(td time.Duration) []geom.Vec {
			if td.Seconds() < 1 {
				return nil
			}
			return []geom.Vec{{X: 1, Y: 0}, {X: 1.1, Y: 0}}
		},
		Seed: 3,
	}
	sm := simpkg.New(sc)
	base := sm.Now(0)

	seen := map[costmap.RejectReason]int{}
	var mu sync.Mutex
	var wg sync.WaitGroup
	stop := make(chan struct{})

	// 单生产者按节拍产出读数（确定性模拟器非并发安全，对应真实系统中
	// 每路驱动各自独立的数据源），分发给每路传感器一个并发写地图 worker。
	feeds := map[string]chan costmap.Reading{}
	for _, spec := range sc.Sources {
		feeds[spec.Name] = make(chan costmap.Reading, 32)
	}
	for name, ch := range feeds {
		wg.Add(1)
		go func(name string, ch chan costmap.Reading) {
			defer wg.Done()
			for r := range ch {
				res := grid.Ingest(r, r.Timestamp.Add(40*time.Millisecond))
				if !res.Accepted() || res.Degraded {
					mu.Lock()
					seen[res.Reason]++
					mu.Unlock()
				}
			}
		}(name, ch)
	}
	wg.Add(1)
	go func() {
		defer wg.Done()
		for tk := 100 * time.Millisecond; tk <= 3*time.Second; tk += 100 * time.Millisecond {
			select {
			case <-stop:
				return
			default:
			}
			for _, r := range sm.Step(tk) {
				feeds[r.Source] <- r
			}
		}
	}()

	// 异常注入者：损坏/越界/过期/乱序/未注册/低置信度
	wg.Add(1)
	go func() {
		defer wg.Done()
		ingest := func(r costmap.Reading, now time.Time) costmap.IngestResult {
			res := grid.Ingest(r, now)
			mu.Lock()
			seen[res.Reason]++
			mu.Unlock()
			return res
		}
		ingest(simpkg.StaleReading("probe", base.Add(-3*time.Second),
			[]geom.Vec{{X: 1, Y: 1}}), base)
		ingest(simpkg.CorruptReading("probe", base.Add(time.Second)), base.Add(time.Second))
		ingest(simpkg.OutOfBoundsReading("probe", base.Add(1100*time.Millisecond),
			geom.Vec{X: 50, Y: 50}), base.Add(1100*time.Millisecond))
		ingest(costmap.Reading{Source: "ghost", Timestamp: base.Add(1200 * time.Millisecond),
			Points: []geom.Vec{{X: 1, Y: 1}}, Confidence: 1}, base.Add(1200*time.Millisecond))
		ingest(costmap.Reading{Source: "probe", Timestamp: base.Add(1300 * time.Millisecond),
			Points: []geom.Vec{{X: 1, Y: 1}}, Confidence: 0.1}, base.Add(1300*time.Millisecond))
		ingest(costmap.Reading{Source: "probe", Timestamp: base.Add(2 * time.Second),
			Points: []geom.Vec{{X: 2, Y: 2}}, Confidence: 1}, base.Add(2*time.Second))
		ingest(costmap.Reading{Source: "probe", Timestamp: base.Add(1500 * time.Millisecond),
			Points: []geom.Vec{{X: 2, Y: 2}}, Confidence: 1}, base.Add(2100*time.Millisecond))
	}()

	// 并发读者：持续校验快照一致性
	for i := 0; i < 4; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < 1000; j++ {
				select {
				case <-stop:
					return
				default:
				}
				s := grid.Snapshot()
				if len(s.Cells) != s.Rows {
					t.Errorf("快照行数不一致")
					return
				}
				for _, row := range s.Cells {
					if len(row) != s.Cols {
						t.Errorf("快照列数不一致")
						return
					}
					for _, c := range row {
						if c < 0 || c > s.Config.LethalCost {
							t.Errorf("越界代价 %v", c)
							return
						}
					}
				}
				time.Sleep(time.Millisecond)
			}
		}()
	}

	time.Sleep(3200 * time.Millisecond)
	close(stop)
	for _, ch := range feeds {
		close(ch)
	}
	wg.Wait()

	t.Logf("异常处置统计: %+v", seen)
	for _, want := range []costmap.RejectReason{
		costmap.RejectStale, costmap.RejectCorrupt, costmap.RejectOutOfBounds,
		costmap.RejectUnknownSensor, costmap.RejectLowConfidence, costmap.RejectOutOfOrder,
	} {
		if seen[want] == 0 {
			t.Errorf("期望异常原因 %s 在并发链路中出现，实际 0 次", want)
		}
	}
}

// TestDynamicChangeDuringReplanning 用固定快照序列验证：重规划决策基于单一
// 快照版本；障碍只抖动 1~2 个周期不切换路径，持续 3 个周期才切换，切换后
// 新路径不再穿障。地图在两次 Update 之间如何变化都不影响单次判定一致性。
func TestDynamicChangeDuringReplanning(t *testing.T) {
	pl, err := planner.New(kinematics.DefaultParams(), planner.DefaultOptions())
	if err != nil {
		t.Fatal(err)
	}
	cfg := replanner.DefaultConfig()
	cfg.BlockedConfirmations = 3
	rp := replanner.New(pl, cfg)

	base := testutil.EmptySnapshot()
	testutil.FillBlocked(&base, -0.2, -2, 0.2, 2)
	start := geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0}
	goal := geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0}

	st := rp.Update(base, start, goal, time.Unix(0, 0))
	if !st.Success {
		t.Fatalf("首次规划失败: %s", st.Detail)
	}
	originalCost := st.CommittedCost
	target := st.Committed[len(st.Committed)/2].Pose.Pos

	noiseBlock := func() costmap.Snapshot {
		s := testutil.CloneSnapshot(base)
		testutil.FillBlocked(&s, target.X-0.15, target.Y-0.15, target.X+0.15, target.Y+0.15)
		return s
	}

	// 两次带噪声快照（障碍“闪现”），必须维持原路径
	for i := 1; i <= 2; i++ {
		st = rp.Update(noiseBlock(), start, goal, time.Unix(0, 0).Add(time.Duration(i)*100*time.Millisecond))
		if st.Trigger == replanner.TrigPathBlocked {
			t.Fatalf("第 %d 个噪声周期不应触发重规划", i)
		}
	}
	// 噪声消退，阻塞计数清零
	st = rp.Update(base, start, goal, time.Unix(0, 0).Add(300*time.Millisecond))
	if st.Trigger != replanner.TrigNone {
		t.Fatalf("噪声消退后应维持, got %s", st.Trigger)
	}
	if st.CommittedCost != originalCost {
		t.Fatalf("噪声不应改变已提交代价: %.2f vs %.2f", st.CommittedCost, originalCost)
	}

	// 障碍持续存在 3 个周期：必须切换
	persistent := noiseBlock()
	var switched bool
	for i := 1; i <= 3; i++ {
		st = rp.Update(persistent, start, goal, time.Unix(0, 0).Add(time.Duration(3+i)*100*time.Millisecond))
		t.Logf("持续障碍周期 %d: trigger=%s success=%v reason=%s",
			i, st.Trigger, st.Success, st.Reason)
		if st.Trigger == replanner.TrigPathBlocked {
			if !st.Success {
				t.Fatalf("确认阻塞后重规划应成功: %s", st.Reason)
			}
			switched = true
			break
		}
	}
	if !switched {
		t.Fatal("持续 3 周期障碍必须触发重规划")
	}
	for _, wp := range rp.Committed() {
		p := wp.Pose.Pos
		if p.X >= target.X-0.2 && p.X <= target.X+0.2 &&
			p.Y >= target.Y-0.2 && p.Y <= target.Y+0.2 {
			t.Fatalf("新路径仍穿过障碍: (%.2f,%.2f)", p.X, p.Y)
		}
	}
}

// TestUnreachableGoalEndToEnd 目标完全被围时，整条链路给出明确失败而非空成功。
func TestUnreachableGoalEndToEnd(t *testing.T) {
	pl, _ := planner.New(kinematics.DefaultParams(), planner.DefaultOptions())
	rp := replanner.New(pl, replanner.DefaultConfig())

	snap := testutil.EmptySnapshot()
	testutil.FillBlocked(&snap, 3.0, -1.0, 3.2, 1.0)
	testutil.FillBlocked(&snap, 4.8, -1.0, 5.0, 1.0)
	testutil.FillBlocked(&snap, 3.0, 0.8, 5.0, 1.0)
	testutil.FillBlocked(&snap, 3.0, -1.0, 5.0, -0.8)

	st := rp.Update(snap,
		geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0},
		geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0}, time.Unix(0, 0))
	t.Logf("不可达目标: success=%v reason=%s detail=%s", st.Success, st.Reason, st.Detail)
	if st.Success || len(st.Committed) != 0 {
		t.Fatal("目标不可达时不得返回成功或非空路径")
	}
	if st.Reason == planner.FailNone {
		t.Fatal("必须给出可区分的失败原因")
	}
}

// TestReproducibleEndToEnd 相同快照序列两次喂给两个重规划器，结论必须一致。
func TestReproducibleEndToEnd(t *testing.T) {
	pl1, _ := planner.New(kinematics.DefaultParams(), planner.DefaultOptions())
	pl2, _ := planner.New(kinematics.DefaultParams(), planner.DefaultOptions())
	r1 := replanner.New(pl1, replanner.DefaultConfig())
	r2 := replanner.New(pl2, replanner.DefaultConfig())

	snaps := []costmap.Snapshot{
		func() costmap.Snapshot {
			s := testutil.EmptySnapshot()
			testutil.FillBlocked(&s, -0.2, -2, 0.2, 2)
			return s
		}(),
	}
	snaps = append(snaps, testutil.CloneSnapshot(snaps[0]))
	snaps = append(snaps, testutil.CloneSnapshot(snaps[0]))

	start := geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0}
	goal := geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0}
	var c1, c2 float64
	for i, s := range snaps {
		now := time.Unix(0, 0).Add(time.Duration(i+1) * 100 * time.Millisecond)
		st1 := r1.Update(s, start, goal, now)
		st2 := r2.Update(s, start, goal, now)
		c1, c2 = st1.CommittedCost, st2.CommittedCost
		if st1.Trigger != st2.Trigger || st1.Success != st2.Success ||
			st1.Reason != st2.Reason || len(st1.Committed) != len(st2.Committed) {
			t.Fatalf("第 %d 周期结论不一致", i)
		}
	}
	if c1 != c2 {
		t.Fatalf("最终代价不可复现: %.4f vs %.4f", c1, c2)
	}
	t.Logf("两条相同快照序列复现相同结论，代价 %.2f", c1)
}
