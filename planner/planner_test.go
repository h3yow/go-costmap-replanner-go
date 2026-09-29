package planner

import (
	"math"
	"sync"
	"testing"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
)

func mapConfig() costmap.Config {
	return costmap.Config{
		Width: 100, Height: 100, Resolution: 0.1,
		OriginX: 0, OriginY: 0,
		MaxAge: time.Minute, ObstacleCost: 254, InflateRadius: 0.3,
	}
}

func planConfig() Config {
	return Config{MinTurnRadius: 0.3, SwitchMargin: 0.1, ObstacleCost: 254}
}

// wallReadings 生成一道竖直墙（x 固定，y ∈ [y0, y1)）的读数。
// 读数坐标取栅格中心，避免浮点边界量化在墙上打出洞。
func wallReadings(sensor string, stamp time.Time, x, y0, y1 float64) []costmap.Reading {
	var rs []costmap.Reading
	for gy := int(y0 / 0.1); gy < int(y1/0.1); gy++ {
		y := (float64(gy) + 0.5) * 0.1
		rs = append(rs, costmap.Reading{
			SensorID: sensor, Stamp: stamp, X: x, Y: y, Range: 2, Intensity: 1,
		})
	}
	return rs
}

// pathHitsLethal 检查路径是否穿过致命栅格。
func pathHitsLethal(snap costmap.Snapshot, cfg costmap.Config, path []Point, threshold uint8) bool {
	cm := costmap.New(cfg)
	_ = cm
	for _, pt := range path {
		gx := int(math.Floor((pt.X - cfg.OriginX) / cfg.Resolution))
		gy := int(math.Floor((pt.Y - cfg.OriginY) / cfg.Resolution))
		if gx < 0 || gy < 0 || gx >= snap.Width || gy >= snap.Height {
			return true
		}
		if snap.Cells[gy*snap.Width+gx] >= threshold {
			return true
		}
	}
	return false
}

// TestStraightLine 空地图上直线可达，且结果可复现。
func TestStraightLine(t *testing.T) {
	cfg := mapConfig()
	cm := costmap.New(cfg)
	snap := cm.Snapshot()
	req := PlanRequest{Start: Point{1.05, 5.05}, Goal: Point{9.05, 5.05}, Heading: 0}

	r1 := New(planConfig()).Plan(snap, cfg, req)
	r2 := New(planConfig()).Plan(snap, cfg, req)
	t.Logf("ok=%v cost=%.4f points=%d explored=%d", r1.OK, r1.Cost, len(r1.Path), r1.Explored)
	if !r1.OK {
		t.Fatalf("空地图应规划成功: %s %s", r1.Reason, r1.Detail)
	}
	if r1.Cost != r2.Cost || r1.Explored != r2.Explored || len(r1.Path) != len(r2.Path) {
		t.Fatalf("同样输入应复现同样结论: %+v vs %+v", r1, r2)
	}
	for i := range r1.Path {
		if r1.Path[i] != r2.Path[i] {
			t.Fatalf("路径点 %d 不一致: %v vs %v", i, r1.Path[i], r2.Path[i])
		}
	}
}

// TestStaticObstacleAvoidance 静态墙留缺口，路径应绕开致命栅格。
func TestStaticObstacleAvoidance(t *testing.T) {
	cfg := mapConfig()
	cm := costmap.New(cfg)
	now := time.Now()
	cm.Ingest(now, wallReadings("lidar", now, 5.05, 0, 7.0)) // 墙 x=5, y∈[0,7)
	snap := cm.Snapshot()
	req := PlanRequest{Start: Point{1.05, 1.05}, Goal: Point{9.05, 1.05}, Heading: 0}

	res := New(planConfig()).Plan(snap, cfg, req)
	t.Logf("ok=%v cost=%.2f points=%d detail=%s", res.OK, res.Cost, len(res.Path), res.Detail)
	if !res.OK {
		t.Fatalf("应绕过带缺口的墙: %s %s", res.Reason, res.Detail)
	}
	if pathHitsLethal(snap, cfg, res.Path, 254) {
		t.Fatal("路径穿过致命栅格")
	}
	// 路径必须绕到墙的上方（存在 y > 7 的路径点）。
	maxY := 0.0
	for _, pt := range res.Path {
		if pt.Y > maxY {
			maxY = pt.Y
		}
	}
	if maxY < 7.0 {
		t.Fatalf("路径未绕过墙顶，maxY=%.2f", maxY)
	}
}

// TestDynamicObstacleReplan 动态障碍出现在原路径上，重规划应绕开。
func TestDynamicObstacleReplan(t *testing.T) {
	cfg := mapConfig()
	cm := costmap.New(cfg)
	now := time.Now()
	req := PlanRequest{Start: Point{1.05, 5.05}, Goal: Point{9.05, 5.05}, Heading: 0}
	pl := New(planConfig())

	r1 := pl.Plan(cm.Snapshot(), cfg, req)
	if !r1.OK {
		t.Fatalf("初始规划失败: %s", r1.Reason)
	}
	t.Logf("初始: cost=%.2f points=%d", r1.Cost, len(r1.Path))

	// 动态障碍：路中央出现一堵横墙，留下方缺口。
	cm.Ingest(now, wallReadings("lidar", now, 5.05, 3.0, 10.0))
	r2 := pl.Plan(cm.Snapshot(), cfg, req)
	t.Logf("重规划: ok=%v cost=%.2f detail=%s", r2.OK, r2.Cost, r2.Detail)
	if !r2.OK {
		t.Fatalf("动态障碍后应重规划成功: %s %s", r2.Reason, r2.Detail)
	}
	if pathHitsLethal(cm.Snapshot(), cfg, r2.Path, 254) {
		t.Fatal("重规划路径穿过动态障碍")
	}
}

// TestGoalEnclosed 目标被 8 邻格致命障碍包围 -> goal_unreachable。
func TestGoalEnclosed(t *testing.T) {
	cfg := mapConfig()
	cm := costmap.New(cfg)
	now := time.Now()
	var rs []costmap.Reading
	for dx := -1; dx <= 1; dx++ {
		for dy := -1; dy <= 1; dy++ {
			if dx == 0 && dy == 0 {
				continue
			}
			rs = append(rs, costmap.Reading{
				SensorID: "lidar", Stamp: now,
				X: 9.05 + float64(dx)*0.1, Y: 5.05 + float64(dy)*0.1,
				Range: 1, Intensity: 1,
			})
		}
	}
	cm.Ingest(now, rs)
	req := PlanRequest{Start: Point{1.05, 5.05}, Goal: Point{9.05, 5.05}, Heading: 0}
	res := New(planConfig()).Plan(cm.Snapshot(), cfg, req)
	t.Logf("ok=%v reason=%q detail=%s explored=%d", res.OK, res.Reason, res.Detail, res.Explored)
	if res.OK || res.Reason != FailGoalUnreachable {
		t.Fatalf("期望 goal_unreachable，实际 ok=%v reason=%q", res.OK, res.Reason)
	}
}

// TestInvalidEndpoints 起点/目标非法的可区分失败原因。
func TestInvalidEndpoints(t *testing.T) {
	cfg := mapConfig()
	cm := costmap.New(cfg)
	now := time.Now()
	cm.Ingest(now, []costmap.Reading{
		{SensorID: "lidar", Stamp: now, X: 9.05, Y: 5.05, Range: 1, Intensity: 1},
	})
	snap := cm.Snapshot()
	cases := []struct {
		name string
		req  PlanRequest
		want FailReason
	}{
		{"起点越界", PlanRequest{Start: Point{-1, 5}, Goal: Point{5.05, 5.05}}, FailStartInvalid},
		{"目标越界", PlanRequest{Start: Point{1.05, 5.05}, Goal: Point{50, 5}}, FailGoalInvalid},
		{"目标在障碍上", PlanRequest{Start: Point{1.05, 5.05}, Goal: Point{9.05, 5.05}}, FailGoalInvalid},
	}
	for _, tc := range cases {
		res := New(planConfig()).Plan(snap, cfg, tc.req)
		t.Logf("case=%s ok=%v reason=%q detail=%s", tc.name, res.OK, res.Reason, res.Detail)
		if res.OK || res.Reason != tc.want {
			t.Fatalf("case=%s 期望 %q，实际 ok=%v reason=%q", tc.name, tc.want, res.OK, res.Reason)
		}
	}
}

// TestCurvatureInfeasible 极大最小转弯半径下，需要转弯的路径应报曲率不可行。
func TestCurvatureInfeasible(t *testing.T) {
	cfg := mapConfig()
	cm := costmap.New(cfg)
	now := time.Now()
	cm.Ingest(now, wallReadings("lidar", now, 5.05, 0, 7.0)) // 必须绕行 -> 必须转弯
	pcfg := planConfig()
	pcfg.MinTurnRadius = 1000 // 物理上不可能满足的约束
	req := PlanRequest{Start: Point{1.05, 1.05}, Goal: Point{9.05, 1.05}, Heading: 0}
	res := New(pcfg).Plan(cm.Snapshot(), cfg, req)
	t.Logf("ok=%v reason=%q detail=%s", res.OK, res.Reason, res.Detail)
	if res.OK || res.Reason != FailCurvature {
		t.Fatalf("期望 curvature_infeasible，实际 ok=%v reason=%q", res.OK, res.Reason)
	}
}

// TestHysteresisStability 噪声抖动下路径不应来回切换：
// 同样的地图重复规划，应保留旧路径。
func TestHysteresisStability(t *testing.T) {
	cfg := mapConfig()
	cm := costmap.New(cfg)
	now := time.Now()
	cm.Ingest(now, wallReadings("lidar", now, 5.05, 0, 7.0))
	req := PlanRequest{Start: Point{1.05, 1.05}, Goal: Point{9.05, 1.05}, Heading: 0}
	pl := New(planConfig())

	r1 := pl.Plan(cm.Snapshot(), cfg, req)
	if !r1.OK {
		t.Fatalf("初始规划失败: %s", r1.Reason)
	}
	// 模拟传感器噪声：同一障碍位置轻微抖动（y 偏移一个栅格重测）。
	for i := 0; i < 5; i++ {
		stamp := now.Add(time.Duration(i+1) * time.Millisecond)
		jitter := 0.0
		if i%2 == 0 {
			jitter = 0.1
		}
		cm.Ingest(stamp, wallReadings("lidar", stamp, 5.05+jitter, 0, 7.0))
		r := pl.Plan(cm.Snapshot(), cfg, req)
		t.Logf("jitter=%.1f ok=%v cost=%.2f detail=%s", jitter, r.OK, r.Cost, r.Detail)
		if !r.OK {
			t.Fatalf("噪声抖动下规划不应失败: %s %s", r.Reason, r.Detail)
		}
		if len(r.Path) != len(r1.Path) {
			t.Fatalf("噪声抖动导致路径切换: %d -> %d 个点", len(r1.Path), len(r.Path))
		}
		for j := range r.Path {
			if r.Path[j] != r1.Path[j] {
				t.Fatalf("噪声抖动导致路径点 %d 变化: %v -> %v", j, r1.Path[j], r.Path[j])
			}
		}
	}
}

// TestConcurrentPlanAndUpdate 规划与地图更新并发进行：
// 每次规划都基于一致性快照，结论只能是成功或可区分的失败原因。
func TestConcurrentPlanAndUpdate(t *testing.T) {
	cfg := mapConfig()
	cm := costmap.New(cfg)
	now := time.Now()
	req := PlanRequest{Start: Point{1.05, 1.05}, Goal: Point{9.05, 9.05}, Heading: 0}
	pl := New(planConfig())

	stop := make(chan struct{})
	var wg sync.WaitGroup
	wg.Add(1)
	go func() { // 写者：持续添加动态障碍
		defer wg.Done()
		i := 0
		for {
			select {
			case <-stop:
				return
			default:
			}
			stamp := now.Add(time.Duration(i) * time.Millisecond)
			x := 2.05 + float64(i%50)*0.1
			cm.Ingest(stamp, wallReadings("lidar", stamp, x, 2.0, 4.0))
			i++
		}
	}()

	valid := map[FailReason]bool{
		FailNone: true, FailGoalUnreachable: true, FailNoProgressStable: true,
		FailStartInvalid: true, FailGoalInvalid: true, FailCurvature: true,
	}
	for i := 0; i < 200; i++ {
		snap := cm.Snapshot()
		res := pl.Plan(snap, cfg, req)
		if !valid[res.Reason] {
			t.Fatalf("未知失败原因 %q", res.Reason)
		}
		if res.Version != snap.Version {
			t.Fatalf("结果版本 %d 与快照版本 %d 不一致", res.Version, snap.Version)
		}
		if res.OK && pathHitsLethal(snap, cfg, res.Path, 254) {
			t.Fatalf("第 %d 次规划路径穿过致命栅格", i)
		}
	}
	close(stop)
	wg.Wait()
	t.Log("200 次并发规划全部基于一致快照，结论均为合法值")
}
