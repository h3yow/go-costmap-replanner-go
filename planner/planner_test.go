package planner

import (
	"math"
	"testing"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
	"github.com/h3yow/go-costmap-replanner-go/geom"
	"github.com/h3yow/go-costmap-replanner-go/kinematics"
)

func newTestPlanner(t *testing.T) *Planner {
	t.Helper()
	pl, err := New(kinematics.DefaultParams(), DefaultOptions())
	if err != nil {
		t.Fatalf("New planner: %v", err)
	}
	return pl
}

// emptySnap 生成一张全自由快照。
func emptySnap() costmap.Snapshot {
	cfg := costmap.DefaultConfig()
	cols := int(math.Round(cfg.Width / cfg.Resolution))
	rows := int(math.Round(cfg.Height / cfg.Resolution))
	cells := make([][]float64, rows)
	raw := make([][]float64, rows)
	for r := range cells {
		cells[r] = make([]float64, cols)
		raw[r] = make([]float64, cols)
	}
	return costmap.Snapshot{
		Cells: cells, Raw: raw, Config: cfg,
		TakenAt: time.Unix(0, 0), Cols: cols, Rows: rows,
	}
}

// fillBlocked 在快照上直接标记一块轴对齐矩形区域为致命（测试用）。
func fillBlocked(s *costmap.Snapshot, x0, y0, x1, y1 float64) {
	for row := 0; row < s.Rows; row++ {
		for col := 0; col < s.Cols; col++ {
			x := s.OriginX - s.Config.Width/2 + (float64(col)+0.5)*s.Config.Resolution
			y := s.OriginY - s.Config.Height/2 + (float64(row)+0.5)*s.Config.Resolution
			if x >= x0 && x <= x1 && y >= y0 && y <= y1 {
				s.Cells[row][col] = s.Config.LethalCost
				s.Raw[row][col] = s.Config.LethalCost
			}
		}
	}
	s.Version++
}

func assertPathValid(t *testing.T, pl *Planner, snap costmap.Snapshot, res Result, start, goal geom.Pose) {
	t.Helper()
	if !res.Success || len(res.Path) < 2 {
		t.Fatalf("期望成功路径, got success=%v reason=%s detail=%s", res.Success, res.Reason, res.Detail)
	}
	first := res.Path[0].Pose
	last := res.Path[len(res.Path)-1].Pose
	if first.Pos.Dist(start.Pos) > 1e-9 {
		t.Fatalf("路径起点 %v != 起始位姿 %v", first.Pos, start.Pos)
	}
	if last.Pos.Dist(goal.Pos) > pl.k.XYGoalTol {
		t.Fatalf("路径终点 %v 未到目标 %v", last.Pos, goal.Pos)
	}
	// 每个路径点必须自由；相邻点用基元生成规则复验弧段无碰撞。
	for _, wp := range res.Path {
		if snap.BlockedAt(wp.Pose.Pos.X, wp.Pose.Pos.Y) {
			t.Fatalf("路径点 (%.2f,%.2f) 位于障碍", wp.Pose.Pos.X, wp.Pose.Pos.Y)
		}
	}
	t.Logf("路径有效：%d 个点，代价 %.2f，展开 %d；%s",
		len(res.Path), res.Cost, res.Expanded, res.Detail)
}

func TestPlanOpenSpace(t *testing.T) {
	pl := newTestPlanner(t)
	snap := emptySnap()
	start := geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0}
	goal := geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0}
	res := pl.Plan(snap, start, goal)
	assertPathValid(t, pl, snap, res, start, goal)
}

func TestPlanAvoidsStaticWall(t *testing.T) {
	pl := newTestPlanner(t)
	snap := emptySnap()
	// x=0 处一堵横墙，只留下侧方绕行通道
	fillBlocked(&snap, -0.2, -2.0, 0.2, 2.0)
	start := geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0}
	goal := geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0}
	res := pl.Plan(snap, start, goal)
	assertPathValid(t, pl, snap, res, start, goal)
	// 路径必须真的绕到墙外（|y| 超过墙端 2.0 - 容差）
	maxY := 0.0
	for _, wp := range res.Path {
		if d := math.Abs(wp.Pose.Pos.Y); d > maxY {
			maxY = d
		}
	}
	if maxY <= 2.0 {
		t.Fatalf("路径未绕开墙, 最大 |y|=%.2f", maxY)
	}
	t.Logf("成功绕过静态墙，绕行幅度 |y|max=%.2f", maxY)
}

func TestStartGoalBlocked(t *testing.T) {
	pl := newTestPlanner(t)
	snap := emptySnap()
	start := geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0}
	goal := geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0}
	s1 := snap
	fillBlocked(&s1, -4.3, -0.3, -3.7, 0.3)
	res := pl.Plan(s1, start, goal)
	t.Logf("起点在障碍中 => reason=%s detail=%s", res.Reason, res.Detail)
	if res.Success || res.Reason != FailStartBlocked {
		t.Fatalf("期望 start_blocked, got %s", res.Reason)
	}

	s2 := emptySnap()
	fillBlocked(&s2, 3.7, -0.3, 4.3, 0.3)
	res = pl.Plan(s2, start, goal)
	t.Logf("目标在障碍中 => reason=%s detail=%s", res.Reason, res.Detail)
	if res.Success || res.Reason != FailGoalBlocked {
		t.Fatalf("期望 goal_blocked, got %s", res.Reason)
	}

	// 地图外也按 blocked 处理，且不能返回看似成功的空路径
	s3 := emptySnap()
	res = pl.Plan(s3, geom.Pose{Pos: geom.Vec{X: -100, Y: 0}}, goal)
	if res.Success || res.Reason != FailStartBlocked || len(res.Path) != 0 {
		t.Fatalf("地图外起点应 start_blocked 且路径为空, got %s/%d", res.Reason, len(res.Path))
	}
}

func TestGoalEncircled(t *testing.T) {
	pl := newTestPlanner(t)
	snap := emptySnap()
	// 用四面墙把目标 (4,0) 围在 1m 见方的小盒子里（留角封闭）。
	fillBlocked(&snap, 3.0, -1.0, 3.2, 1.0)  // 左墙
	fillBlocked(&snap, 4.8, -1.0, 5.0, 1.0)  // 右墙
	fillBlocked(&snap, 3.0, 0.8, 5.0, 1.0)   // 上墙
	fillBlocked(&snap, 3.0, -1.0, 5.0, -0.8) // 下墙
	start := geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0}
	goal := geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0}
	res := pl.Plan(snap, start, goal)
	t.Logf("目标被围 => reason=%s detail=%s", res.Reason, res.Detail)
	if res.Success {
		t.Fatalf("被包围目标不应规划出路径")
	}
	if res.Reason != FailGoalEncircled && res.Reason != FailUnreachable {
		t.Fatalf("期望 goal_encircled/unreachable, got %s", res.Reason)
	}
}

func TestStartEncircled(t *testing.T) {
	pl := newTestPlanner(t)
	snap := emptySnap()
	start := geom.Pose{Pos: geom.Vec{X: 0, Y: 0}, H: 0}
	goal := geom.Pose{Pos: geom.Vec{X: 6, Y: 0}, H: 0}
	fillBlocked(&snap, -0.8, -0.8, -0.6, 0.8)
	fillBlocked(&snap, 0.6, -0.8, 0.8, 0.8)
	fillBlocked(&snap, -0.8, 0.6, 0.8, 0.8)
	fillBlocked(&snap, -0.8, -0.8, 0.8, -0.6)
	res := pl.Plan(snap, start, goal)
	t.Logf("起点被围 => reason=%s detail=%s", res.Reason, res.Detail)
	if res.Success {
		t.Fatalf("被包围起点不应规划出路径")
	}
	if res.Reason != FailStartEncircled && res.Reason != FailGoalEncircled {
		t.Fatalf("期望 encircled 类失败, got %s", res.Reason)
	}
}

func TestDeterminism(t *testing.T) {
	pl := newTestPlanner(t)
	snap := emptySnap()
	fillBlocked(&snap, -0.2, -2.0, 0.2, 2.0)
	start := geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0}
	goal := geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0}
	r1 := pl.Plan(snap, start, goal)
	for i := 0; i < 5; i++ {
		r2 := pl.Plan(snap, start, goal)
		if r1.Success != r2.Success || r1.Reason != r2.Reason || r1.Cost != r2.Cost ||
			len(r1.Path) != len(r2.Path) || r1.Expanded != r2.Expanded {
			t.Fatalf("第 %d 次规划结果不一致", i)
		}
		for j := range r1.Path {
			if r1.Path[j].Pose != r2.Path[j].Pose {
				t.Fatalf("第 %d 个路径点不一致: %v vs %v", j, r1.Path[j].Pose, r2.Path[j].Pose)
			}
		}
	}
	t.Logf("连续 6 次规划结论完全一致：%s，代价 %.2f，点数 %d", r1.Reason, r1.Cost, len(r1.Path))
}

func TestDynamicObstacleChangesResult(t *testing.T) {
	pl := newTestPlanner(t)
	before := emptySnap()
	fillBlocked(&before, -0.2, -2.0, 0.2, 2.0)
	start := geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0}
	goal := geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0}
	r1 := pl.Plan(before, start, goal)
	if !r1.Success {
		t.Fatalf("动态障碍出现前应可规划: %s", r1.Detail)
	}
	// 动态障碍封堵原绕行口（上方），路径必须改走下方
	after := before
	cp := make([][]float64, after.Rows)
	for r := range after.Cells {
		cp[r] = append([]float64(nil), after.Cells[r]...)
	}
	after.Cells = cp
	fillBlocked(&after, -0.6, 2.0, 1.5, 4.0)
	r2 := pl.Plan(after, start, goal)
	if !r2.Success {
		t.Fatalf("动态障碍出现后仍应能绕行: %s %s", r2.Reason, r2.Detail)
	}
	assertPathValid(t, pl, after, r2, start, goal)
	t.Logf("动态障碍前路径代价 %.2f，之后 %.2f，规划均成功且路径有效", r1.Cost, r2.Cost)
}

func TestExpansionBudgetTimeout(t *testing.T) {
	opts := DefaultOptions()
	opts.MaxExpansions = 5 // 极小预算，必然耗尽
	pl, err := New(kinematics.DefaultParams(), opts)
	if err != nil {
		t.Fatal(err)
	}
	snap := emptySnap()
	fillBlocked(&snap, -0.2, -2.0, 0.2, 2.0)
	res := pl.Plan(snap,
		geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0},
		geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0})
	t.Logf("极小预算 => success=%v reason=%s expanded=%d detail=%s",
		res.Success, res.Reason, res.Expanded, res.Detail)
	if res.Success || res.Reason != FailTimeout {
		t.Fatalf("期望 timeout, got success=%v reason=%s", res.Success, res.Reason)
	}
	if len(res.Path) != 0 {
		t.Fatal("失败时路径必须为空")
	}
}
