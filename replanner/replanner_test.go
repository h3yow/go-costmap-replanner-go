package replanner

import (
	"testing"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
	"github.com/h3yow/go-costmap-replanner-go/geom"
	"github.com/h3yow/go-costmap-replanner-go/internal/testutil"
	"github.com/h3yow/go-costmap-replanner-go/kinematics"
	"github.com/h3yow/go-costmap-replanner-go/planner"
)

var (
	startPose = geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0}
	goalPose  = geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0}
	tBase     = time.Unix(100, 0)
)

func newTestReplanner(t *testing.T, cfg Config) *Replanner {
	t.Helper()
	pl, err := planner.New(kinematics.DefaultParams(), planner.DefaultOptions())
	if err != nil {
		t.Fatalf("New planner: %v", err)
	}
	return New(pl, cfg)
}

// wallSnap 返回一堵横在 x=0 的静态墙快照（两侧可绕行）。
func wallSnap() costmap.Snapshot {
	snap := testutil.EmptySnapshot()
	testutil.FillBlocked(&snap, -0.2, -2, 0.2, 2)
	return snap
}

func TestFirstPlanThenHold(t *testing.T) {
	r := newTestReplanner(t, DefaultConfig())
	snap := wallSnap()

	st := r.Update(snap, startPose, goalPose, tBase)
	if !st.Success || st.Trigger != TrigFirstPlan || len(st.Committed) == 0 {
		t.Fatalf("首次规划应成功并提交路径: success=%v trigger=%s detail=%s",
			st.Success, st.Trigger, st.Detail)
	}
	t.Logf("[首次] trigger=%s cost=%.2f 路径点数=%d: %s",
		st.Trigger, st.CommittedCost, len(st.Committed), st.Detail)

	cost1 := st.CommittedCost
	// 地图不变，机器人沿路径小幅前进：必须保持同一路径
	st = r.Update(snap, geom.Pose{Pos: geom.Vec{X: -3.5, Y: 0}, H: 0}, goalPose, tBase.Add(100*time.Millisecond))
	if st.Trigger != TrigNone {
		t.Fatalf("无变化时不应重规划, trigger=%s", st.Trigger)
	}
	if st.CommittedCost != cost1 {
		t.Fatalf("维持阶段已提交代价不应变化: %.2f vs %.2f", st.CommittedCost, cost1)
	}
	t.Logf("[保持] trigger=%s 代价保持 %.2f: %s", st.Trigger, st.CommittedCost, st.Detail)
}

func TestNoiseJitterDoesNotFlip(t *testing.T) {
	cfg := DefaultConfig()
	cfg.BlockedConfirmations = 3
	r := newTestReplanner(t, cfg)

	snap := wallSnap()
	st := r.Update(snap, startPose, goalPose, tBase)
	if !st.Success {
		t.Fatalf("首次规划失败: %s", st.Detail)
	}
	committed := r.Committed()

	// 在路径中段点模拟“一闪而过”的噪声障碍：仅 2 个周期出现
	target := committed[len(committed)/2].Pose.Pos
	for cycle := 1; cycle <= 2; cycle++ {
		noisy := testutil.CloneSnapshot(snap)
		testutil.FillBlocked(&noisy, target.X-0.15, target.Y-0.15, target.X+0.15, target.Y+0.15)
		st = r.Update(noisy, startPose, goalPose, tBase.Add(time.Duration(cycle)*100*time.Millisecond))
		t.Logf("[噪声周期 %d] trigger=%s blockedIdx=%d streak=%d",
			cycle, st.Trigger, st.BlockedIdx, st.BlockedSamples)
		if st.Trigger == TrigPathBlocked {
			t.Fatalf("短暂噪声（%d 个周期）不应确认阻塞并触发重规划", cycle)
		}
	}

	// 噪声消失后下一个周期：计数清零，路径不变
	st = r.Update(snap, startPose, goalPose, tBase.Add(300*time.Millisecond))
	if st.Trigger != TrigNone {
		t.Fatalf("噪声消失后应维持原路径, trigger=%s", st.Trigger)
	}
	t.Logf("[噪声消退] trigger=%s 已提交路径代价保持 %.2f", st.Trigger, st.CommittedCost)
}

func TestPersistentBlockTriggersReplan(t *testing.T) {
	cfg := DefaultConfig()
	cfg.BlockedConfirmations = 3
	r := newTestReplanner(t, cfg)

	snap := wallSnap()
	st := r.Update(snap, startPose, goalPose, tBase)
	committed := r.Committed()

	// 在已提交路径前方点放置持续存在的动态障碍，连续 3 个周期观测
	target := committed[len(committed)/2].Pose.Pos
	blocked := testutil.CloneSnapshot(snap)
	testutil.FillBlocked(&blocked, target.X-0.2, target.Y-0.2, target.X+0.2, target.Y+0.2)

	var triggered bool
	for cycle := 1; cycle <= 3; cycle++ {
		st = r.Update(blocked, startPose, goalPose, tBase.Add(time.Duration(cycle)*100*time.Millisecond))
		t.Logf("[持续障碍周期 %d] trigger=%s blockedIdx=%d streak=%d success=%v",
			cycle, st.Trigger, st.BlockedIdx, st.BlockedSamples, st.Success)
		if st.Trigger == TrigPathBlocked {
			triggered = true
			if !st.Success {
				t.Fatalf("确认阻塞后重规划应成功（存在绕行空间）: %s %s", st.Reason, st.Detail)
			}
			break
		}
	}
	if !triggered {
		t.Fatal("连续 3 个周期阻塞应触发 path_blocked 重规划")
	}
	// 新路径不得再经过障碍块
	for _, wp := range r.Committed() {
		p := wp.Pose.Pos
		if p.X >= target.X-0.25 && p.X <= target.X+0.25 &&
			p.Y >= target.Y-0.25 && p.Y <= target.Y+0.25 {
			t.Fatalf("重规划路径仍穿过动态障碍块: (%.2f,%.2f)", p.X, p.Y)
		}
	}
	t.Logf("重规划成功绕开持续动态障碍，新代价 %.2f", st.CommittedCost)
}

func TestReplanFailureKeepsReasonAndOldPath(t *testing.T) {
	cfg := DefaultConfig()
	cfg.BlockedConfirmations = 1 // 尽快触发重规划
	r := newTestReplanner(t, cfg)

	snap := testutil.EmptySnapshot()
	st := r.Update(snap, startPose, goalPose, tBase)
	if !st.Success {
		t.Fatalf("首次规划失败: %s", st.Detail)
	}
	oldCost := st.CommittedCost
	oldLen := len(st.Committed)

	// 地图变为目标被围死：重规划必须失败且给出明确原因，同时保留旧路径
	dead := testutil.EmptySnapshot()
	testutil.FillBlocked(&dead, 3.0, -1.0, 3.2, 1.0)
	testutil.FillBlocked(&dead, 4.8, -1.0, 5.0, 1.0)
	testutil.FillBlocked(&dead, 3.0, 0.8, 5.0, 1.0)
	testutil.FillBlocked(&dead, 3.0, -1.0, 5.0, -0.8)
	st = r.Update(dead, startPose, goalPose, tBase.Add(100*time.Millisecond))
	t.Logf("[目标被围] trigger=%s replanned=%v success=%v reason=%s detail=%s",
		st.Trigger, st.Replanned, st.Success, st.Reason, st.Detail)
	if st.Success {
		t.Fatal("目标不可达时绝不能返回成功路径")
	}
	if st.Reason != planner.FailGoalEncircled && st.Reason != planner.FailUnreachable &&
		st.Reason != planner.FailGoalBlocked {
		t.Fatalf("失败原因不可区分: %s", st.Reason)
	}
	cur := r.Committed()
	if len(cur) != oldLen {
		t.Fatalf("重规划失败时应保留旧路径, len=%d want %d", len(cur), oldLen)
	}
	if cur[len(cur)-1].Cost != oldCost {
		t.Fatalf("重规划失败时旧路径代价应保留: %.2f want %.2f", cur[len(cur)-1].Cost, oldCost)
	}
	t.Logf("失败原因 %s 可区分，旧路径（%d 点，代价 %.2f）已保留", st.Reason, oldLen, oldCost)
}

func TestDeterministicReplan(t *testing.T) {
	r := newTestReplanner(t, DefaultConfig())
	snap := wallSnap()
	st1 := r.Update(snap, startPose, goalPose, tBase)
	r2 := New(r.pl, DefaultConfig())
	st2 := r2.Update(snap, startPose, goalPose, tBase)
	if st1.CommittedCost != st2.CommittedCost || len(st1.Committed) != len(st2.Committed) {
		t.Fatal("相同输入下两个重规划器结论不一致")
	}
	t.Logf("相同快照与位姿复现相同已提交代价 %.2f 与点数 %d",
		st1.CommittedCost, len(st1.Committed))
}
