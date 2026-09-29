// Package replanner 在代价地图持续更新时维持一条稳定、可解释的已提交路径，
// 并在必要时触发重规划。
//
// 稳定性策略（防止障碍短暂抖动导致路径反复切换）：
//   - 已提交路径只有在被确认阻塞、机器人偏航、或替代路径代价优势超过滞回阈值时才替换；
//   - 阻塞确认需要同一路径位置连续 BlockedConfirmations 个更新周期被观测到（去抖），
//     单个周期出现又消失的噪声障碍不会触发重规划；
//   - 替代路径即使更优，也必须好出 HysteresisRatio 比例才切换，避免两条近似路径来回摇摆。
package replanner

import (
	"fmt"
	"math"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
	"github.com/h3yow/go-costmap-replanner-go/geom"
	"github.com/h3yow/go-costmap-replanner-go/planner"
)

// Trigger 描述本次 Update 触发（或未触发）重规划的原因。
type Trigger int

const (
	TrigNone              Trigger = iota // 沿已提交路径继续
	TrigFirstPlan                        // 尚无已提交路径，首次规划
	TrigPathBlocked                      // 已提交路径被持续确认阻塞
	TrigDeviated                         // 机器人实际位姿偏离已提交路径
	TrigBetterAlternative                // 出现代价优势超过滞回阈值的替代路径
)

func (t Trigger) String() string {
	switch t {
	case TrigNone:
		return "none"
	case TrigFirstPlan:
		return "first_plan"
	case TrigPathBlocked:
		return "path_blocked"
	case TrigDeviated:
		return "deviated"
	case TrigBetterAlternative:
		return "better_alternative"
	default:
		return "unknown"
	}
}

// Status 是一次 Update 的可解释输出。
type Status struct {
	Trigger        Trigger
	Replanned      bool
	Success        bool
	Reason         planner.FailReason // 重规划失败时的原因
	Detail         string
	CommittedCost  float64
	CandidateCost  float64
	BlockedIdx     int // 被阻塞路径点下标，-1 表示无
	BlockedSamples int // 该位置连续被观测为阻塞的次数
	MapVersion     uint64
	Committed      []planner.Waypoint
}

// Config 控制重规划滞回与去抖。
type Config struct {
	// BlockedConfirmations：路径同一点连续多少次更新被阻塞才确认。
	BlockedConfirmations int
	// HysteresisRatio：新路径总代价必须比旧路径低该比例以上才替换，如 0.15。
	HysteresisRatio float64
	// DeviationTol：机器人偏离已提交路径的允许距离（米）。
	DeviationTol float64
	// PeriodicReplan：定期尝试寻找更优路径的间隔；0 表示不定期。
	PeriodicReplan time.Duration
	// ReplanLookahead：跳过机器人身后/脚下的路径点，只检查该下标之后的点。
	ReplanLookahead int
}

// DefaultConfig 返回默认重规划配置。
func DefaultConfig() Config {
	return Config{
		BlockedConfirmations: 3,
		HysteresisRatio:      0.15,
		DeviationTol:         0.5,
		PeriodicReplan:       0,
		ReplanLookahead:      2,
	}
}

// Replanner 持有已提交路径与去抖状态。
type Replanner struct {
	pl  *planner.Planner
	cfg Config

	committed   []planner.Waypoint
	blockStreak []int // 与 committed 等长：每个路径点连续被观测为阻塞的次数

	lastPeriodic time.Time
}

// New 创建重规划器。
func New(pl *planner.Planner, cfg Config) *Replanner {
	if cfg.BlockedConfirmations <= 0 {
		cfg = DefaultConfig()
	}
	return &Replanner{pl: pl, cfg: cfg}
}

// Update 基于新的地图快照与当前位姿推进一次决策。
// 整个判定基于单个快照版本，地图在判定期间不会变化。
func (r *Replanner) Update(snap costmap.Snapshot, pose geom.Pose, goal geom.Pose, now time.Time) Status {
	st := Status{BlockedIdx: -1, MapVersion: snap.Version}

	// 1) 首次规划
	if len(r.committed) == 0 {
		st.Trigger = TrigFirstPlan
		res := r.pl.Plan(snap, pose, goal)
		r.applyPlanResult(&st, res)
		return st
	}

	// 2) 定位机器人在已提交路径上的位置，检查是否偏离
	nearestIdx, nearestDist := nearestWaypoint(r.committed, pose.Pos)
	if nearestDist > r.cfg.DeviationTol {
		st.Trigger = TrigDeviated
		res := r.pl.Plan(snap, pose, goal)
		r.applyPlanResult(&st, res)
		st.Detail = fmt.Sprintf("机器人偏离已提交路径 %.2fm，以当前位姿重新规划；%s", nearestDist, st.Detail)
		return st
	}

	// 3) 去抖检查：路径前方是否有点连续 N 个周期被阻塞
	if idx := r.findConfirmedBlocked(snap, nearestIdx); idx >= 0 {
		st.Trigger = TrigPathBlocked
		st.BlockedIdx = idx
		st.BlockedSamples = r.blockStreak[idx]
		res := r.pl.Plan(snap, pose, goal)
		r.applyPlanResult(&st, res)
		return st
	}
	// 记录当前最前方被阻塞点（即使未确认），便于解释
	if i := r.firstBlockedAhead(snap, nearestIdx); i >= 0 {
		st.BlockedIdx = i
		st.BlockedSamples = r.blockStreak[i]
	}

	// 4) 定期尝试更优替代路径（滞回切换）
	if r.cfg.PeriodicReplan > 0 &&
		(r.lastPeriodic.IsZero() || now.Sub(r.lastPeriodic) >= r.cfg.PeriodicReplan) {
		r.lastPeriodic = now
		if cand := r.pl.Plan(snap, pose, goal); cand.Success {
			st.CandidateCost = cand.Cost
			// 与已提交路径“从当前位置起的剩余代价”比较，
			// 避免机器人前进后路径天然变短造成的伪优势。
			oldRemaining := r.committed[len(r.committed)-1].Cost - r.committed[nearestIdx].Cost
			if cand.Cost < oldRemaining*(1-r.cfg.HysteresisRatio) {
				r.commit(cand.Path)
				st.Trigger = TrigBetterAlternative
				st.Replanned = true
				st.Success = true
				st.CommittedCost = cand.Cost
				st.Detail = fmt.Sprintf("替代路径代价 %.2f 较剩余路径 %.2f 优势超过 %.0f%% 滞回阈值，切换路径",
					cand.Cost, oldRemaining, r.cfg.HysteresisRatio*100)
				st.Committed = r.snapshotPath()
				return st
			}
		}
	}

	// 5) 维持已提交路径
	st.Trigger = TrigNone
	st.Success = true
	st.CommittedCost = r.committed[len(r.committed)-1].Cost
	st.Committed = r.snapshotPath()
	st.Detail = "已提交路径仍然有效，继续执行（未达到任何重规划触发条件）"
	return st
}

// findConfirmedBlocked 更新每个路径点的连续阻塞计数，返回前方第一个达到确认阈值的点。
// 任意点在某周期恢复自由，其计数立即清零（去抖核心）。
func (r *Replanner) findConfirmedBlocked(snap costmap.Snapshot, from int) int {
	start := from + r.cfg.ReplanLookahead
	first := -1
	for i := 0; i < len(r.committed); i++ {
		var blocked bool
		if i >= start {
			wp := r.committed[i].Pose.Pos
			blocked = snap.BlockedAt(wp.X, wp.Y)
		}
		if blocked {
			r.blockStreak[i]++
			if r.blockStreak[i] >= r.cfg.BlockedConfirmations && (first < 0 || i < first) {
				first = i
			}
		} else {
			r.blockStreak[i] = 0
		}
	}
	return first
}

func (r *Replanner) firstBlockedAhead(snap costmap.Snapshot, from int) int {
	for i := from + r.cfg.ReplanLookahead; i < len(r.committed); i++ {
		wp := r.committed[i].Pose.Pos
		if snap.BlockedAt(wp.X, wp.Y) {
			return i
		}
	}
	return -1
}

func (r *Replanner) applyPlanResult(st *Status, res planner.Result) {
	st.Replanned = true
	st.Success = res.Success
	if res.Success {
		r.commit(res.Path)
		st.CommittedCost = res.Cost
		st.Detail = res.Detail
	} else {
		// 重规划失败时保留旧路径（若存在），并给出可区分原因
		st.Reason = res.Reason
		st.CommittedCost = 0
		if len(r.committed) > 0 {
			st.CommittedCost = r.committed[len(r.committed)-1].Cost
		}
		st.Detail = "重规划失败，保留已提交路径：" + string(res.Reason) + " — " + res.Detail
	}
	st.Committed = r.snapshotPath()
}

func (r *Replanner) commit(path []planner.Waypoint) {
	r.committed = path
	r.blockStreak = make([]int, len(path))
}

func (r *Replanner) snapshotPath() []planner.Waypoint {
	if len(r.committed) == 0 {
		return nil
	}
	out := make([]planner.Waypoint, len(r.committed))
	copy(out, r.committed)
	return out
}

// Committed 返回当前已提交路径的拷贝（可能为 nil）。
func (r *Replanner) Committed() []planner.Waypoint { return r.snapshotPath() }

// Reset 清除已提交路径与去抖状态。
func (r *Replanner) Reset() {
	r.committed = nil
	r.blockStreak = nil
	r.lastPeriodic = time.Time{}
}

// nearestWaypoint 返回距离 p 最近的路径点下标与距离。
func nearestWaypoint(path []planner.Waypoint, p geom.Vec) (int, float64) {
	bestIdx, best := 0, math.Inf(1)
	for i, wp := range path {
		if d := wp.Pose.Pos.Dist(p); d < best {
			best, bestIdx = d, i
		}
	}
	return bestIdx, best
}
