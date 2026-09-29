// Package planner 在代价地图快照上做路径规划与重规划，
// 保证结果可解释、可复现，并带抖动抑制。
package planner

// FailReason 是规划失败的可区分原因。
type FailReason string

const (
	FailNone             FailReason = ""                  // 成功
	FailStartInvalid     FailReason = "start_invalid"     // 起点越界或落在障碍上
	FailGoalInvalid      FailReason = "goal_invalid"      // 目标越界或落在障碍上
	FailGoalUnreachable  FailReason = "goal_unreachable"  // 目标被障碍包围，无可达路径
	FailCurvature        FailReason = "curvature_infeasible" // 路径不满足差速底盘曲率约束
	FailNoProgressStable FailReason = "no_path"           // 搜索耗尽，无路径
)

// Point 是地图坐标系中的点（米）。
type Point struct{ X, Y float64 }

// PlanRequest 是一次规划请求。
type PlanRequest struct {
	Start, Goal Point
	Heading     float64 // 当前航向（弧度），用于曲率约束
}

// PlanResult 是规划结论：成功给出路径与代价，失败给出可区分原因。
type PlanResult struct {
	OK       bool
	Reason   FailReason
	Detail   string
	Path     []Point
	Cost     float64 // 路径总代价（含障碍代价与曲率惩罚）
	Explored int     // 搜索扩展的节点数，用于解释与复现核对
	Version  uint64  // 规划所基于的地图版本
}

// Config 是规划器配置。
type Config struct {
	MinTurnRadius   float64 // 差速底盘最小转弯半径（米）
	CurvatureWeight float64 // 曲率惩罚权重
	SwitchMargin    float64 // 重规划滞后阈值：新路径代价须优于旧路径该比例才切换
	ObstacleCost    uint8   // 视为致命障碍的代价阈值
}
