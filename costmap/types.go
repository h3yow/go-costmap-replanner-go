// Package costmap 提供局部代价地图：并发接入多路传感器读数，
// 校验并融合为栅格代价地图。
package costmap

import "time"

// RejectReason 描述一条读数被拒绝或降级的原因，可区分、可解释。
type RejectReason string

const (
	RejectNone          RejectReason = ""                 // 正常接受
	RejectStale         RejectReason = "stale"            // 时间戳过期
	RejectOutOfOrder    RejectReason = "out_of_order"     // 乱序（比已应用的同传感器读数更旧）
	RejectOutOfBounds   RejectReason = "out_of_bounds"    // 命中点越出地图边界
	RejectCorrupt       RejectReason = "corrupt"          // 数据损坏（NaN/Inf/负距离等）
	RejectFutureStamped RejectReason = "future_timestamp" // 时间戳来自未来
)

// Reading 是一路传感器的一次障碍物观测。
type Reading struct {
	SensorID  string    // 传感器标识，用于乱序检测
	Stamp     time.Time // 观测时间戳
	X, Y      float64   // 障碍物命中点（地图坐标系，米）
	Range     float64   // 传感器到命中点的距离（米），用于校验
	Intensity float64   // 置信度 [0,1]，越低代价增量越小
}

// Decision 是一条读数的处理结论。
type Decision struct {
	Accepted bool
	Reason   RejectReason
	Detail   string
}

// Config 是代价地图配置。
type Config struct {
	Width, Height int     // 栅格数
	Resolution    float64 // 米/栅格
	OriginX       float64 // 地图原点（栅格 (0,0) 角点）在地图坐标系中的 X
	OriginY       float64
	MaxAge        time.Duration // 读数允许的最大年龄
	ObstacleCost  uint8         // 障碍基础代价
	InflateRadius float64       // 膨胀半径（米）
}

// Snapshot 是地图在某一时刻的一致性快照（拷贝，可安全并发读）。
type Snapshot struct {
	Cells   []uint8 // 行优先，len == Width*Height
	Width   int
	Height  int
	Version uint64 // 单调递增版本号，每次成功提交的更新批次 +1
}
