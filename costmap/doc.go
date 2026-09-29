// Package costmap 的坐标、时间与更新规则约定。
//
// # 坐标系
//
// 采用右手局部地图坐标系：原点位于地图中心，+X 指向机器人前方，+Y 指向左侧，
// 角度相对 +X 轴逆时针为正（弧度，归一化区间 [-π, π)）。地图以原点为中心
// 占据 Width × Height 平方米，按 Resolution 划分为规则栅格；行 row 对应 y
// 由小到大，列 col 对应 x 由小到大。
//
// 世界坐标 (x,y) -> 网格：
//
//	col = floor((x + Width/2)  / Resolution)
//	row = floor((y + Height/2) / Resolution)
//
// 网格中心 -> 世界坐标见 CellToWorld。落在地图范围外的点不进图；快照查询时
// 地图外一律按“致命/阻塞”保守处理（Snapshot.BlockedAt），避免规划器把机器人
// 引向未知区域。
//
// # 时间约定
//
// 每条 Reading 带绝对时间戳；Ingest/IngestMany 额外接收一个判定用当前时间 now：
//
//   - now - Timestamp > MaxAge        -> RejectStale（过期）
//   - Timestamp - now > MaxFutureSkew -> RejectFuture（时钟未来漂移）
//   - 同一传感器后到读数时间戳早于已接受读数 -> RejectOutOfOrder（乱序）
//
// 乱序判定按“传感器源”分别维护：不同传感器的时钟互不影响。被拒绝的读数不
// 推进该源时间戳；被接受（含降级接受）的读数才推进。
//
// # 代价模型与更新规则
//
// 每个格子维护两层数据，且两层始终在同一把写锁内一起更新，读者永远看不到
// “更新了一半”的状态：
//
//   - Raw  原始占用层，代价区间 [0, LethalCost]；
//   - Cells 膨胀层（快照中对外暴露），由 Raw 按 InflationRadius 重算：
//     致命障碍的膨胀半径内一律致命；非致命占用按到障碍距离线性衰减扩散。
//
// 更新规则：
//
//   - 有效命中：Raw[cell] += HitIncrement × weight，封顶 LethalCost；
//     weight=1（正常）或 confidence（RejectLowConfidence 降级读数）。
//   - 部分越界：越界点被裁剪，剩余点以 RejectOutOfBounds+Degraded 降级融合。
//   - 时间衰减：Decay(now) 使每格 Raw 减去 DecayPerSecond×Δt（不为负），
//     随后重建膨胀层。因此“一闪而过”的噪声障碍会快速消退，持续命中的真实
//     障碍才会累积到致命阈值（默认 10Hz 激光下约 2~3 个采样周期致命）。
//   - IngestMany 原子批量：全部读数先校验、再在同一临界区顺序生效，版本号
//     在整批完成后只递增一次；若整批全部无效，地图完全不变。
//
// # 拒绝/降级原因
//
// 任何读数都返回显式处置结果（IngestResult），绝不静默丢弃：
//
//   - RejectNone          正常接受；
//   - RejectCorrupt       时间戳零值、置信度非法或坐标含 NaN/Inf；
//   - RejectEmpty         不含任何点；
//   - RejectUnknownSensor 源未通过 RegisterSource 注册；
//   - RejectStale/RejectFuture/RejectOutOfOrder 见“时间约定”；
//   - RejectOutOfBounds   全部点越界（拒绝）或部分越界（降级，Degraded=true）；
//   - RejectLowConfidence 置信度低于阈值，降级接受且命中权重=置信度。
//
// 通过 IngestResult.Accepted() 可区分“拒绝（未入图）”与“降级（已入图但有
// 说明）”；Detail 字段给出人类可读的判定依据。
//
// # 并发
//
// Grid 内部使用 sync.RWMutex：多个 Snapshot/CostAt/IsBlocked 读者可并发，
// 所有 Ingest/Decay 写操作互斥并与读者互斥。Snapshot 返回双层数组的深拷贝，
// 是不可变值，规划/重规划可在不加锁的情况下基于它做确定性计算。
package costmap
