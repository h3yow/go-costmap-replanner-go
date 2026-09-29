# go-costmap-replanner-go

移动机器人**局部代价地图 + 动态重规划**的最小完整实现。使用本地确定性传感器
模拟数据把整条链路跑通：

```
多路传感器读数(并发) → 校验/拒绝/降级 → 原子融合进代价地图(含衰减/膨胀)
        → Hybrid A* 差速约束规划 → 去抖+滞回的稳定重规划 → 可执行路径
```

## 包结构

| 包 | 职责 |
|---|---|
| `geom` | 二维向量/位姿、角度归一化与坐标变换 |
| `costmap` | 并发安全的二维栅格代价地图：读数接入、时效/完整性校验、占用累积、时间衰减、障碍膨胀、不可变快照 |
| `kinematics` | 差速底盘运动学：最小转弯半径约束、前进/倒车圆弧与原地旋转运动基元 |
| `planner` | Hybrid A* 搜索：状态为 `(列,行,航向箱)`，只走满足运动学的基元；返回可解释成功/失败 |
| `replanner` | 已提交路径维护：阻塞去抖、代价滞回切换、偏航重规划，失败时保留旧路径 |
| `sim` | 确定性多传感器模拟：独立种子 PRNG、可调噪声/周期/量程、异常读数构造器 |
| `integration` | 端到端集成测试（并发、异常链路、动态障碍、不可达、可复现） |
| `cmd/demo` | 本地端到端演示 |

## 坐标与时间约定

- **坐标**：右手局部地图系，原点在地图中心，+X 朝前、+Y 朝左；角度相对 +X
  逆时针为正（弧度，归一化到 `[-π, π)`）。默认地图 20m×20m、分辨率 0.1m。
- **时间**：每条读数带绝对时间戳，接入时另传判定用 `now`。
  - `now - ts > MaxAge`（默认 2s）→ 过期拒绝；
  - `ts - now > MaxFutureSkew`（默认 100ms）→ 未来时间戳拒绝；
  - 同一路源后到读数时间戳更早 → 乱序拒绝（按源分别判定，互不影响）。

## 代价地图更新规则

每格维护**原始占用层 Raw** 与**膨胀层 Cells**，二者在同一把写锁内一起更新：

- 有效命中 `Raw += HitIncrement × weight`（默认 +55，封顶 100）；
  低置信度读数降级接受，`weight=confidence`；
- 障碍按 `InflationRadius`（默认 0.3m）膨胀，膨胀半径内致命，外围线性衰减；
- `Decay(now)` 按 `DecayPerSecond`（默认 20/s）衰减：单周期噪声会快速消退，
  持续 2~3 个采样周期的真实观测才累积为致命障碍；
- `IngestMany` 为**原子批量**：全部校验通过后在同一临界区顺序生效，版本号
  每批只增一次；整批无效则地图完全不变；
- `Snapshot()` 返回双层数组深拷贝，是不可变值——规划期间地图状态不会变化。

### 读数处置原因（均显式返回，绝不静默丢弃）

| 原因 | 含义 | 是否入图 |
|---|---|---|
| `accepted` | 正常接受 | 是 |
| `corrupt` | NaN/Inf 坐标、零时间戳、置信度越界 | 否 |
| `empty` | 不含观测点 | 否 |
| `unknown_sensor` | 源未注册 | 否 |
| `stale` / `future_timestamp` / `out_of_order` | 过期 / 未来漂移 / 乱序 | 否 |
| `out_of_bounds` | 全部越界则拒绝；部分越界则裁剪后**降级** | 部分 |
| `low_confidence_degraded` | 置信度 < 0.5，权重按置信度衰减 | 是（降级） |

## 规划器与运动学约束

- 差速底盘允许**原地旋转**与沿固定曲率圆弧行驶；转向角集合换算出的转弯半径
  不得小于 `MinTurnRadius`（默认 1m），违反约束的基元在构造期即报错。
- Hybrid A* 状态 `(col,row,heading_bin)`，扩展顺序、并列打破规则（f→g→入队序）
  均固定，**相同 `(start, goal, 快照)` 必返回相同路径与代价**。
- 代价 = 弧长 + 转向角 + 倒车/换挡惩罚 + 沿途代价地图代价；碰撞检测沿弧段
  以分辨率为步长等距采样。
- 搜索前先用忽略航向的 8 邻域 BFS 计算几何距离（同时作为启发式），据此把
  “不可达”细分为端点阻塞 / 被包围 / 区域隔离。

### 规划失败原因

| 原因 | 含义 |
|---|---|
| `start_blocked` / `goal_blocked` | 起点/目标位于致命代价区或地图外（绝不返回看似成功的空路径） |
| `start_encircled` | 起点自由连通区封闭且面积不足（被障碍完全包围） |
| `goal_encircled` | 目标自由连通区封闭且与起点隔离（被完全包围） |
| `unreachable` | 几何自由空间连通但受最小转弯半径/航向箱约束不可达，或两区隔离 |
| `timeout` | 超过展开节点数或时间预算 |

成功时 `Result` 带路径、总代价、展开节点数与说明；路径点逐个可查代价。

## 重规划触发条件与稳定性

`replanner.Update(快照, 当前位姿, 目标, now)` 每个感知周期调用一次，基于
**单个快照版本**做判定，返回 `Trigger`：

| Trigger | 条件 |
|---|---|
| `first_plan` | 尚无已提交路径 |
| `path_blocked` | 路径前方（跳过脚下 `ReplanLookahead` 个点）同一点**连续 N 个周期**被观测为致命（默认 N=3） |
| `deviated` | 机器人实际位姿偏离已提交路径超过 `DeviationTol`（默认 0.5m） |
| `better_alternative` | （可选定期重规划）候选路径比**剩余**已提交路径代价低出 `HysteresisRatio`（默认 15%）以上 |
| `none` | 均不满足，维持已提交路径 |

稳定性设计：

1. **去抖**：每个路径点维护“连续阻塞计数”，任一周期恢复自由立即清零——
   单/双周期的噪声障碍不会引发重规划；
2. **滞回**：替代路径必须显著更优（默认便宜 15%）才切换，避免近似路径来回摇摆；
3. **失败保留**：重规划失败时保留旧路径，并在 `Status.Reason/Detail` 给出
   可区分原因，成功或失败都可解释。

## 本地验证

需要 Go 1.25+，无第三方依赖。

```bash
# 全量单元/集成测试（建议带竞态检测）
go test ./... -race

# 仅看详细日志（含关键输入与判定依据）
go test ./... -race -v

# 端到端演示：约 12 秒，实时展示融合、异常拒绝、动态障碍触发绕行直至到达
go run ./cmd/demo
```

演示会注入：过期、损坏（NaN）、全部/部分越界、未注册源、低置信度、乱序
读数，并在结尾打印每种处置原因的出现次数；动态障碍在 2s~4.8s 堵住门洞，
约在 3.2s 观察到连续确认的阻塞并触发一次 `path_blocked` 重规划绕行。

### 测试覆盖对照

| 需求 | 测试 |
|---|---|
| 乱序/过期/越界/损坏/空/未注册/低置信度 | `costmap.TestRejectReasons`、`TestOutOfOrder`、`integration.TestConcurrentPipelineConsistency` |
| 并发读写下地图一致（-race） | `costmap.TestConcurrentConsistency`、`integration.TestConcurrentPipelineConsistency` |
| 批量融合原子性（全无效不变/部分有效版本+1） | `costmap.TestIngestManyAtomicity` |
| 时间衰减 | `costmap.TestDecay` |
| 静态墙绕行、动态障碍改道 | `planner.TestPlanAvoidsStaticWall`、`TestDynamicObstacleChangesResult` |
| 噪声抖动不切换、持续障碍才重规划 | `replanner.TestNoiseJitterDoesNotFlip`、`TestPersistentBlockTriggersReplan`、`integration.TestDynamicChangeDuringReplanning` |
| 目标/起点被包围、不可达明确失败 | `planner.TestGoalEncircled/StartEncircled`、`replanner.TestReplanFailureKeepsReasonAndOldPath`、`integration.TestUnreachableGoalEndToEnd` |
| 最小转弯半径约束 | `kinematics.TestPrimitivesRespectMinRadius`、`TestApplyGeometry` |
| 结论可复现 | `planner.TestDeterminism`、`sim.TestDeterministicOutput`、`integration.TestReproducibleEndToEnd` |
