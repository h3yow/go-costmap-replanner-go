// Package planner 在局部代价地图上做满足差速底盘约束的 Hybrid A* 搜索。
//
// 搜索在离散状态 (网格列, 网格行, 航向箱) 上进行，状态转移只使用
// kinematics 包生成的、满足最小转弯半径的运动基元，因此输出路径天然可执行。
// 结果完全由 (start, goal, 地图快照) 决定：固定的基元顺序与确定性的
// 并列打分打破规则，保证相同输入复现相同结论。
package planner

import (
	"container/heap"
	"fmt"
	"math"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
	"github.com/h3yow/go-costmap-replanner-go/geom"
	"github.com/h3yow/go-costmap-replanner-go/kinematics"
)

// FailReason 是可区分的规划失败原因。
type FailReason string

const (
	FailNone           FailReason = ""
	FailStartBlocked   FailReason = "start_blocked"   // 起点位于致命代价区（或地图外）
	FailGoalBlocked    FailReason = "goal_blocked"    // 目标位于致命代价区（或地图外）
	FailStartEncircled FailReason = "start_encircled" // 起点自由连通区被障碍封闭
	FailGoalEncircled  FailReason = "goal_encircled"  // 目标被障碍完全包围
	FailUnreachable    FailReason = "unreachable"     // 自由空间连通但受运动学约束不可达，或区域被墙隔离
	FailTimeout        FailReason = "timeout"         // 超出展开/时间预算
)

// Options 控制 Hybrid A* 搜索。
type Options struct {
	// Timeout 已弃用：为保证结论可复现，预算只看 MaxExpansions，不看墙钟。
	Timeout           time.Duration //nolint:unused
	MaxExpansions     int
	HeadingBins       int
	ReversePenalty    float64
	SwitchPenalty     float64
	TurningCostFactor float64
	CostmapWeight     float64
	StepCostFactor    float64
}

// DefaultOptions 返回一组默认搜索参数。
func DefaultOptions() Options {
	return Options{
		Timeout:           2 * time.Second,
		MaxExpansions:     200000,
		HeadingBins:       36,
		ReversePenalty:    5.0,
		SwitchPenalty:     8.0,
		TurningCostFactor: 2.0,
		CostmapWeight:     0.5,
		StepCostFactor:    1.0,
	}
}

// Waypoint 路径上的一个位姿点。
type Waypoint struct {
	Pose      geom.Pose
	Cost      float64 // 到该点累计的实际代价
	Heuristic float64 // 该点到目标估计代价（解释用）
	Steering  float64 // 到达该点使用的基元转向角
	Reverse   bool
}

// Result 是一次规划的结果；失败时 Reason != FailNone。
type Result struct {
	Path     []Waypoint
	Success  bool
	Reason   FailReason
	Detail   string
	Cost     float64 // 成功时路径总代价
	Expanded int     // 展开节点数（解释用）
}

// Planner 基于 Hybrid A* 的约束路径规划器。
type Planner struct {
	k     kinematics.Params
	opts  Options
	prims []kinematics.Primitive
}

// New 创建 Hybrid A* 规划器。
func New(k kinematics.Params, opts Options) (*Planner, error) {
	prims, err := kinematics.Primitives(k)
	if err != nil {
		return nil, err
	}
	if opts.HeadingBins <= 0 {
		opts = DefaultOptions()
	}
	return &Planner{k: k, opts: opts, prims: prims}, nil
}

type searchNode struct {
	pose     geom.Pose
	state    stateKey
	cell     costmap.Cell
	g, f     float64
	parent   *searchNode
	steering float64
	reverse  bool
	depth    int
	index    int // heap 索引
}

type stateKey struct {
	col, row, hbin int
}

type pq []*searchNode

func (q pq) Len() int { return len(q) }
func (q pq) Less(i, j int) bool {
	// 确定性并列打破：f 相同则 g 小的优先，再相同则先入队的优先。
	if q[i].f != q[j].f {
		return q[i].f < q[j].f
	}
	if q[i].g != q[j].g {
		return q[i].g < q[j].g
	}
	return q[i].index < q[j].index
}
func (q pq) Swap(i, j int)       { q[i], q[j] = q[j], q[i] }
func (q *pq) Push(x interface{}) { *q = append(*q, x.(*searchNode)) }
func (q *pq) Pop() interface{} {
	old := *q
	n := len(old)
	it := old[n-1]
	*q = old[:n-1]
	return it
}

// Plan 在给定地图快照上执行确定性搜索。
func (pl *Planner) Plan(snap costmap.Snapshot, start, goal geom.Pose) Result {
	// —— 1. 端点合法性 ——
	if snap.BlockedAt(start.Pos.X, start.Pos.Y) {
		return Result{Success: false, Reason: FailStartBlocked,
			Detail: fmt.Sprintf("起点 (%.2f,%.2f) 位于致命代价区或地图外", start.Pos.X, start.Pos.Y)}
	}
	if snap.BlockedAt(goal.Pos.X, goal.Pos.Y) {
		return Result{Success: false, Reason: FailGoalBlocked,
			Detail: fmt.Sprintf("目标 (%.2f,%.2f) 位于致命代价区或地图外", goal.Pos.X, goal.Pos.Y)}
	}

	// —— 2. 忽略航向的自由空间 BFS：启发式 + 连通性诊断 ——
	hmap, reachable := bfsDistance(snap, goal, pl.k.XYGoalTol)
	startCell, _ := worldCell(snap, start.Pos)
	goalCell, _ := worldCell(snap, goal.Pos)
	if _, ok := hmap[startCell]; !ok {
		reason, detail := pl.classifyUnreachable(snap, startCell, goalCell, reachable)
		return Result{Success: false, Reason: reason, Detail: detail}
	}

	// —— 3. Hybrid A* ——
	// 预算只用“展开节点数”这一确定性量，不使用墙钟，从而保证相同输入
	// （相同快照/起终点）在任何机器负载下都得到完全一致的结论。
	headingAt := func(h float64) float64 {
		return math.Abs(geom.AngleDiff(h, goal.H))
	}
	heuristic := func(pose geom.Pose, c costmap.Cell) float64 {
		hd := hmap[c]
		return hd*snap.Config.Resolution*pl.opts.StepCostFactor + headingAt(pose.H)*pl.opts.TurningCostFactor
	}

	startNode := &searchNode{
		pose:  start,
		state: pl.key(start, startCell),
		cell:  startCell,
	}
	startNode.f = heuristic(start, startCell)

	best := map[stateKey]*searchNode{startNode.state: startNode}
	queue := &pq{startNode}
	heap.Init(queue)
	enqueueSeq := 0

	expanded := 0
	for queue.Len() > 0 {
		if expanded >= pl.opts.MaxExpansions {
			return Result{Success: false, Reason: FailTimeout, Expanded: expanded,
				Detail: fmt.Sprintf("展开预算耗尽（%d 个节点）仍未找到路径", expanded)}
		}
		cur := heap.Pop(queue).(*searchNode)
		if best[cur.state] != cur {
			continue // 同一离散状态已有更优路径，跳过过期队列项
		}
		expanded++

		// 连续空间到达判定（位置与航向容差）。
		if kinematics.GoalReached(cur.pose, goal, pl.k) {
			path, total := pl.reconstruct(cur, hmap, snap)
			return Result{Success: true, Path: path, Cost: total, Expanded: expanded,
				Detail: fmt.Sprintf("搜索成功：展开 %d 个节点，路径 %d 点，总代价 %.2f",
					expanded, len(path), total)}
		}

		for _, pr := range pl.prims {
			nextPose := kinematics.Apply(cur.pose, pr, pl.k)
			if !collisionFree(snap, cur.pose, pr, pl.k) {
				continue
			}
			nextCell, ok := worldCell(snap, nextPose.Pos)
			if !ok {
				continue
			}
			nkey := pl.key(nextPose, nextCell)
			if _, dup := best[nkey]; dup {
				// 离散状态已被到达；由于扩展顺序固定且代价非负，
				// 先到的路径即为最优，保持稳定不重复打开。
				continue
			}
			step := primitiveCost(snap, cur.pose, pr, pl.k, pl.opts)
			if pr.Reverse && !cur.reverse && cur.depth > 0 {
				step += pl.opts.SwitchPenalty
			}
			if cur.depth == 0 && pr.Reverse {
				step += pl.opts.SwitchPenalty
			}
			node := &searchNode{
				pose: nextPose, state: nkey, cell: nextCell,
				g: cur.g + step, parent: cur,
				steering: pr.SteerAngle, reverse: pr.Reverse,
				depth: cur.depth + 1,
			}
			node.f = node.g + heuristic(nextPose, nextCell)
			node.index = enqueueSeq
			enqueueSeq++
			best[nkey] = node
			heap.Push(queue, node)
		}
	}

	// BFS 几何可达但 Hybrid A* 未找到：受运动学约束不可达。
	return Result{Success: false, Reason: FailUnreachable, Expanded: expanded,
		Detail: fmt.Sprintf("目标在几何自由空间内，但在航向箱=%d/最小转弯半径%.2fm约束下扩展 %d 个节点仍不可达",
			pl.opts.HeadingBins, pl.k.MinTurnRadius, expanded)}
}

// classifyUnreachable 区分起点/目标被包围与普通不可达。
func (pl *Planner) classifyUnreachable(snap costmap.Snapshot, startCell, goalCell costmap.Cell, reachable map[costmap.Cell]bool) (FailReason, string) {
	startComp, startOpen := floodComponent(snap, startCell)
	goalComp, goalOpen := floodComponent(snap, goalCell)
	if !startOpen && len(startComp) <= enclosedCellBudget(pl.k, snap) {
		return FailStartEncircled, fmt.Sprintf(
			"起点自由连通区仅 %d 格且不触及地图边界，被障碍完全包围", len(startComp))
	}
	if !goalOpen && len(goalComp) <= enclosedCellBudget(pl.k, snap) {
		return FailGoalEncircled, fmt.Sprintf(
			"目标自由连通区仅 %d 格且不触及地图边界，被障碍完全包围", len(goalComp))
	}
	if !goalOpen {
		return FailGoalEncircled, fmt.Sprintf(
			"目标所在自由区不触及地图边界且与起点隔离（%d 格），判定为被包围", len(goalComp))
	}
	return FailUnreachable, fmt.Sprintf(
		"起点与目标的自由区域均开放但互不连通（起点区 %d 格，目标区 %d 格），路径不可达",
		len(startComp), len(goalComp))
}

// enclosedCellBudget 给出“封闭口袋”大小上限：以最小转弯半径估算转身所需面积。
func enclosedCellBudget(k kinematics.Params, snap costmap.Snapshot) int {
	need := 2 * k.MinTurnRadius
	return int(math.Ceil((need/snap.Config.Resolution)*(need/snap.Config.Resolution))) + 8
}

func (pl *Planner) key(pose geom.Pose, c costmap.Cell) stateKey {
	bin := int(math.Floor((geom.NormAngle(pose.H) + math.Pi) / (2 * math.Pi) * float64(pl.opts.HeadingBins)))
	bin %= pl.opts.HeadingBins
	return stateKey{col: c.Col, row: c.Row, hbin: bin}
}

func (pl *Planner) reconstruct(n *searchNode, hmap map[costmap.Cell]float64, snap costmap.Snapshot) ([]Waypoint, float64) {
	var nodes []*searchNode
	for x := n; x != nil; x = x.parent {
		nodes = append(nodes, x)
	}
	path := make([]Waypoint, 0, len(nodes))
	for i := len(nodes) - 1; i >= 0; i-- {
		x := nodes[i]
		path = append(path, Waypoint{
			Pose:      x.pose,
			Cost:      x.g,
			Heuristic: hmap[x.cell] * snap.Config.Resolution * pl.opts.StepCostFactor,
			Steering:  x.steering,
			Reverse:   x.reverse,
		})
	}
	return path, n.g
}

// worldCell 与快照内部换算保持一致。
func worldCell(snap costmap.Snapshot, p geom.Vec) (costmap.Cell, bool) {
	if !snap.Contains(p.X, p.Y) {
		return costmap.Cell{}, false
	}
	col := int((p.X - snap.OriginX + snap.Config.Width/2) / snap.Config.Resolution)
	row := int((p.Y - snap.OriginY + snap.Config.Height/2) / snap.Config.Resolution)
	if col >= snap.Cols {
		col = snap.Cols - 1
	}
	if row >= snap.Rows {
		row = snap.Rows - 1
	}
	return costmap.Cell{Col: col, Row: row}, true
}

// collisionFree 沿基元弧段等距采样，任一点致命即不可行；地图外同样视为碰撞。
func collisionFree(snap costmap.Snapshot, start geom.Pose, pr kinematics.Primitive, k kinematics.Params) bool {
	samples := int(math.Ceil(k.StepLength/snap.Config.Resolution)) + 1
	if samples < 2 {
		samples = 2
	}
	for i := 1; i <= samples; i++ {
		t := float64(i) / float64(samples)
		var p geom.Pose
		if pr.SteerAngle == kinematics.SteerAngleRotate {
			p = geom.Pose{Pos: start.Pos, H: geom.NormAngle(start.H + pr.TurnAngle*t)}
		} else {
			partial := kinematics.Primitive{
				SteerAngle: pr.SteerAngle,
				Length:     pr.Length * t,
				TurnAngle:  pr.TurnAngle * t,
				Reverse:    pr.Reverse,
			}
			p = kinematics.Apply(start, partial, k)
		}
		if snap.BlockedAt(p.Pos.X, p.Pos.Y) {
			return false
		}
	}
	return true
}

// primitiveCost 估算执行基元的代价：弧长 + 转向 + 倒车 + 沿途代价地图代价。
func primitiveCost(snap costmap.Snapshot, start geom.Pose, pr kinematics.Primitive, k kinematics.Params, opts Options) float64 {
	c := math.Abs(pr.Length) * opts.StepCostFactor
	c += math.Abs(pr.TurnAngle) * opts.TurningCostFactor
	if pr.Reverse {
		c += opts.ReversePenalty
	}
	// 沿途采样代价地图非致命代价（高代价区尽量绕行）。
	samples := int(math.Ceil(k.StepLength/snap.Config.Resolution)) + 1
	sum := 0.0
	for i := 1; i <= samples; i++ {
		t := float64(i) / float64(samples)
		var p geom.Vec
		if pr.SteerAngle == kinematics.SteerAngleRotate {
			p = start.Pos
		} else {
			partial := kinematics.Primitive{SteerAngle: pr.SteerAngle, Length: pr.Length * t, TurnAngle: pr.TurnAngle * t}
			p = kinematics.Apply(start, partial, k).Pos
		}
		if cell, ok := worldCell(snap, p); ok {
			if v, ok := snap.CellCost(cell.Col, cell.Row); ok && v < snap.Config.LethalCost {
				sum += v / snap.Config.LethalCost
			}
		}
	}
	return c + (sum/float64(samples))*opts.CostmapWeight*10
}

// bfsDistance 在非致命网格上做 8 邻域 BFS，返回每个可达格到目标容差区的
// 最近格距离（以格为单位）。第二个返回值标记可达格集合。
func bfsDistance(snap costmap.Snapshot, goal geom.Pose, tol float64) (map[costmap.Cell]float64, map[costmap.Cell]bool) {
	goalCell, _ := worldCell(snap, goal.Pos)
	tolCells := int(math.Ceil(tol / snap.Config.Resolution))
	dist := map[costmap.Cell]float64{}
	visited := map[costmap.Cell]bool{}
	type qitem struct {
		c costmap.Cell
		d float64
	}
	var queue []qitem
	// 容差圈内的格子距离都为 0
	for dr := -tolCells; dr <= tolCells; dr++ {
		for dc := -tolCells; dc <= tolCells; dc++ {
			if math.Hypot(float64(dr), float64(dc)) > float64(tolCells) {
				continue
			}
			c := costmap.Cell{Col: goalCell.Col + dc, Row: goalCell.Row + dr}
			if v, ok := snap.CellCost(c.Col, c.Row); !ok || v >= snap.Config.LethalCost {
				continue
			}
			if !visited[c] {
				visited[c] = true
				dist[c] = 0
				queue = append(queue, qitem{c, 0})
			}
		}
	}
	head := 0
	neigh := [8][2]int{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}
	for head < len(queue) {
		cur := queue[head]
		head++
		for _, n := range neigh {
			nc := costmap.Cell{Col: cur.c.Col + n[0], Row: cur.c.Row + n[1]}
			if visited[nc] {
				continue
			}
			v, ok := snap.CellCost(nc.Col, nc.Row)
			if !ok || v >= snap.Config.LethalCost {
				continue
			}
			// 禁止对角“穿墙角”：两正交邻格都必须自由。
			if n[0] != 0 && n[1] != 0 {
				v1, ok1 := snap.CellCost(cur.c.Col+n[0], cur.c.Row)
				v2, ok2 := snap.CellCost(cur.c.Col, cur.c.Row+n[1])
				if !ok1 || !ok2 || v1 >= snap.Config.LethalCost || v2 >= snap.Config.LethalCost {
					continue
				}
			}
			step := 1.0
			if n[0] != 0 && n[1] != 0 {
				step = math.Sqrt2
			}
			visited[nc] = true
			dist[nc] = cur.d + step
			queue = append(queue, qitem{nc, cur.d + step})
		}
	}
	return dist, visited
}

// floodComponent 返回某格所在自由连通区大小，以及该区域是否触及地图边界。
func floodComponent(snap costmap.Snapshot, start costmap.Cell) (map[costmap.Cell]bool, bool) {
	comp := map[costmap.Cell]bool{}
	if v, ok := snap.CellCost(start.Col, start.Row); !ok || v >= snap.Config.LethalCost {
		return comp, false
	}
	touchesBorder := false
	queue := []costmap.Cell{start}
	comp[start] = true
	neigh := [8][2]int{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}
	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		if cur.Col == 0 || cur.Row == 0 || cur.Col == snap.Cols-1 || cur.Row == snap.Rows-1 {
			touchesBorder = true
		}
		for _, n := range neigh {
			nc := costmap.Cell{Col: cur.Col + n[0], Row: cur.Row + n[1]}
			if comp[nc] {
				continue
			}
			v, ok := snap.CellCost(nc.Col, nc.Row)
			if !ok || v >= snap.Config.LethalCost {
				continue
			}
			comp[nc] = true
			queue = append(queue, nc)
		}
	}
	return comp, touchesBorder
}
