package planner

import (
	"container/heap"
	"fmt"
	"math"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
)

// Planner 基于代价地图快照做 A* 规划，带滞后（hysteresis）抑制路径抖动。
// Planner 仅保存上一条采纳的路径用于稳定性比较，不持有地图引用。
type Planner struct {
	cfg      Config
	lastPath []Point
	lastCost float64
	hasLast  bool
}

// New 创建规划器。
func New(cfg Config) *Planner {
	if cfg.ObstacleCost == 0 {
		cfg.ObstacleCost = 254
	}
	return &Planner{cfg: cfg}
}

// gridCtx 把快照与配置打包，提供确定性的栅格访问。
type gridCtx struct {
	snap costmap.Snapshot
	cfg  costmap.Config
}

func (g *gridCtx) idx(gx, gy int) int { return gy*g.snap.Width + gx }

func (g *gridCtx) worldToGrid(x, y float64) (int, int, bool) {
	gx := int(math.Floor((x - g.cfg.OriginX) / g.cfg.Resolution))
	gy := int(math.Floor((y - g.cfg.OriginY) / g.cfg.Resolution))
	ok := gx >= 0 && gy >= 0 && gx < g.snap.Width && gy < g.snap.Height
	return gx, gy, ok
}

func (g *gridCtx) gridToWorld(gx, gy int) Point {
	return Point{
		X: g.cfg.OriginX + (float64(gx)+0.5)*g.cfg.Resolution,
		Y: g.cfg.OriginY + (float64(gy)+0.5)*g.cfg.Resolution,
	}
}

// lethal 判断栅格是否为致命障碍。
func (g *gridCtx) lethal(gx, gy int, threshold uint8) bool {
	return g.snap.Cells[g.idx(gx, gy)] >= threshold
}

// stepCost 进入栅格的代价：距离 * (1 + 代价惩罚)，惩罚随栅格代价线性增长。
func (g *gridCtx) stepCost(gx, gy int, dist float64) float64 {
	c := float64(g.snap.Cells[g.idx(gx, gy)])
	return dist * (1 + c/64)
}

// 固定的 8 邻域扩展顺序，保证搜索结果确定可复现。
var nbrs = [8][2]int{{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}

type node struct {
	gx, gy int
	f, h   float64
	seq    int // 入堆序号，最终 tie-break，保证确定性
}

// pq 是最小堆，排序键 (f, h, seq) 全序，保证同样的输入弹出顺序完全一致。
type pq []node

func (q pq) Len() int { return len(q) }
func (q pq) Less(i, j int) bool {
	if q[i].f != q[j].f {
		return q[i].f < q[j].f
	}
	if q[i].h != q[j].h {
		return q[i].h < q[j].h
	}
	return q[i].seq < q[j].seq
}
func (q pq) Swap(i, j int)      { q[i], q[j] = q[j], q[i] }
func (q *pq) Push(x interface{}) { *q = append(*q, x.(node)) }
func (q *pq) Pop() interface{} {
	old := *q
	n := len(old)
	it := old[n-1]
	*q = old[:n-1]
	return it
}

// Plan 在给定地图快照上规划从 Start 到 Goal 的路径。
// 同样的 (快照, 请求, 规划器历史) 输入必然得到同样的结论（确定性）：
// 邻域扩展顺序固定，堆比较键为全序，无任何随机性与 map 迭代依赖。
func (p *Planner) Plan(snap costmap.Snapshot, cfg costmap.Config, req PlanRequest) PlanResult {
	g := &gridCtx{snap: snap, cfg: cfg}
	res := PlanResult{Version: snap.Version}

	sx, sy, ok := g.worldToGrid(req.Start.X, req.Start.Y)
	if !ok {
		res.Reason = FailStartInvalid
		res.Detail = fmt.Sprintf("起点 (%.2f, %.2f) 在地图外", req.Start.X, req.Start.Y)
		return res
	}
	if g.lethal(sx, sy, p.cfg.ObstacleCost) {
		res.Reason = FailStartInvalid
		res.Detail = fmt.Sprintf("起点所在栅格 (%d, %d) 为致命障碍", sx, sy)
		return res
	}
	gx, gy, ok := g.worldToGrid(req.Goal.X, req.Goal.Y)
	if !ok {
		res.Reason = FailGoalInvalid
		res.Detail = fmt.Sprintf("目标 (%.2f, %.2f) 在地图外", req.Goal.X, req.Goal.Y)
		return res
	}
	if g.lethal(gx, gy, p.cfg.ObstacleCost) {
		res.Reason = FailGoalInvalid
		res.Detail = fmt.Sprintf("目标所在栅格 (%d, %d) 为致命障碍", gx, gy)
		return res
	}

	path, cost, explored, found := p.astar(g, sx, sy, gx, gy)
	res.Explored = explored
	if !found {
		if p.goalEnclosed(g, gx, gy) {
			res.Reason = FailGoalUnreachable
			res.Detail = fmt.Sprintf("目标栅格 (%d, %d) 被致命障碍完全包围", gx, gy)
		} else {
			res.Reason = FailNoProgressStable
			res.Detail = fmt.Sprintf("搜索扩展 %d 个节点后耗尽，起点与目标不连通", explored)
		}
		return res
	}

	// 曲率约束：先视线拉直，再逐步回插中点修复超限转弯。
	maxK := maxCurvature(p.cfg.MinTurnRadius)
	smoothed := p.smooth(g, path)
	smoothed = p.repairCurvature(smoothed, path, maxK)
	if !pathCurvatureOK(withHeading(smoothed, req), maxK) {
		res.Reason = FailCurvature
		res.Detail = fmt.Sprintf("路径最小转弯半径 %.2f m 无法满足底盘约束 %.2f m",
			1/maxK, p.cfg.MinTurnRadius)
		return res
	}

	// 滞后切换：旧路径仍可行且新路径未明显更优时，保留旧路径，抑制抖动。
	if p.hasLast && p.pathStillValid(g, p.lastPath) {
		if cost >= p.lastCost*(1-p.cfg.SwitchMargin) {
			res.OK = true
			res.Path = p.lastPath
			res.Cost = p.lastCost
			res.Detail = fmt.Sprintf("保留上一条路径（新代价 %.2f 未优于旧代价 %.2f 的滞后阈值）", cost, p.lastCost)
			return res
		}
	}
	p.lastPath, p.lastCost, p.hasLast = smoothed, cost, true

	res.OK = true
	res.Path = smoothed
	res.Cost = cost
	res.Detail = fmt.Sprintf("规划成功：%d 个路径点，总代价 %.2f", len(smoothed), cost)
	return res
}

// astar 在栅格上做 8 连通 A*，返回世界坐标路径（栅格中心）与总代价。
func (p *Planner) astar(g *gridCtx, sx, sy, gx, gy int) ([]Point, float64, int, bool) {
	w, h := g.snap.Width, g.snap.Height
	const inf = math.MaxFloat64
	gScore := make([]float64, w*h)
	came := make([]int, w*h)
	closed := make([]bool, w*h)
	for i := range gScore {
		gScore[i] = inf
		came[i] = -1
	}
	start := g.idx(sx, sy)
	goal := g.idx(gx, gy)
	gScore[start] = 0

	open := &pq{{gx: sx, gy: sy, f: 0, h: 0}}
	heap.Init(open)
	seq := 0
	explored := 0
	// 安全上限：理论上每格只出堆一次，上限防御性兜底。
	maxExpand := w * h * 4

	for open.Len() > 0 && explored < maxExpand {
		cur := heap.Pop(open).(node)
		ci := g.idx(cur.gx, cur.gy)
		if closed[ci] {
			continue
		}
		closed[ci] = true
		explored++
		if ci == goal {
			return p.reconstruct(g, came, goal, gScore[goal]), gScore[goal], explored, true
		}
		for _, d := range nbrs {
			nx, ny := cur.gx+d[0], cur.gy+d[1]
			if nx < 0 || ny < 0 || nx >= w || ny >= h {
				continue
			}
			ni := g.idx(nx, ny)
			if closed[ni] || g.lethal(nx, ny, p.cfg.ObstacleCost) {
				continue
			}
			// 禁止穿过两个对角障碍的夹缝。
			if d[0] != 0 && d[1] != 0 &&
				g.lethal(cur.gx+d[0], cur.gy, p.cfg.ObstacleCost) &&
				g.lethal(cur.gx, cur.gy+d[1], p.cfg.ObstacleCost) {
				continue
			}
			dist := math.Hypot(float64(d[0]), float64(d[1])) * g.cfg.Resolution
			tentative := gScore[ci] + g.stepCost(nx, ny, dist)
			if tentative < gScore[ni] {
				gScore[ni] = tentative
				came[ni] = ci
				h := math.Hypot(float64(nx-gx), float64(ny-gy)) * g.cfg.Resolution
				seq++
				heap.Push(open, node{gx: nx, gy: ny, f: tentative + h, h: h, seq: seq})
			}
		}
	}
	return nil, 0, explored, false
}

// reconstruct 回溯父指针生成世界坐标路径。
func (p *Planner) reconstruct(g *gridCtx, came []int, goal int, cost float64) []Point {
	var rev []Point
	w := g.snap.Width
	for i := goal; i != -1; i = came[i] {
		rev = append(rev, g.gridToWorld(i%w, i/w))
	}
	path := make([]Point, len(rev))
	for i, pt := range rev {
		path[len(rev)-1-i] = pt
	}
	return path
}

// goalEnclosed 判断目标栅格是否被致命障碍（或边界）完全包围。
func (p *Planner) goalEnclosed(g *gridCtx, gx, gy int) bool {
	for _, d := range nbrs {
		nx, ny := gx+d[0], gy+d[1]
		if nx < 0 || ny < 0 || nx >= g.snap.Width || ny >= g.snap.Height {
			continue
		}
		if !g.lethal(nx, ny, p.cfg.ObstacleCost) {
			return false
		}
	}
	return true
}

// lineOfSight 检查两栅格间直线是否穿过致命障碍（Bresenham）。
func (p *Planner) lineOfSight(g *gridCtx, x0, y0, x1, y1 int) bool {
	dx := int(math.Abs(float64(x1 - x0)))
	dy := int(math.Abs(float64(y1 - y0)))
	sx, sy := 1, 1
	if x0 > x1 {
		sx = -1
	}
	if y0 > y1 {
		sy = -1
	}
	err := dx - dy
	x, y := x0, y0
	for {
		if g.lethal(x, y, p.cfg.ObstacleCost) {
			return false
		}
		if x == x1 && y == y1 {
			return true
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x += sx
		}
		if e2 < dx {
			err += dx
			y += sy
		}
	}
}

// smooth 视线拉直：贪心跳过中间点，结果确定。
func (p *Planner) smooth(g *gridCtx, path []Point) []Point {
	if len(path) <= 2 {
		return path
	}
	out := []Point{path[0]}
	i := 0
	for i < len(path)-1 {
		j := len(path) - 1
		for j > i+1 {
			a, b := path[i], path[j]
			ax, ay, _ := g.worldToGrid(a.X, a.Y)
			bx, by, _ := g.worldToGrid(b.X, b.Y)
			if p.lineOfSight(g, ax, ay, bx, by) {
				break
			}
			j--
		}
		out = append(out, path[j])
		i = j
	}
	return out
}

// repairCurvature 对拉直后的路径，在曲率超限处回插原始路径的中点，
// 直到满足约束或达到确定性上限。
func (p *Planner) repairCurvature(smoothed, original []Point, maxK float64) []Point {
	if math.IsInf(maxK, 1) {
		return smoothed
	}
	cur := smoothed
	for iter := 0; iter < 32; iter++ {
		worst, worstK := -1, 0.0
		for i := 1; i+1 < len(cur); i++ {
			if k := mengerCurvature(cur[i-1], cur[i], cur[i+1]); k > maxK && k > worstK {
				worst, worstK = i, k
			}
		}
		if worst < 0 {
			return cur
		}
		// 在最差拐点处插入原始路径中离该点最近的点（确定性选择）。
		anchor := cur[worst]
		best, bestD := -1, math.MaxFloat64
		for i, pt := range original {
			d := math.Hypot(pt.X-anchor.X, pt.Y-anchor.Y)
			if d < bestD {
				best, bestD = i, d
			}
		}
		if best < 0 {
			return cur
		}
		next := make([]Point, 0, len(cur)+1)
		next = append(next, cur[:worst]...)
		next = append(next, original[best])
		next = append(next, cur[worst:]...)
		cur = next
	}
	return cur
}

// withHeading 在路径前补一个沿当前航向的虚拟前驱点，
// 使第一个转弯也纳入曲率检查。
func withHeading(path []Point, req PlanRequest) []Point {
	if len(path) < 2 {
		return path
	}
	step := math.Hypot(path[1].X-path[0].X, path[1].Y-path[0].Y)
	prev := Point{
		X: path[0].X - math.Cos(req.Heading)*step,
		Y: path[0].Y - math.Sin(req.Heading)*step,
	}
	return append([]Point{prev}, path...)
}

// pathStillValid 检查旧路径在当前快照上是否仍然可行（不碰致命障碍）。
func (p *Planner) pathStillValid(g *gridCtx, path []Point) bool {
	for i := 0; i+1 < len(path); i++ {
		ax, ay, okA := g.worldToGrid(path[i].X, path[i].Y)
		bx, by, okB := g.worldToGrid(path[i+1].X, path[i+1].Y)
		if !okA || !okB || !p.lineOfSight(g, ax, ay, bx, by) {
			return false
		}
	}
	return true
}
