package planner

import "math"

// maxCurvature 返回最小转弯半径对应的最大曲率（1/米）。
// minTurnRadius <= 0 表示无曲率约束。
func maxCurvature(minTurnRadius float64) float64 {
	if minTurnRadius <= 0 {
		return math.Inf(1)
	}
	return 1.0 / minTurnRadius
}

// mengerCurvature 计算三点的外接圆曲率（Menger 曲率）。
// 共线时返回 0；有重复点时返回 +Inf（视为不可行）。
func mengerCurvature(a, b, c Point) float64 {
	ab := math.Hypot(b.X-a.X, b.Y-a.Y)
	bc := math.Hypot(c.X-b.X, c.Y-b.Y)
	ca := math.Hypot(a.X-c.X, a.Y-c.Y)
	if ab == 0 || bc == 0 || ca == 0 {
		return math.Inf(1)
	}
	// 两倍有向面积
	area2 := math.Abs((b.X-a.X)*(c.Y-a.Y) - (c.X-a.X)*(b.Y-a.Y))
	return 2 * area2 / (ab * bc * ca)
}

// pathCurvatureOK 检查折线路径每个中间点的曲率是否不超过 maxK。
// 允许 1% 数值容差，保证判定稳定。
func pathCurvatureOK(path []Point, maxK float64) bool {
	if math.IsInf(maxK, 1) {
		return true
	}
	for i := 1; i+1 < len(path); i++ {
		if mengerCurvature(path[i-1], path[i], path[i+1]) > maxK*1.01 {
			return false
		}
	}
	return true
}
