package geom

import "math"

// Vec 表示一个二维向量，单位米。坐标采用机器人局部地图坐标系：
// x 向前、y 向左，右手系，角度相对 +x 轴，逆时针为正。
type Vec struct {
	X float64
	Y float64
}

// Pose 表示带朝向的位姿，Heading 单位弧度。
type Pose struct {
	Pos Vec
	H   float64
}

func (v Vec) Add(o Vec) Vec       { return Vec{X: v.X + o.X, Y: v.Y + o.Y} }
func (v Vec) Sub(o Vec) Vec       { return Vec{X: v.X - o.X, Y: v.Y - o.Y} }
func (v Vec) Scale(s float64) Vec { return Vec{X: v.X * s, Y: v.Y * s} }
func (v Vec) Dot(o Vec) float64   { return v.X*o.X + v.Y*o.Y }

// Len 返回向量长度。
func (v Vec) Len() float64 { return math.Hypot(v.X, v.Y) }

// Dist 返回两点欧氏距离。
func (v Vec) Dist(o Vec) float64 { return math.Hypot(v.X-o.X, v.Y-o.Y) }

// Point 返回位姿的位置分量。
func (p Pose) Point() Vec { return p.Pos }

// NormAngle 把角度归一化到 [-pi, pi)。
func NormAngle(a float64) float64 {
	a = math.Mod(a+math.Pi, 2*math.Pi)
	if a < 0 {
		a += 2 * math.Pi
	}
	return a - math.Pi
}

// AngleDiff 返回 a-b 的最短有符号角差，范围 [-pi, pi)。
func AngleDiff(a, b float64) float64 { return NormAngle(a - b) }

// Transform 将世界/地图坐标 p 变换到以 origin 为原点、x 轴沿 origin.H 的局部坐标系。
func Transform(p, origin Pose) Vec {
	d := p.Pos.Sub(origin.Pos)
	c, s := math.Cos(-origin.H), math.Sin(-origin.H)
	return Vec{X: c*d.X - s*d.Y, Y: s*d.X + c*d.Y}
}

const epsilon = 1e-9
