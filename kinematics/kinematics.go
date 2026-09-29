// Package kinematics 描述差速底盘的可行运动约束。
//
// 差速底盘可以原地旋转，并沿固定曲率的圆弧（含直线）行驶。
// 规划器只使用满足最小转弯半径约束的运动基元，从而保证路径可执行。
// 曲率约束：|曲率| <= 1/MinTurnRadius，即 |转向角| 对应的转弯半径不小于最小值。
package kinematics

import (
	"fmt"
	"math"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/geom"
)

// Params 描述差速底盘的运动学参数。
type Params struct {
	MinTurnRadius   float64   // 最小转弯半径（米），>0
	WheelBase       float64   // 轴距（米），用于转向角/曲率换算
	MaxSpeed        float64   // 最大线速度（米/秒）
	StepLength      float64   // 前进/后退运动基元弧长（米）
	SteerAngles     []float64 // 等效前轮转向角集合（弧度），0 表示直线
	TurnInPlace     bool      // 是否允许原地旋转（差速底盘允许）
	TurnInPlaceStep float64   // 原地旋转角步长（弧度）
	AllowReverse    bool      // 是否允许后退

	// XYGoalTol / YawTol 用于判定到达目标。
	XYGoalTol float64
	YawTol    float64
}

// DefaultParams 返回一组默认差速底盘参数（最小转弯半径 1m）。
func DefaultParams() Params {
	return Params{
		MinTurnRadius:   1.0,
		WheelBase:       0.5,
		MaxSpeed:        1.0,
		StepLength:      0.5,
		SteerAngles:     []float64{-0.45, 0, 0.45},
		TurnInPlace:     true,
		TurnInPlaceStep: math.Pi / 6, // 30°
		AllowReverse:    true,
		XYGoalTol:       0.25,
		YawTol:          math.Pi / 6,
	}
}

// Primitive 是一个固定弧长的运动基元。
type Primitive struct {
	SteerAngle float64 // 等效转向角；SteerAngleRotate 表示原地旋转
	Length     float64 // 弧长（旋转基元为 0）
	TurnAngle  float64 // 基元结束时的航向变化（弧度）
	Reverse    bool
}

// SteerAngleRotate 标记一个原地旋转基元。
const SteerAngleRotate = 1e9

// Duration 返回以 MaxSpeed 执行该基元所需时长（旋转基元按转速估算）。
func (pr Primitive) Duration(p Params) time.Duration {
	if pr.SteerAngle == SteerAngleRotate {
		// 原地旋转的角速度按最小转弯半径处的横摆率估算
		yawRate := p.MaxSpeed / p.MinTurnRadius
		return time.Duration(math.Abs(pr.TurnAngle) / yawRate * float64(time.Second))
	}
	return time.Duration(math.Abs(pr.Length) / p.MaxSpeed * float64(time.Second))
}

// Validate 校验参数自洽性。
func (p Params) Validate() error {
	if p.MinTurnRadius <= 0 {
		return fmt.Errorf("kinematics: MinTurnRadius 必须为正, got %v", p.MinTurnRadius)
	}
	if p.StepLength <= 0 {
		return fmt.Errorf("kinematics: StepLength 必须为正, got %v", p.StepLength)
	}
	if p.MaxSpeed <= 0 {
		return fmt.Errorf("kinematics: MaxSpeed 必须为正, got %v", p.MaxSpeed)
	}
	if p.XYGoalTol <= 0 || p.YawTol <= 0 {
		return fmt.Errorf("kinematics: 到达容差必须为正")
	}
	for _, a := range p.SteerAngles {
		if math.Abs(a) >= math.Pi/2 {
			return fmt.Errorf("kinematics: 转向角 %v 对应零转弯半径，违反约束", a)
		}
	}
	return nil
}

// turnRadius 由自行车模型换算转弯半径 R = L/tan(delta)；直线返回 +inf。
func turnRadius(p Params, steer float64) float64 {
	if math.Abs(steer) < 1e-9 {
		return math.Inf(1)
	}
	return math.Abs(p.WheelBase / math.Tan(steer))
}

// Primitives 依据底盘参数生成确定性的运动基元集合：
// 每个转向角对应前进（及可选后退）圆弧，外加左右两个原地旋转基元。
func Primitives(p Params) ([]Primitive, error) {
	if err := p.Validate(); err != nil {
		return nil, err
	}
	var out []Primitive
	for _, a := range p.SteerAngles {
		if r := turnRadius(p, a); r < p.MinTurnRadius*(1-1e-9) {
			return nil, fmt.Errorf("kinematics: 转向角 %v 对应半径 %.3f < 最小转弯半径 %.3f", a, r, p.MinTurnRadius)
		}
		turn := p.StepLength / turnRadius(p, a) // 弧长/R；直线时为 0
		out = append(out, Primitive{SteerAngle: a, Length: p.StepLength, TurnAngle: turn})
		if p.AllowReverse && !(math.Abs(a) < 1e-9) {
			// 倒车时转向角的几何效果：航向变化方向与前进相反，
			// 为保持基元端点确定性，倒车采用同方向曲率、反向弧长。
			out = append(out, Primitive{
				SteerAngle: a, Length: -p.StepLength, TurnAngle: -turn, Reverse: true,
			})
		} else if p.AllowReverse {
			out = append(out, Primitive{
				SteerAngle: 0, Length: -p.StepLength, TurnAngle: 0, Reverse: true,
			})
		}
	}
	if p.TurnInPlace {
		out = append(out,
			Primitive{SteerAngle: SteerAngleRotate, TurnAngle: p.TurnInPlaceStep},
			Primitive{SteerAngle: SteerAngleRotate, TurnAngle: -p.TurnInPlaceStep},
		)
	}
	return out, nil
}

// Apply 在给定位姿上执行一个基元，返回结束位姿（精确圆弧积分）。
func Apply(start geom.Pose, pr Primitive, p Params) geom.Pose {
	h0 := start.H
	if pr.SteerAngle == SteerAngleRotate {
		return geom.Pose{Pos: start.Pos, H: geom.NormAngle(h0 + pr.TurnAngle)}
	}
	if math.Abs(pr.TurnAngle) < 1e-12 {
		// 直线
		return geom.Pose{
			Pos: geom.Vec{X: start.Pos.X + pr.Length*math.Cos(h0), Y: start.Pos.Y + pr.Length*math.Sin(h0)},
			H:   geom.NormAngle(h0),
		}
	}
	// 圆弧：半径 R = L/|dtheta|，圆心在航向左侧（正向转弯）。
	r := pr.Length / pr.TurnAngle
	cx := start.Pos.X - r*math.Sin(h0)
	cy := start.Pos.Y + r*math.Cos(h0)
	h1 := geom.NormAngle(h0 + pr.TurnAngle)
	return geom.Pose{
		Pos: geom.Vec{X: cx + r*math.Sin(h1), Y: cy - r*math.Cos(h1)},
		H:   h1,
	}
}

// GoalReached 判断候选位姿是否在位置与航向容差内到达目标。
func GoalReached(cand, goal geom.Pose, p Params) bool {
	if cand.Pos.Dist(goal.Pos) > p.XYGoalTol {
		return false
	}
	return math.Abs(geom.AngleDiff(cand.H, goal.H)) <= p.YawTol
}
