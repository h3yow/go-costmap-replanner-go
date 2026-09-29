package kinematics

import (
	"math"
	"testing"

	"github.com/h3yow/go-costmap-replanner-go/geom"
)

func TestPrimitivesRespectMinRadius(t *testing.T) {
	p := DefaultParams()
	prims, err := Primitives(p)
	if err != nil {
		t.Fatalf("Primitives: %v", err)
	}
	for _, pr := range prims {
		if pr.SteerAngle == SteerAngleRotate {
			continue
		}
		if math.Abs(pr.SteerAngle) > 1e-9 {
			r := math.Abs(p.WheelBase / math.Tan(pr.SteerAngle))
			if r < p.MinTurnRadius*(1-1e-9) {
				t.Fatalf("基元转弯半径 %.3f 小于最小值 %.3f", r, p.MinTurnRadius)
			}
		}
	}
	t.Logf("生成 %d 个基元，全部满足最小转弯半径 %.2fm", len(prims), p.MinTurnRadius)

	bad := p
	bad.SteerAngles = []float64{1.4} // tan(1.4)*R 几乎为零
	if _, err := Primitives(bad); err == nil {
		t.Fatal("违反最小转弯半径的转向角应报错")
	}
}

func TestApplyGeometry(t *testing.T) {
	p := DefaultParams()
	prims, _ := Primitives(p)

	// 直线前进 0.5m
	straight := Primitive{SteerAngle: 0, Length: 0.5}
	got := Apply(geom.Pose{Pos: geom.Vec{X: 0, Y: 0}, H: 0}, straight, p)
	if math.Abs(got.Pos.X-0.5) > 1e-9 || math.Abs(got.Pos.Y) > 1e-9 {
		t.Fatalf("直线端点 = %+v, want (0.5,0)", got.Pos)
	}

	// 整圆：转弯角 2π 的基元应回到起点附近
	arc := prims[2] // 左转弯前进
	r := arc.Length / arc.TurnAngle
	full := Primitive{SteerAngle: arc.SteerAngle, Length: r * 2 * math.Pi, TurnAngle: 2 * math.Pi}
	end := Apply(geom.Pose{Pos: geom.Vec{X: 1, Y: 2}, H: 0.3}, full, p)
	if end.Pos.Dist(geom.Vec{X: 1, Y: 2}) > 1e-6 {
		t.Fatalf("整圆未回到起点: %+v", end.Pos)
	}

	// 原地旋转只改航向
	rot := Primitive{SteerAngle: SteerAngleRotate, TurnAngle: math.Pi / 2}
	end = Apply(geom.Pose{Pos: geom.Vec{X: 3, Y: 4}, H: 0}, rot, p)
	if end.Pos.X != 3 || end.Pos.Y != 4 || math.Abs(geom.AngleDiff(end.H, math.Pi/2)) > 1e-9 {
		t.Fatalf("原地旋转结果错误: %+v", end)
	}
}

func TestGoalReached(t *testing.T) {
	p := DefaultParams()
	if !GoalReached(
		geom.Pose{Pos: geom.Vec{X: 0.1, Y: 0}, H: 0.1},
		geom.Pose{Pos: geom.Vec{X: 0, Y: 0}, H: 0}, p) {
		t.Fatal("容差内应判定到达")
	}
	if GoalReached(
		geom.Pose{Pos: geom.Vec{X: 5, Y: 0}, H: 0},
		geom.Pose{Pos: geom.Vec{X: 0, Y: 0}, H: 0}, p) {
		t.Fatal("距离远超容差不应判定到达")
	}
}
