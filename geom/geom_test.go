package geom

import (
	"math"
	"testing"
)

func TestVecAndAngle(t *testing.T) {
	if got := (Vec{3, 4}).Len(); math.Abs(got-5) > epsilon {
		t.Fatalf("Len = %v, want 5", got)
	}
	if got := (Vec{0, 0}).Dist(Vec{3, 4}); math.Abs(got-5) > epsilon {
		t.Fatalf("Dist = %v, want 5", got)
	}
	cases := []struct{ in, want float64 }{
		{3 * math.Pi, -math.Pi},
		{-3 * math.Pi / 2, math.Pi / 2},
		{0, 0},
		{2 * math.Pi, 0},
	}
	for _, c := range cases {
		if got := NormAngle(c.in); math.Abs(got-c.want) > epsilon {
			t.Errorf("NormAngle(%v) = %v, want %v", c.in, got, c.want)
		}
	}
	if got := AngleDiff(0.1, -0.1); math.Abs(got-0.2) > epsilon {
		t.Errorf("AngleDiff = %v, want 0.2", got)
	}
}

func TestTransform(t *testing.T) {
	origin := Pose{Pos: Vec{1, 0}, H: math.Pi / 2}
	// 全局点 (1,1) 在原点局部系（x 朝全局 +y）应为 (1,0)
	got := Transform(Pose{Pos: Vec{1, 1}}, origin)
	if math.Abs(got.X-1) > epsilon || math.Abs(got.Y) > epsilon {
		t.Fatalf("Transform = %+v, want {1 0}", got)
	}
}
