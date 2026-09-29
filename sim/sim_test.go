package sim_test

import (
	"testing"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
	"github.com/h3yow/go-costmap-replanner-go/geom"
	simpkg "github.com/h3yow/go-costmap-replanner-go/sim"
)

func TestDeterministicOutput(t *testing.T) {
	sc := simpkg.Scenario{
		Sources: simpkg.DefaultSources(),
		Static:  simpkg.WallPoints(geom.Vec{X: 0, Y: -1}, geom.Vec{X: 0, Y: 1}, 0.25),
		Dynamic: func(td time.Duration) []geom.Vec {
			return []geom.Vec{{X: 1, Y: td.Seconds()}}
		},
		Seed: 7,
	}
	run := func() []costmap.Reading {
		s := simpkg.New(sc)
		var all []costmap.Reading
		for tk := 100 * time.Millisecond; tk <= time.Second; tk += 100 * time.Millisecond {
			all = append(all, s.Step(tk)...)
		}
		return all
	}
	a, b := run(), run()
	if len(a) != len(b) {
		t.Fatalf("两次运行读数数量不同: %d vs %d", len(a), len(b))
	}
	for i := range a {
		if a[i].Source != b[i].Source || a[i].Timestamp != b[i].Timestamp ||
			len(a[i].Points) != len(b[i].Points) {
			t.Fatalf("第 %d 条读数不一致", i)
		}
		for j := range a[i].Points {
			if a[i].Points[j] != b[i].Points[j] {
				t.Fatalf("读数 %d 点 %d 噪声不一致: %v vs %v", i, j, a[i].Points[j], b[i].Points[j])
			}
		}
	}
	t.Logf("相同 Seed 两次运行产出 %d 条读数，时间戳/噪声/点集完全一致", len(a))
}

func TestSensorPeriods(t *testing.T) {
	sc := simpkg.Scenario{Seed: 1} // 默认两路传感器
	s := simpkg.New(sc)
	counts := map[string]int{}
	for tk := 100 * time.Millisecond; tk <= time.Second; tk += 100 * time.Millisecond {
		for _, r := range s.Step(tk) {
			counts[r.Source]++
		}
	}
	t.Logf("1s 内读数次数: %v", counts)
	if counts["lidar"] != 10 {
		t.Fatalf("lidar 100ms 周期应产出 10 条, got %d", counts["lidar"])
	}
	if counts["sonar"] != 5 {
		t.Fatalf("sonar 200ms 周期应产出 5 条, got %d", counts["sonar"])
	}
}

func TestNoiseJitterBoundedAndAccepted(t *testing.T) {
	g, err := costmap.NewGrid(costmap.DefaultConfig())
	if err != nil {
		t.Fatal(err)
	}
	sc := simpkg.Scenario{
		Sources: []simpkg.SourceSpec{{Name: "lidar", Noise: 0.02, Period: 100 * time.Millisecond, MaxDist: 8}},
		Seed:    99,
		Dynamic: func(td time.Duration) []geom.Vec {
			// 障碍在 (1,0) 附近抖动（传感器噪声）
			return []geom.Vec{{X: 1, Y: 0}}
		},
	}
	for _, s := range sc.Sources {
		g.RegisterSource(s.Name)
	}
	sm := simpkg.New(sc)
	rejected := 0
	for tk := 100 * time.Millisecond; tk <= time.Second; tk += 100 * time.Millisecond {
		for _, r := range sm.Step(tk) {
			res := g.Ingest(r, r.Timestamp.Add(50*time.Millisecond))
			if !res.Accepted() {
				rejected++
			}
		}
	}
	if rejected != 0 {
		t.Fatalf("仅含轻微噪声的读数不应被拒绝, rejected=%d", rejected)
	}
	// 抖动的观测点应只影响 (1,0) 附近的少量格子
	n := 0
	for row := range g.Snapshot().Raw {
		for _, v := range g.Snapshot().Raw[row] {
			if v > 0 {
				n++
			}
		}
	}
	t.Logf("噪声读数全部接受，受影响格子数=%d（期望为少数邻近格）", n)
	if n > 6 {
		t.Fatalf("2cm 噪声不应扩散到过多格子, got %d", n)
	}
}
