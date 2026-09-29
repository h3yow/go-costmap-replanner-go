// Package sim 用本地确定性数据源模拟多路传感器读数，
// 供端到端演示与测试使用：静态墙、动态障碍、噪声抖动。
//
// 模拟器内部为每一路传感器维护独立的、带固定种子的 PRNG 与单调步进时钟，
// 因此相同 Seed + 相同 Step 序列产生完全相同的读数序列，可复现。
// 异常读数（乱序/过期/损坏/越界）不由正常 Step 产生，而由显式构造器生成，
// 以便测试精确断言每一种拒绝/降级原因。
package sim

import (
	"math"
	"math/rand"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
	"github.com/h3yow/go-costmap-replanner-go/geom"
)

// SourceSpec 描述一路模拟传感器。
type SourceSpec struct {
	Name    string
	Noise   float64       // 观测点高斯噪声标准差（米）
	Period  time.Duration // 采样周期
	Offset  time.Duration // 相对场景时钟的固定相位偏移
	MaxDist float64       // 最大有效量程（米），超出的点不输出
}

// DefaultSources 返回两路默认传感器（激光、超声）。
func DefaultSources() []SourceSpec {
	return []SourceSpec{
		{Name: "lidar", Noise: 0.02, Period: 100 * time.Millisecond, MaxDist: 8},
		{Name: "sonar", Noise: 0.08, Period: 200 * time.Millisecond, MaxDist: 4},
	}
}

// Scenario 描述一个可复现的模拟场景。
type Scenario struct {
	Sources []SourceSpec
	// Static 是静态障碍点集（例如墙体采样点）。
	Static []geom.Vec
	// Dynamic 给定时刻 t（相对场景起点）返回当前动态障碍点集。
	Dynamic func(t time.Duration) []geom.Vec
	// Confidence 给定时刻与传感器名返回置信度；nil 表示恒为 1。
	Confidence func(t time.Duration, source string) float64
	Seed       int64
}

// Simulator 按时间步进产生多路读数。
type Simulator struct {
	sc      Scenario
	rngs    map[string]*rand.Rand
	stepFor map[string]int64
	start   time.Time
}

// New 创建模拟器。
func New(sc Scenario) *Simulator {
	if sc.Sources == nil {
		sc.Sources = DefaultSources()
	}
	s := &Simulator{
		sc:      sc,
		rngs:    map[string]*rand.Rand{},
		stepFor: map[string]int64{},
		start:   time.Unix(1000, 0),
	}
	for i, spec := range sc.Sources {
		// 每路传感器独立且确定的种子。
		s.rngs[spec.Name] = rand.New(rand.NewSource(sc.Seed + int64(i)*1000003))
	}
	return s
}

// Step 推进到相对时刻 t，返回当前时刻各路传感器“恰好到期”的读数。
// 返回切片顺序与 Sources 定义顺序一致，保证确定性。
func (s *Simulator) Step(t time.Duration) []costmap.Reading {
	var out []costmap.Reading
	for _, spec := range s.sc.Sources {
		n := int64(t / spec.Period)
		prev := s.stepFor[spec.Name]
		s.stepFor[spec.Name] = n
		if n <= prev {
			continue // 该传感器此周期尚未到期
		}
		ts := s.start.Add(time.Duration(n)*spec.Period + spec.Offset)

		var pts []geom.Vec
		emit := func(p geom.Vec) {
			if p.Len() > spec.MaxDist {
				return
			}
			if spec.Noise > 0 {
				p.X += s.rngs[spec.Name].NormFloat64() * spec.Noise
				p.Y += s.rngs[spec.Name].NormFloat64() * spec.Noise
			}
			pts = append(pts, p)
		}
		for _, p := range s.sc.Static {
			emit(p)
		}
		if s.sc.Dynamic != nil {
			for _, p := range s.sc.Dynamic(t) {
				emit(p)
			}
		}
		conf := 1.0
		if s.sc.Confidence != nil {
			conf = s.sc.Confidence(t, spec.Name)
		}
		out = append(out, costmap.Reading{
			Source: spec.Name, Timestamp: ts, Points: pts, Confidence: conf,
		})
	}
	return out
}

// Now 返回相对时刻 t 对应的墙钟（与读数时间戳同一时基）。
func (s *Simulator) Now(t time.Duration) time.Time {
	return s.start.Add(t)
}

// —— 异常读数显式构造器，供测试精确制造各类异常 ——

// CorruptReading 构造一条含 NaN 坐标的损坏读数。
func CorruptReading(src string, ts time.Time) costmap.Reading {
	return costmap.Reading{Source: src, Timestamp: ts,
		Points: []geom.Vec{{X: math.NaN(), Y: 1}}, Confidence: 1}
}

// OutOfBoundsReading 构造一条观测点落在 far 的越界读数。
func OutOfBoundsReading(src string, ts time.Time, far geom.Vec) costmap.Reading {
	return costmap.Reading{Source: src, Timestamp: ts,
		Points: []geom.Vec{far}, Confidence: 1}
}

// StaleReading 构造一条时间戳陈旧的读数。
func StaleReading(src string, ts time.Time, pts []geom.Vec) costmap.Reading {
	return costmap.Reading{Source: src, Timestamp: ts, Points: pts, Confidence: 1}
}

// WallPoints 在两点之间以 spacing 间隔采样墙体障碍点。
func WallPoints(a, b geom.Vec, spacing float64) []geom.Vec {
	d := b.Sub(a)
	n := int(math.Ceil(d.Len() / spacing))
	var out []geom.Vec
	for i := 0; i <= n; i++ {
		t := float64(i) / float64(n)
		out = append(out, a.Add(d.Scale(t)))
	}
	return out
}
