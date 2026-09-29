// Package testutil 为代价地图/规划相关测试与演示提供确定性的地图构造工具。
package testutil

import (
	"math"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
)

// EmptySnapshot 生成一张全自由代价地图快照。
func EmptySnapshot() costmap.Snapshot {
	cfg := costmap.DefaultConfig()
	cols := int(math.Round(cfg.Width / cfg.Resolution))
	rows := int(math.Round(cfg.Height / cfg.Resolution))
	cells := make([][]float64, rows)
	raw := make([][]float64, rows)
	for r := range cells {
		cells[r] = make([]float64, cols)
		raw[r] = make([]float64, cols)
	}
	return costmap.Snapshot{
		Cells: cells, Raw: raw, Config: cfg,
		TakenAt: time.Unix(0, 0), Cols: cols, Rows: rows, Version: 1,
	}
}

// CloneSnapshot 深拷贝快照，便于在其基础上叠加动态障碍。
func CloneSnapshot(s costmap.Snapshot) costmap.Snapshot {
	cp := s
	cp.Cells = make([][]float64, s.Rows)
	cp.Raw = make([][]float64, s.Rows)
	for r := range s.Cells {
		cp.Cells[r] = append([]float64(nil), s.Cells[r]...)
		cp.Raw[r] = append([]float64(nil), s.Raw[r]...)
	}
	cp.Version++
	return cp
}

// FillBlocked 在快照上把轴对齐矩形区域（世界坐标，米）标记为致命。
func FillBlocked(s *costmap.Snapshot, x0, y0, x1, y1 float64) {
	for row := 0; row < s.Rows; row++ {
		for col := 0; col < s.Cols; col++ {
			x := s.OriginX - s.Config.Width/2 + (float64(col)+0.5)*s.Config.Resolution
			y := s.OriginY - s.Config.Height/2 + (float64(row)+0.5)*s.Config.Resolution
			if x >= x0 && x <= x1 && y >= y0 && y <= y1 {
				s.Cells[row][col] = s.Config.LethalCost
				s.Raw[row][col] = s.Config.LethalCost
			}
		}
	}
	s.Version++
}

// CountBlocked 统计致命格数量，供一致性断言。
func CountBlocked(s costmap.Snapshot) int {
	n := 0
	for row := range s.Cells {
		for _, v := range s.Cells[row] {
			if v >= s.Config.LethalCost {
				n++
			}
		}
	}
	return n
}
