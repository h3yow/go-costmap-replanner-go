package costmap

import (
	"fmt"
	"math"
	"sync"
	"time"
)

// futureTolerance 容忍时钟微小超前（如不同传感器时钟偏差）。
const futureTolerance = 200 * time.Millisecond

// CostMap 是并发安全的局部代价地图。
// 更新以批次为单位原子提交：一批读数在同一个写锁内完成校验与融合，
// 读者只能看到批次提交前或提交后的状态，不存在"更新了一半"的中间状态。
type CostMap struct {
	mu      sync.RWMutex
	cfg     Config
	cells   []uint8
	version uint64
	// lastStamp 记录每个传感器已应用读数的最新时间戳，用于乱序检测。
	lastStamp map[string]time.Time
}

// New 创建代价地图。配置非法时返回 nil。
func New(cfg Config) *CostMap {
	if cfg.Width <= 0 || cfg.Height <= 0 || cfg.Resolution <= 0 {
		return nil
	}
	if cfg.MaxAge <= 0 {
		cfg.MaxAge = 500 * time.Millisecond
	}
	if cfg.ObstacleCost == 0 {
		cfg.ObstacleCost = 254
	}
	return &CostMap{
		cfg:       cfg,
		cells:     make([]uint8, cfg.Width*cfg.Height),
		lastStamp: make(map[string]time.Time),
	}
}

// Config 返回地图配置（值拷贝）。
func (m *CostMap) Config() Config { return m.cfg }

// validate 校验单条读数，返回拒绝原因（RejectNone 表示通过）。
// 检查顺序固定，保证同样的输入得到同样的结论。
func (m *CostMap) validate(now time.Time, r Reading) (RejectReason, string) {
	// 1. 损坏：NaN/Inf、负距离、置信度越界。
	if math.IsNaN(r.X) || math.IsNaN(r.Y) || math.IsInf(r.X, 0) || math.IsInf(r.Y, 0) ||
		math.IsNaN(r.Range) || math.IsInf(r.Range, 0) || r.Range < 0 ||
		math.IsNaN(r.Intensity) || r.Intensity < 0 || r.Intensity > 1 {
		return RejectCorrupt, "坐标/距离/置信度含 NaN、Inf 或非法值"
	}
	// 2. 时间戳来自未来（超出容忍）。
	if r.Stamp.After(now.Add(futureTolerance)) {
		return RejectFutureStamped, fmt.Sprintf("时间戳超前当前时间 %v", r.Stamp.Sub(now))
	}
	// 3. 过期。
	if age := now.Sub(r.Stamp); age > m.cfg.MaxAge {
		return RejectStale, fmt.Sprintf("读数年龄 %v 超过上限 %v", age, m.cfg.MaxAge)
	}
	// 4. 乱序：比同传感器已应用的最新读数更旧。
	if last, ok := m.lastStamp[r.SensorID]; ok && r.Stamp.Before(last) {
		return RejectOutOfOrder, fmt.Sprintf("时间戳早于传感器 %q 已应用的 %v", r.SensorID, last)
	}
	// 5. 越界。
	if _, _, ok := m.WorldToGrid(r.X, r.Y); !ok {
		return RejectOutOfBounds, fmt.Sprintf("命中点 (%.2f, %.2f) 在地图外", r.X, r.Y)
	}
	return RejectNone, ""
}

// Ingest 校验并融合一批读数，返回每条读数的处理结论（与输入一一对应）。
// 整个批次在一次写锁内原子提交；即使部分读数被拒绝，
// 被接受的读数也作为一个整体生效，版本号只前进一次。
func (m *CostMap) Ingest(now time.Time, batch []Reading) []Decision {
	decisions := make([]Decision, len(batch))
	m.mu.Lock()
	defer m.mu.Unlock()

	accepted := false
	for i, r := range batch {
		reason, detail := m.validate(now, r)
		if reason != RejectNone {
			decisions[i] = Decision{Accepted: false, Reason: reason, Detail: detail}
			continue
		}
		m.apply(r)
		m.lastStamp[r.SensorID] = r.Stamp
		decisions[i] = Decision{Accepted: true}
		accepted = true
	}
	if accepted {
		m.version++
	}
	return decisions
}

// apply 将一条已校验读数融合进地图：命中栅格置为障碍代价（按置信度缩放），
// 并在膨胀半径内按距离衰减叠加代价。融合采用 max 语义，重复观测不会无限累积。
// 调用方必须持有写锁。
func (m *CostMap) apply(r Reading) {
	gx, gy, _ := m.WorldToGrid(r.X, r.Y)
	base := float64(m.cfg.ObstacleCost) * r.Intensity
	radCells := int(math.Ceil(m.cfg.InflateRadius / m.cfg.Resolution))
	for dy := -radCells; dy <= radCells; dy++ {
		for dx := -radCells; dx <= radCells; dx++ {
			nx, ny := gx+dx, gy+dy
			if nx < 0 || ny < 0 || nx >= m.cfg.Width || ny >= m.cfg.Height {
				continue
			}
			dist := math.Hypot(float64(dx), float64(dy)) * m.cfg.Resolution
			if dist > m.cfg.InflateRadius {
				continue
			}
			cost := base * (1 - dist/m.cfg.InflateRadius)
			if cost < 1 {
				cost = 1
			}
			idx := ny*m.cfg.Width + nx
			if c := uint8(math.Min(cost, 255)); c > m.cells[idx] {
				m.cells[idx] = c
			}
		}
	}
}

// Snapshot 返回当前地图的一致性快照（在读锁内整体拷贝）。
func (m *CostMap) Snapshot() Snapshot {
	m.mu.RLock()
	defer m.mu.RUnlock()
	cells := make([]uint8, len(m.cells))
	copy(cells, m.cells)
	return Snapshot{
		Cells:   cells,
		Width:   m.cfg.Width,
		Height:  m.cfg.Height,
		Version: m.version,
	}
}

// WorldToGrid 将地图坐标转换为栅格坐标，越界时 ok=false。
func (m *CostMap) WorldToGrid(x, y float64) (gx, gy int, ok bool) {
	gx = int(math.Floor((x - m.cfg.OriginX) / m.cfg.Resolution))
	gy = int(math.Floor((y - m.cfg.OriginY) / m.cfg.Resolution))
	ok = gx >= 0 && gy >= 0 && gx < m.cfg.Width && gy < m.cfg.Height
	return gx, gy, ok
}
