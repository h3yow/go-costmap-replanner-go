// Package costmap 实现并发安全的二维局部代价地图。
//
// 坐标与时间约定见 doc.go。
package costmap

import (
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/geom"
)

// RejectReason 描述一条传感器读数被拒绝或降级的原因。
type RejectReason int

const (
	RejectNone          RejectReason = iota // 读数正常接受
	RejectCorrupt                           // 数据损坏（NaN/Inf、时间戳零值、置信度非法等）
	RejectOutOfBounds                       // 观测点（全部）落在地图范围外
	RejectStale                             // 读数时间戳过期
	RejectFuture                            // 时间戳超出允许的未来漂移
	RejectOutOfOrder                        // 时间戳早于该传感器已接受的最新读数（乱序）
	RejectEmpty                             // 读数不含任何点
	RejectUnknownSensor                     // 未注册的传感器源
	RejectLowConfidence                     // 置信度过低：降级接受，命中权重衰减
)

func (r RejectReason) String() string {
	switch r {
	case RejectNone:
		return "accepted"
	case RejectCorrupt:
		return "corrupt"
	case RejectOutOfBounds:
		return "out_of_bounds"
	case RejectStale:
		return "stale"
	case RejectFuture:
		return "future_timestamp"
	case RejectOutOfOrder:
		return "out_of_order"
	case RejectEmpty:
		return "empty"
	case RejectUnknownSensor:
		return "unknown_sensor"
	case RejectLowConfidence:
		return "low_confidence_degraded"
	default:
		return "unknown_reason"
	}
}

// Reading 是一路传感器在某一时刻对若干障碍点的观测。
// 所有点必须在同一坐标系（局部地图坐标系）下表达。
type Reading struct {
	Source    string
	Timestamp time.Time
	Points    []geom.Vec
	// Confidence 取值 (0,1]；< LowConfidenceThreshold 时读数降级接受。
	Confidence float64
}

// IngestResult 记录一条读数的处置结果，拒绝与降级均显式上报，绝不静默丢弃。
type IngestResult struct {
	Reading Reading
	Reason  RejectReason
	// Degraded 为 true 表示读数未被丢弃，而是以衰减/裁剪方式降级融合。
	Degraded bool
	Detail   string
}

// Accepted 报告读数是否对地图产生了（可能是降级的）有效更新。
func (r IngestResult) Accepted() bool {
	return r.Reason == RejectNone || r.Degraded
}

// Config 描述代价地图几何与时效参数。
type Config struct {
	Width, Height float64 // 地图尺寸（米），以原点为中心
	Resolution    float64 // 单元格边长（米）
	// MaxAge：早于 now-MaxAge 的读数视为过期。
	MaxAge time.Duration
	// MaxFutureSkew：允许的时钟未来漂移。
	MaxFutureSkew time.Duration
	// HitIncrement：一次满置信命中对单元格占用代价的增量。
	HitIncrement float64
	// DecayPerSecond：无新命中时，单元格占用代价每秒衰减量。
	DecayPerSecond float64
	// InflationRadius：障碍膨胀半径（米）。
	InflationRadius float64
	// LethalCost：代价 >= 该值视为致命障碍。
	LethalCost float64
	// LowConfidenceThreshold：Confidence 低于该值的读数降级接受。
	LowConfidenceThreshold float64
}

// DefaultConfig 返回一组可用的默认参数。
func DefaultConfig() Config {
	return Config{
		Width:                  20,
		Height:                 20,
		Resolution:             0.1,
		MaxAge:                 2 * time.Second,
		MaxFutureSkew:          100 * time.Millisecond,
		HitIncrement:           55,
		DecayPerSecond:         20,
		InflationRadius:        0.3,
		LethalCost:             100,
		LowConfidenceThreshold: 0.5,
	}
}

// Snapshot 是某一时刻代价地图（含膨胀层）的不可变快照。
type Snapshot struct {
	Cells   [][]float64 // [row][col] 行优先，row 对应 y 由小到大，含膨胀
	Raw     [][]float64 // 未膨胀的原始占用代价
	Config  Config
	TakenAt time.Time
	OriginX float64
	OriginY float64
	Cols    int
	Rows    int
	Version uint64
}

// Cell 是单个网格坐标。
type Cell struct{ Col, Row int }

// CellCost 返回单元格代价；越界网格第二个返回值为 false。
func (s Snapshot) CellCost(col, row int) (float64, bool) {
	if col < 0 || row < 0 || col >= s.Cols || row >= s.Rows {
		return 0, false
	}
	return s.Cells[row][col], true
}

// Contains 判断世界坐标是否在地图范围内。
func (s Snapshot) Contains(x, y float64) bool {
	dx := x - s.OriginX
	dy := y - s.OriginY
	return dx >= -s.Config.Width/2 && dx < s.Config.Width/2 &&
		dy >= -s.Config.Height/2 && dy < s.Config.Height/2
}

// BlockedAt 判断世界坐标是否被阻塞：地图边界外按致命处理（保守），
// 边界内代价 >= LethalCost 视为阻塞。
func (s Snapshot) BlockedAt(x, y float64) bool {
	if !s.Contains(x, y) {
		return true
	}
	col := int((x - s.OriginX + s.Config.Width/2) / s.Config.Resolution)
	row := int((y - s.OriginY + s.Config.Height/2) / s.Config.Resolution)
	c, ok := s.CellCost(col, row)
	return ok && c >= s.Config.LethalCost
}

// Grid 是并发安全的局部代价地图。
// 内部维护原始占用层 raw 与膨胀层 infl；两者始终在同一把锁内一起更新。
type Grid struct {
	mu sync.RWMutex

	cfg        Config
	raw, infl  [][]float64
	cols, rows int
	originX    float64
	originY    float64

	registered   map[string]bool
	lastAccepted map[string]time.Time
	lastUpdate   time.Time
	version      uint64
}

// NewGrid 按配置创建地图，参数非法时返回错误。
func NewGrid(cfg Config) (*Grid, error) {
	if cfg.Width <= 0 || cfg.Height <= 0 || cfg.Resolution <= 0 {
		return nil, fmt.Errorf("costmap: 地图尺寸与分辨率必须为正，得到 %vx%v @%v", cfg.Width, cfg.Height, cfg.Resolution)
	}
	if cfg.Resolution > cfg.Width || cfg.Resolution > cfg.Height {
		return nil, fmt.Errorf("costmap: 分辨率 %v 大于地图尺寸", cfg.Resolution)
	}
	if cfg.MaxAge <= 0 || cfg.HitIncrement <= 0 || cfg.DecayPerSecond < 0 || cfg.LethalCost <= 0 {
		return nil, fmt.Errorf("costmap: 时效/代价参数非法: %+v", cfg)
	}
	cols := int(math.Round(cfg.Width / cfg.Resolution))
	rows := int(math.Round(cfg.Height / cfg.Resolution))
	raw := make([][]float64, rows)
	infl := make([][]float64, rows)
	for r := range raw {
		raw[r] = make([]float64, cols)
		infl[r] = make([]float64, cols)
	}
	return &Grid{
		cfg:          cfg,
		raw:          raw,
		infl:         infl,
		cols:         cols,
		rows:         rows,
		registered:   map[string]bool{},
		lastAccepted: map[string]time.Time{},
	}, nil
}

// RegisterSource 注册一路允许接入的传感器。
func (g *Grid) RegisterSource(name string) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.registered[name] = true
}

// Ingest 接收一路传感器读数。乱序/过期等读数返回可区分的拒绝原因，绝不静默丢弃。
func (g *Grid) Ingest(r Reading, now time.Time) IngestResult {
	results := g.IngestMany([]Reading{r}, now)
	return results[0]
}

// IngestMany 原子地融合一批读数：所有判定与写入在同一把写锁内顺序完成，
// 外部观测者要么看到本批次全部生效，要么完全看不到。每个输入都有对应结果。
func (g *Grid) IngestMany(rs []Reading, now time.Time) []IngestResult {
	g.mu.Lock()
	defer g.mu.Unlock()

	results := make([]IngestResult, len(rs))
	appliedAny := false
	for i, r := range rs {
		res := g.validateLocked(r, now)
		results[i] = res
		if !res.Accepted() {
			continue
		}
		weight := 1.0
		if res.Reason == RejectLowConfidence {
			weight = res.Reading.Confidence
		}
		// res.Reading.Points 可能已经过越界裁剪，必须用结果中的读数。
		g.applyPointsLocked(res.Reading.Points, weight)
		// 乱序判定基于“已接受”时间戳；降级读数同样推进该源时间戳。
		g.lastAccepted[res.Reading.Source] = res.Reading.Timestamp
		appliedAny = true
	}
	if appliedAny {
		g.recomputeInflationLocked()
		g.bumpVersionLocked(now)
	}
	return results
}

// validateLocked 必须在持有 g.mu 时调用。
func (g *Grid) validateLocked(r Reading, now time.Time) IngestResult {
	if !g.registered[r.Source] {
		return IngestResult{Reading: r, Reason: RejectUnknownSensor,
			Detail: fmt.Sprintf("传感器源 %q 未注册", r.Source)}
	}
	if len(r.Points) == 0 {
		return IngestResult{Reading: r, Reason: RejectEmpty, Detail: "读数不含任何观测点"}
	}
	// 数据完整性：时间戳、置信度、每个点的坐标都必须是有限数。
	if r.Timestamp.IsZero() {
		return IngestResult{Reading: r, Reason: RejectCorrupt, Detail: "时间戳为零值"}
	}
	if math.IsNaN(r.Confidence) || math.IsInf(r.Confidence, 0) ||
		r.Confidence <= 0 || r.Confidence > 1 {
		return IngestResult{Reading: r, Reason: RejectCorrupt,
			Detail: fmt.Sprintf("置信度非法: %v", r.Confidence)}
	}
	for i, p := range r.Points {
		if math.IsNaN(p.X) || math.IsInf(p.X, 0) || math.IsNaN(p.Y) || math.IsInf(p.Y, 0) {
			return IngestResult{Reading: r, Reason: RejectCorrupt,
				Detail: fmt.Sprintf("第 %d 个点坐标非有限数: %+v", i, p)}
		}
	}
	// 时效判定先于乱序判定：严重过期直接以 stale 拒绝。
	if age := now.Sub(r.Timestamp); age > g.cfg.MaxAge {
		return IngestResult{Reading: r, Reason: RejectStale,
			Detail: fmt.Sprintf("读数年龄 %v 超过 MaxAge %v", age, g.cfg.MaxAge)}
	}
	if drift := r.Timestamp.Sub(now); drift > g.cfg.MaxFutureSkew {
		return IngestResult{Reading: r, Reason: RejectFuture,
			Detail: fmt.Sprintf("时间戳超前 %v 超过允许漂移 %v", drift, g.cfg.MaxFutureSkew)}
	}
	if last, ok := g.lastAccepted[r.Source]; ok && r.Timestamp.Before(last) {
		return IngestResult{Reading: r, Reason: RejectOutOfOrder,
			Detail: fmt.Sprintf("时间戳 %v 早于该源最新已接受时间 %v", r.Timestamp, last)}
	}
	// 边界处理：全部越界则拒绝；部分越界则裁剪后降级接受。
	inBounds := r.Points[:0:0]
	for _, p := range r.Points {
		if g.containsLocked(p.X, p.Y) {
			inBounds = append(inBounds, p)
		}
	}
	if len(inBounds) == 0 {
		return IngestResult{Reading: r, Reason: RejectOutOfBounds,
			Detail: fmt.Sprintf("%d 个点全部落在地图范围外", len(r.Points))}
	}
	if len(inBounds) < len(r.Points) {
		total, kept := len(r.Points), len(inBounds)
		r.Points = inBounds
		return IngestResult{Reading: r, Reason: RejectOutOfBounds, Degraded: true,
			Detail: fmt.Sprintf("裁剪 %d/%d 个越界点后降级融合", total-kept, total)}
	}
	if r.Confidence < g.cfg.LowConfidenceThreshold {
		return IngestResult{Reading: r, Reason: RejectLowConfidence, Degraded: true,
			Detail: fmt.Sprintf("置信度 %.2f 低于阈值 %.2f，命中权重衰减",
				r.Confidence, g.cfg.LowConfidenceThreshold)}
	}
	return IngestResult{Reading: r, Reason: RejectNone}
}

// applyPointsLocked 把命中累加到原始占用层。
func (g *Grid) applyPointsLocked(points []geom.Vec, weight float64) {
	for _, p := range points {
		c, ok := g.worldToCellLocked(p.X, p.Y)
		if !ok {
			continue // 越界点已在校验阶段被裁剪
		}
		v := g.raw[c.Row][c.Col] + g.cfg.HitIncrement*weight
		if v > g.cfg.LethalCost {
			v = g.cfg.LethalCost
		}
		g.raw[c.Row][c.Col] = v
	}
}

// recomputeInflationLocked 依据原始层重算膨胀层，与原始层在同一临界区更新。
// 致命障碍膨胀半径内一律致命；非致命占用按线性衰减扩散。
func (g *Grid) recomputeInflationLocked() {
	radius := g.cfg.InflationRadius
	rCells := int(math.Ceil(radius / g.cfg.Resolution))
	for row := 0; row < g.rows; row++ {
		for col := 0; col < g.cols; col++ {
			best := g.raw[row][col]
			if rCells > 0 {
				for dr := -rCells; dr <= rCells; dr++ {
					for dc := -rCells; dc <= rCells; dc++ {
						if dr == 0 && dc == 0 {
							continue
						}
						rr, cc := row+dr, col+dc
						if rr < 0 || cc < 0 || rr >= g.rows || cc >= g.cols {
							continue
						}
						rv := g.raw[rr][cc]
						if rv <= 0 {
							continue
						}
						dist := math.Hypot(float64(dr), float64(dc)) * g.cfg.Resolution
						if dist > radius {
							continue
						}
						v := rv * (1 - dist/(radius+g.cfg.Resolution))
						if rv >= g.cfg.LethalCost && dist <= radius {
							v = g.cfg.LethalCost
						}
						if v > best {
							best = v
						}
					}
				}
			}
			g.infl[row][col] = best
		}
	}
}

// Decay 应用时间衰减：raw 每格减去 DecayPerSecond*dt，然后重建膨胀层。
func (g *Grid) Decay(now time.Time) Snapshot {
	g.mu.Lock()
	defer g.mu.Unlock()
	if !g.lastUpdate.IsZero() && now.After(g.lastUpdate) {
		dt := now.Sub(g.lastUpdate).Seconds()
		drop := g.cfg.DecayPerSecond * dt
		changed := false
		for row := range g.raw {
			for col := range g.raw[row] {
				v := g.raw[row][col]
				if v <= 0 {
					continue
				}
				changed = true
				v -= drop
				if v < 0 {
					v = 0
				}
				g.raw[row][col] = v
			}
		}
		if changed {
			g.recomputeInflationLocked()
		}
	}
	g.bumpVersionLocked(now)
	return g.snapshotLocked(now)
}

func (g *Grid) bumpVersionLocked(now time.Time) {
	g.version++
	g.lastUpdate = now
}

// Snapshot 返回当前代价地图（膨胀层+原始层）的不可变拷贝。
func (g *Grid) Snapshot() Snapshot {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.snapshotLocked(g.lastUpdate)
}

func (g *Grid) snapshotLocked(at time.Time) Snapshot {
	cp := Snapshot{
		Cells:   make([][]float64, g.rows),
		Raw:     make([][]float64, g.rows),
		Config:  g.cfg,
		TakenAt: at,
		OriginX: g.originX,
		OriginY: g.originY,
		Cols:    g.cols,
		Rows:    g.rows,
		Version: g.version,
	}
	for r := range g.infl {
		cp.Cells[r] = append([]float64(nil), g.infl[r]...)
		cp.Raw[r] = append([]float64(nil), g.raw[r]...)
	}
	return cp
}

// WorldToCell / CellToWorld 进行坐标与网格换算。
func (g *Grid) WorldToCell(x, y float64) (Cell, bool) {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.worldToCellLocked(x, y)
}

func (g *Grid) worldToCellLocked(x, y float64) (Cell, bool) {
	if !g.containsLocked(x, y) {
		return Cell{}, false
	}
	col := int((x - g.originX + g.cfg.Width/2) / g.cfg.Resolution)
	row := int((y - g.originY + g.cfg.Height/2) / g.cfg.Resolution)
	if col >= g.cols {
		col = g.cols - 1
	}
	if row >= g.rows {
		row = g.rows - 1
	}
	return Cell{Col: col, Row: row}, true
}

func (g *Grid) containsLocked(x, y float64) bool {
	dx := x - g.originX
	dy := y - g.originY
	return dx >= -g.cfg.Width/2 && dx < g.cfg.Width/2 &&
		dy >= -g.cfg.Height/2 && dy < g.cfg.Height/2
}

// CellToWorld 返回网格中心的世界坐标。
func (g *Grid) CellToWorld(c Cell) geom.Vec {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return geom.Vec{X: g.originX - g.cfg.Width/2 + (float64(c.Col)+0.5)*g.cfg.Resolution, Y: g.originY - g.cfg.Height/2 + (float64(c.Row)+0.5)*g.cfg.Resolution}
}

// CostAt 返回某世界坐标的当前（膨胀层）代价；地图外第二个返回值为 false。
func (g *Grid) CostAt(x, y float64) (float64, bool) {
	g.mu.RLock()
	defer g.mu.RUnlock()
	c, ok := g.worldToCellLocked(x, y)
	if !ok {
		return 0, false
	}
	return g.infl[c.Row][c.Col], true
}

// IsBlocked 判断世界坐标是否为致命障碍（含膨胀）；地图外视为阻塞。
func (g *Grid) IsBlocked(x, y float64) bool {
	g.mu.RLock()
	defer g.mu.RUnlock()
	c, ok := g.worldToCellLocked(x, y)
	if !ok {
		return true
	}
	return g.infl[c.Row][c.Col] >= g.cfg.LethalCost
}

// IsBlockedCell 判断网格是否为致命障碍（含膨胀）。
func (g *Grid) IsBlockedCell(col, row int) bool {
	g.mu.RLock()
	defer g.mu.RUnlock()
	if col < 0 || row < 0 || col >= g.cols || row >= g.rows {
		return true
	}
	return g.infl[row][col] >= g.cfg.LethalCost
}

// Version 返回地图单调递增的版本号。
func (g *Grid) Version() uint64 {
	g.mu.RLock()
	defer g.mu.RUnlock()
	return g.version
}

// LastAccepted 返回某路传感器最新被接受（含降级接受）读数的时间戳。
func (g *Grid) LastAccepted(source string) (time.Time, bool) {
	g.mu.RLock()
	defer g.mu.RUnlock()
	ts, ok := g.lastAccepted[source]
	return ts, ok
}
