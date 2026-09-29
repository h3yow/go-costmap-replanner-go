// sim 用本地模拟传感器数据跑通 地图更新 -> 规划 -> 重规划 全链路。
// 使用固定随机种子，输出确定可复现。
package main

import (
	"fmt"
	"math"
	"math/rand"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
	"github.com/h3yow/go-costmap-replanner-go/planner"
)

func main() {
	cfg := costmap.Config{
		Width: 100, Height: 100, Resolution: 0.1,
		OriginX: 0, OriginY: 0,
		MaxAge: 500 * time.Millisecond, ObstacleCost: 254, InflateRadius: 0.3,
	}
	cm := costmap.New(cfg)
	pl := planner.New(planner.Config{
		MinTurnRadius: 0.5, SwitchMargin: 0.1, ObstacleCost: 254,
	})

	base := time.Now()
	rng := rand.New(rand.NewSource(42))

	// 场景：一道静态墙（x=5.05, y∈[0,7)），留上方缺口。
	// 读数取栅格中心坐标，避免浮点边界量化在墙上打出洞。
	var staticBatch []costmap.Reading
	for gy := 0; gy < 70; gy++ {
		y := (float64(gy) + 0.5) * 0.1
		staticBatch = append(staticBatch, costmap.Reading{
			SensorID: "lidar", Stamp: base, X: 5.05, Y: y, Range: 3.0, Intensity: 1.0,
		})
	}
	// 混入异常读数：过期、乱序、越界、损坏、未来时间戳。
	bad := []costmap.Reading{
		{SensorID: "lidar", Stamp: base.Add(-time.Second), X: 1, Y: 1, Range: 1, Intensity: 1}, // 过期
		{SensorID: "lidar", Stamp: base.Add(-time.Millisecond), X: 2, Y: 2, Range: 1, Intensity: 1}, // 乱序
		{SensorID: "sonar", Stamp: base, X: 99, Y: 99, Range: 1, Intensity: 1},                      // 越界
		{SensorID: "sonar", Stamp: base, X: math.NaN(), Y: 1, Range: 1, Intensity: 1},               // 损坏
		{SensorID: "sonar", Stamp: base.Add(time.Hour), X: 1, Y: 1, Range: 1, Intensity: 1},         // 未来
	}
	now := base.Add(10 * time.Millisecond)
	for i, d := range cm.Ingest(now, append(staticBatch, bad...)) {
		if !d.Accepted {
			fmt.Printf("[ingest] 读数 #%d 被拒绝: %s (%s)\n", i, d.Reason, d.Detail)
		}
	}
	snap := cm.Snapshot()
	fmt.Printf("[map] 版本=%d\n", snap.Version)

	req := planner.PlanRequest{
		Start: planner.Point{X: 1.05, Y: 5.05}, Goal: planner.Point{X: 9.05, Y: 5.05}, Heading: 0,
	}
	res := pl.Plan(snap, cfg, req)
	fmt.Printf("[plan] ok=%v reason=%q cost=%.2f points=%d explored=%d | %s\n",
		res.OK, res.Reason, res.Cost, len(res.Path), res.Explored, res.Detail)

	// 动态障碍：t=100ms 时堵住上方缺口的一部分（y∈[7,8.5)），
	// 仍留有更上方的通道，触发重规划。
	now = base.Add(100 * time.Millisecond)
	var dyn []costmap.Reading
	for gy := 70; gy < 85; gy++ {
		y := (float64(gy) + 0.5) * 0.1
		dyn = append(dyn, costmap.Reading{
			SensorID: "lidar", Stamp: now, X: 5.05, Y: y, Range: 2.0, Intensity: 1.0,
		})
	}
	cm.Ingest(now, dyn)
	res = pl.Plan(cm.Snapshot(), cfg, req)
	fmt.Printf("[replan] ok=%v reason=%q cost=%.2f points=%d | %s\n",
		res.OK, res.Reason, res.Cost, len(res.Path), res.Detail)

	// 目标被完全包围：在目标栅格的 8 邻格上直接放置障碍，失败原因可区分。
	now = base.Add(200 * time.Millisecond)
	var wall []costmap.Reading
	for dx := -1; dx <= 1; dx++ {
		for dy := -1; dy <= 1; dy++ {
			if dx == 0 && dy == 0 {
				continue
			}
			wall = append(wall, costmap.Reading{
				SensorID: "lidar", Stamp: now,
				X: 9.05 + float64(dx)*0.1, Y: 5.05 + float64(dy)*0.1,
				Range: 1, Intensity: 1.0,
			})
		}
	}
	cm.Ingest(now, wall)
	res = pl.Plan(cm.Snapshot(), cfg, req)
	fmt.Printf("[surrounded] ok=%v reason=%q | %s\n", res.OK, res.Reason, res.Detail)

	// 噪声抖动：同一障碍附近随机扰动，验证路径不来回切换。
	_ = rng // 抖动场景在单元测试中覆盖，此处保持输出精简
	fmt.Println("[done] 全链路演示结束")
}
