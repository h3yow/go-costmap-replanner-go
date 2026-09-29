// Command demo 在本地用模拟传感器读数跑通
// 传感器并发接入 -> 代价地图融合 -> 规划 -> 动态重规划 的完整链路，
// 并显式注入乱序/过期/损坏/越界/未注册读数以展示可区分的拒绝原因。
//
// 并发模型：单个确定性生产者按场景时钟产出读数，分发给每路传感器
// 各自的消费 goroutine，由它们并发调用 costmap.IngestMany，
// 从而真实地考验地图内部的锁与原子批量更新。
package main

import (
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/h3yow/go-costmap-replanner-go/costmap"
	"github.com/h3yow/go-costmap-replanner-go/geom"
	"github.com/h3yow/go-costmap-replanner-go/kinematics"
	"github.com/h3yow/go-costmap-replanner-go/planner"
	"github.com/h3yow/go-costmap-replanner-go/replanner"
	"github.com/h3yow/go-costmap-replanner-go/sim"
)

const simDuration = 12 * time.Second

func main() {
	// —— 地图与底盘 ——
	grid, err := costmap.NewGrid(costmap.DefaultConfig())
	must(err)
	for _, src := range sim.DefaultSources() {
		grid.RegisterSource(src.Name)
	}
	// probe：专门用于注入异常读数的注册源；seq：用于演示乱序判定
	grid.RegisterSource("probe")
	grid.RegisterSource("seq")
	kp := kinematics.DefaultParams()
	kp.AllowReverse = false // 演示场景正向即可绕行，避免简化控制器倒车抖动
	pl, err := planner.New(kp, planner.DefaultOptions())
	must(err)
	rpCfg := replanner.DefaultConfig()
	rpCfg.BlockedConfirmations = 2
	rpCfg.ReplanLookahead = 1
	// 定期重规划默认关闭：演示中路径切换只由“确认阻塞/偏航”等硬条件触发，
	// 更能体现滞回稳定性；需要时可打开（如 500ms）。
	rpCfg.PeriodicReplan = 0
	rp := replanner.New(pl, rpCfg)

	start := geom.Pose{Pos: geom.Vec{X: -4, Y: 0}, H: 0}
	goal := geom.Pose{Pos: geom.Vec{X: 4, Y: 0}, H: 0}

	// —— 场景：x=0 处一堵带门洞的静态墙；一个动态障碍 2s~4s 横穿门洞 ——
	wallTop := sim.WallPoints(geom.Vec{X: 0, Y: 0.8}, geom.Vec{X: 0, Y: 3}, 0.12)
	wallBottom := sim.WallPoints(geom.Vec{X: 0, Y: -3}, geom.Vec{X: 0, Y: -0.8}, 0.12)
	sc := sim.Scenario{
		Sources: sim.DefaultSources(),
		Static:  append(wallTop, wallBottom...),
		Dynamic: func(t time.Duration) []geom.Vec {
			tv := t.Seconds()
			if tv < 2 || tv > 4.8 {
				return nil
			}
			y := -1.5 + 3.0*((tv-2)/2)
			return []geom.Vec{{X: 0, Y: y}, {X: 0.1, Y: y}, {X: -0.1, Y: y}}
		},
		Seed: 42,
	}
	simulator := sim.New(sc)

	// —— 每路传感器一个消费 worker，并发写入地图 ——
	feeds := map[string]chan costmap.Reading{}
	for _, spec := range sc.Sources {
		feeds[spec.Name] = make(chan costmap.Reading, 16)
	}
	var wg sync.WaitGroup         // 传感器消费 workers
	var producerWg sync.WaitGroup // 模拟器生产者 + 异常注入者
	var rejMu sync.Mutex
	done := make(chan struct{})
	rejectCount := map[costmap.RejectReason]int{}
	record := func(name string, results []costmap.IngestResult) {
		for _, res := range results {
			if !res.Accepted() || res.Degraded {
				rejMu.Lock()
				rejectCount[res.Reason]++
				rejMu.Unlock()
				fmt.Printf("  [传感器] %-5s 读数处置: %-24s %s\n", name, res.Reason, oneLine(res.Detail))
			}
		}
	}
	for _, spec := range sc.Sources {
		wg.Add(1)
		go func(name string, ch chan costmap.Reading) {
			defer wg.Done()
			for r := range ch {
				record(name, grid.IngestMany([]costmap.Reading{r}, r.Timestamp.Add(20*time.Millisecond)))
				time.Sleep(3 * time.Millisecond) // 与决策循环交错，制造真实竞争
			}
		}(spec.Name, feeds[spec.Name])
	}

	// —— 生产者：每 100ms 由确定性模拟器产出读数并分发 ——
	producerWg.Add(1)
	go func() {
		defer producerWg.Done()
		// 按墙钟节拍推进仿真：每 100ms 一拍，使读数时间戳与决策循环的
		// 衰减时钟保持同一时基（真实机器人传感器驱动也是这个节奏）。
		for t := 100 * time.Millisecond; t <= simDuration; t += 100 * time.Millisecond {
			select {
			case <-done:
				return
			default:
			}
			for _, r := range simulator.Step(t) {
				select {
				case feeds[r.Source] <- r:
				case <-done:
					return
				}
			}
			time.Sleep(100 * time.Millisecond)
		}
	}()

	// —— 显式异常注入（独立 goroutine，与正常读数并发竞争）——
	producerWg.Add(1)
	go func() {
		defer producerWg.Done()
		sleep := func(d time.Duration) { time.Sleep(d) }
		sleep(1500 * time.Millisecond)
		injectAt := func(r costmap.Reading, label string, now time.Time) {
			res := grid.Ingest(r, now)
			rejMu.Lock()
			rejectCount[res.Reason]++
			rejMu.Unlock()
			fmt.Printf("  [注入] %-12s %s — %s\n", label, res.Reason, oneLine(res.Detail))
		}
		inject := func(r costmap.Reading, label string) {
			injectAt(r, label, r.Timestamp.Add(50*time.Millisecond))
		}
		// 使用 probe 专用源与场景时基，避免与正常传感器流的时间戳竞争，
		// 从而让每一种异常原因都由对应的校验稳定命中。
		// 第一条先投过期读数（该源尚无更新时间戳，避免被乱序规则抢先命中）：
		// 时间戳前置 1s，以场景 1.5s 判定时年龄 2.5s > MaxAge 2s。
		injectAt(sim.StaleReading("probe", simulator.Now(0).Add(-1*time.Second),
			[]geom.Vec{{X: 1, Y: 1}}), "过期读数", simulator.Now(1500*time.Millisecond))
		sleep(100 * time.Millisecond)
		inject(sim.CorruptReading("probe", simulator.Now(1600*time.Millisecond)), "损坏读数")
		sleep(100 * time.Millisecond)
		inject(sim.OutOfBoundsReading("probe", simulator.Now(1700*time.Millisecond), geom.Vec{X: 99, Y: 99}), "全部越界")
		sleep(100 * time.Millisecond)
		// 部分越界：一个点正常、一个点出界，应被裁剪后“降级接受”
		inject(costmap.Reading{Source: "probe", Timestamp: simulator.Now(1800 * time.Millisecond),
			Points: []geom.Vec{{X: 1, Y: 1}, {X: 99, Y: 99}}, Confidence: 1}, "部分越界降级")
		sleep(100 * time.Millisecond)
		// 低置信度读数应降级接受
		inject(costmap.Reading{Source: "probe", Timestamp: simulator.Now(1900 * time.Millisecond),
			Points: []geom.Vec{{X: -2, Y: 2}}, Confidence: 0.2}, "低置信度降级")
		sleep(100 * time.Millisecond)
		inject(costmap.Reading{
			Source: "ghost", Timestamp: simulator.Now(2000 * time.Millisecond),
			Points: []geom.Vec{{X: 1, Y: 1}}, Confidence: 1,
		}, "未注册源")
		sleep(200 * time.Millisecond)
		// 乱序：先在 seq 源接受较新读数，再投递较旧读数，后者必须被拒绝
		inject(costmap.Reading{Source: "seq", Timestamp: simulator.Now(2300 * time.Millisecond),
			Points: []geom.Vec{{X: 2, Y: 2}}, Confidence: 1}, "时序较新读数")
		inject(costmap.Reading{Source: "seq", Timestamp: simulator.Now(2100 * time.Millisecond),
			Points: []geom.Vec{{X: 2, Y: 2}}, Confidence: 1}, "乱序读数")
	}()

	// —— 决策循环 ——
	fmt.Println("=== 局部代价地图 + 动态重规划 端到端演示 ===")
	fmt.Printf("起 (%.1f,%.1f) -> 终 (%.1f,%.1f)；动态障碍 2s~4s 穿过门洞\n\n",
		start.Pos.X, start.Pos.Y, goal.Pos.X, goal.Pos.Y)

	pose := start
	progress := 0 // 已提交路径上的进度下标，只增不减
	ticker := time.NewTicker(200 * time.Millisecond)
	defer ticker.Stop()
	reached := false
	for decisionT := time.Duration(0); decisionT <= simDuration; decisionT += 200 * time.Millisecond {
		select {
		case <-ticker.C:
		}
		now := simulator.Now(decisionT)
		snap := grid.Decay(now)
		st := rp.Update(snap, pose, goal, now)
		fmt.Printf("[t=%.1fs] map=v%-4d trigger=%-18s replan=%-5v ok=%-5v cost=%6.2f %s\n",
			decisionT.Seconds(), st.MapVersion, st.Trigger, st.Replanned, st.Success,
			st.CommittedCost, oneLine(st.Detail))
		if !st.Success && st.Reason != "" {
			fmt.Printf("            失败原因=%s\n", st.Reason)
		}
		if path := rp.Committed(); len(path) > 1 && st.Success {
			if st.Replanned {
				progress = nearestFreeAhead(pose, path, snap)
			}
			pose, progress = advanceAlong(pose, path, 0.18, progress, snap)
			if pose.Pos.Dist(goal.Pos) < 0.25 {
				fmt.Printf("\n到达目标，结束。最终地图版本 v%d\n", grid.Version())
				reached = true
				break
			}
		}
	}
	if !reached {
		fmt.Printf("\n仿真结束仍未到达目标（位姿 %.2f,%.2f），最后状态见上。\n", pose.Pos.X, pose.Pos.Y)
	}
	close(done)       // 通知生产者/注入者停止发送
	producerWg.Wait() // 等它们全部退出后才允许关闭 feed
	for _, ch := range feeds {
		close(ch)
	}
	wg.Wait() // 等待传感器 workers 排空已入队读数

	fmt.Println("\n=== 读数处置统计（异常均被显式拒绝/降级，无静默丢弃）===")
	for _, r := range []costmap.RejectReason{
		costmap.RejectCorrupt, costmap.RejectOutOfBounds, costmap.RejectStale,
		costmap.RejectFuture, costmap.RejectOutOfOrder, costmap.RejectEmpty,
		costmap.RejectUnknownSensor, costmap.RejectLowConfidence,
	} {
		if n := rejectCount[r]; n > 0 {
			fmt.Printf("  %-24s %d 次\n", r, n)
		}
	}
}

// nearestFreeAhead 在新提交路径上找到机器人前方最近的自由点下标。
func nearestFreeAhead(pose geom.Pose, path []planner.Waypoint, snap costmap.Snapshot) int {
	idx, best := 0, math.Inf(1)
	for i, wp := range path {
		if snap.BlockedAt(wp.Pose.Pos.X, wp.Pose.Pos.Y) {
			continue
		}
		if d := wp.Pose.Pos.Dist(pose.Pos); d < best {
			best, idx = d, i
		}
	}
	return idx
}

// advanceAlong 从 progress 下标起沿路径前进 step 米，返回新位姿与新进度。
func advanceAlong(
	pose geom.Pose, path []planner.Waypoint, step float64, progress int, snap costmap.Snapshot,
) (geom.Pose, int) {
	idx := progress
	remain := step
	for idx+1 < len(path) {
		next := path[idx+1].Pose
		d := pose.Pos.Dist(next.Pos)
		if d >= remain {
			frac := remain / d
			return geom.Pose{
				Pos: geom.Vec{
					X: pose.Pos.X + (next.Pos.X-pose.Pos.X)*frac,
					Y: pose.Pos.Y + (next.Pos.Y-pose.Pos.Y)*frac,
				},
				H: next.H,
			}, idx
		}
		remain -= d
		pose.Pos = next.Pos
		idx++
	}
	return path[len(path)-1].Pose, len(path) - 1
}

func oneLine(s string) string {
	out := make([]rune, 0, 60)
	for i, r := range s {
		if i >= 60 {
			out = append(out, '…')
			break
		}
		if r == '\n' {
			r = ' '
		}
		out = append(out, r)
	}
	return string(out)
}

func must(err error) {
	if err != nil {
		panic(err)
	}
}
