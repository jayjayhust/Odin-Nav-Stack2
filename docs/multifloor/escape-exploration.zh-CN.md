# 卡死逃逸探索（Stuck-Escape Exploration）——两层结构，一份契约

本分支的目的，是让卡死逃逸中*路线探索*的那一半，达到*局部机动*那一半早已具备的
仿真↔真机一致性，并让逃逸具备**地形感知**能力：机器狗正在楼梯中段攀爬时，绝不能
被换掉路线。`NeuPAN/neupan/escape_policies/stuck_escape.py` 记录了这件事为什么重要：在它出现
之前，benchmark 里带着一处手写的简化，"所以关于逃逸调参的仿真结果根本无法迁移到
真机上"。本分支让*另一半*也遵守同样的规则。

逃逸被刻意拆成两层，各在一处实现：

| | 局部机动 | 路线探索 |
|---|---|---|
| 回答的问题 | "我现在卡住了——倒车并转向" | "这条走廊是死路——换一条绕行" |
| 真机 | `NeuPAN/neupan/escape_policies/stuck_escape.py`（`StuckEscapePolicy`） | `ros_ws/src/mission_manager` 逃逸回合 |
| 仿真 | tick 里的 `state.blocked` / `state.backOut` | `replanGlobal()` 多选最优 + 回合内轮换 |
| 决策库 | （在策略内部） | `mission_manager/include/mission_manager/escape_route_gate.h` |
| 触发 | 停留计数（`trigger_dwell_ms`） | `StuckEscapePolicy.request_replan` / `/neupan/replan` |

`escape_route_gate.h` 是纯头文件、**不依赖 ROS** 的库，被 `ros_ws` 与 `ros2_ws`
两棵树逐字节共享。真机上关于*选哪条路线*的每一个决策都是对它的一次调用，因此
接受、排序、轮换和地形判定的数学只有一份实现。

## 一致性是被证明的，而不是被断言的

一份表驱动的 fixture 同时驱动仿真探针和 C++ 测试：

- `docs/multifloor/sim/escape_contract.fixture.json`
- `docs/multifloor/sim/.probe/probe_escape_contract.js`（node，直接从
  `mesh_nav.template.html` 里切出 JS 决策函数）
- `ros_ws/src/mission_manager/test/test_escape_route_gate.cpp`（及其 `ros2_ws`
  镜像）把*同一个*文件喂给头文件跑一遍。

只要两个运行时在任一用例上不一致，两个探针里就会有一个失败。这就是一致性闸门。
改一条规则必须两边都改，然后两边都重跑。

地形行为由 `docs/multifloor/sim/.probe/probe_escape_terrain.py` 单独校验（4 个
用例：中段保持、距平台一级台阶时安装、撤退目标为平台、楼层切换停留哨兵）。

## 回合状态机（路线层）

路线层是"一次卡死过程"的记忆。真机上叫 `EscapeEpisode`，仿真里是
`state.escape {try, sigs, at, ep}`，形状一致：

```
        (新目标 / 到达 / 取消 / 复位)
                    │  resetEpisode()
                    ▼
  空闲 ──请求──► 评估 ──terrainGate()──► 保持 / 撤退   （中段：不换路线，
 (round 0)        │   ▲                     (不换路线)   保持原路线，
                  │   └── episodeMaybeReset ─► (已推进 >1.2 m 或         爬向
                  ▼                              陈旧 >20 s)             平台）
              挑选 / 接受 ──► 经普通重规划闸门安装，noteEpisodeInstall
                  │
                  ▼
              tried_sigs 增长；round 前进（饱和于 max_rounds）
```

- **Round 0** 安装打分最高（argmax）的路线，与本分支之前完全一致。
- **Round ≥ 1** 刻意把这一轮花在打分排第二、第三……且签名未被试过的路线上——
  排序只是代理指标，反复安装那条刚刚又堵住的"最优"路线正是本层要打破的死循环。
- 一旦机器狗真的重新前进，或这一过程变陈旧，回合会复位（回到全新 argmax），
  于是稍后一次无关的卡死不会被旧记忆拖累。
- **触发基于“实际推进”而非“阻塞标志跳变”。** 停计时只在机器狗真正越过它开始
  被堵的位置（仿真里为再多走 `0.5 m` 路线，一个 tick 内常量）时才复位，而不是在
  blocked 判定碰巧消失的第一帧就清零。贴着障碍蹭动的狗每隔约两秒就会把
  `state.blocked` 翻一下却几乎不动；若每次翻动都清空计时，一次“真的卡住但在抖”
  的过程就永远触发不了逃逸。

## 四条地形规则

以 `stair_commit_enable` 为开关；关闭时逃逸是地形盲的（分支前的行为）。
`terrainGate()` 返回 `allow | hold | retreat_to_landing`。

1. **中段：不换路线。** 当机器狗处于某段楼梯的两个平台之间、且不在任一平台的
   `landing_snap_radius_m` 范围内时，逃逸不生成、不安装任何路线——狗先爬到最近的
   平台。真机上 `runEscapeEpisode()` 会*消费*掉这次重规划请求但不改变路径。
2. **撤退以平台为目标。** 当确实需要在中段反向时，目标是最近的 portal 单元，而非
   "沿原路后退"（一段未被识别的台阶边缘不是可站立的表面）。
3. **重回已承诺楼梯的候选被直接拒绝，而非降权。** 一条重新进入本回合已承诺楼梯的
   候选会被置 `terrain_penalty > 0`，在接受打分*之前*就被过滤掉。
4. **楼层交接不是卡死。** 更长的 `transition_dwell_ms`（普通 `trigger_dwell_ms`
   的 2 倍）以实时“在楼梯上”信号（`state.onStairNow`）为准——即机器狗处于某段
   楼梯的两平台之间或正在步态交接——使缓慢攀爬不被误判为卡住；开阔平地上的堵塞
   仍用普通 `trigger_dwell_ms`。轮换永不丢弃 stairDecouple 生成的楼梯中心线腿。
   （它故意*不*以“平地缓慢通行证窗口” `floorPass` 为准：后者在平地上为真，会
   错误地拉长平地上的普通堵塞阈值。）

真机侧局部策略有一个对应的、可选开启的闸门（`stuck_escape.py`：`note_terrain()` +
`stair_suppress_enable`，默认 **false**，由 `/stair_gait/status` 喂入），使倒车加
转向不在楼梯中段启动。

## 绕行是临时的，规划不是

逃逸安装的路线是一次**战术级、单趟**机动，而不是对战略任务的改写。逼停机器狗的
障碍往往*是临时的*——路过的人、被挪开的椅子、一时关上的门——所以把这条绕远的路
固化进全局规划，会让机器狗在隐患早已消失之后还在绕路。

仿真里用两条折线来体现这一点：

- `state.path`——机器狗当前实际骑行的路线。逃逸只把绕行段装到这里，且仅此处
  （`installPath(best.pts, /*ephemeral*/ true)`）。
- `state.basePath`——**已提交的任务线**，在用户点「规划」（或带 hash 启动自动规划
  demo）时写入，`replanGlobal` 永不触碰。

当循环演示的一趟走到终点后，tick 会把 `state.basePath` 恢复为活动路线并清掉回合
记忆，于是下一趟重新从**真正的起点**跑原始任务：障碍还在就再次脱困，障碍已清就
直接走近路。这与真机一致——真机不循环，但 `mission_manager` 会不断从静态先验重新
规划，战术绕行同样不会被固化进战略路线。

## 候选池——候选是怎么生成的

`replanGlobal()` 不会只向规划器要一条备选路线，而是构造一个真正互不相同的
**best-of-N 候选池**，用 `routeSafety`（物理 OCC 净空与动态障碍净空取更紧者，
按局部层的同一口径读取）逐条打分，再安装其中能过接受闸门的最安全一条。候选来源，
按开销从低到高：

| 类别（`cls`） | 生成方式 | 规划图 → 打分图 | 来源 |
|---|---|---|---|
| `绕开立柱` | 把动态立柱合进搜索（`globalMergePosts`）后重规划同一串站点 | 实时 → 实时 | 核心 |
| `贴墙绕行` | 在膨胀带兄弟图 `state._bandMap` 上重规划，把贴墙路线拉离墙面 | 宽带 → 宽带 | 核心 |
| `全局×N` | **膨胀扇出** —— 按 `candidate_inflation_scales` 中大于 1.0 的每项各规划一条（`ensureInflMaps()` 懒建） | 宽带 → **实时** | 仅仿真 |
| `绕行试探` | `detourSeeds()` 的绕行途经点，在膨胀带上规划 | 宽带 → 宽带 | 核心 |
| `绕行·现场` | 同一批途经点在**实时图**上再规划一遍（round ≥ 1）——两条真正全新的路线 | 实时 → 实时 | 仅仿真 |
| `绕行·随机` | **随机途经点**散布在整段前瞻窗口，每个爆发重摇一次 | 实时 → 实时 | 仅仿真 |

### `detourSeeds()`——途经点生成器

它先在路线前方 `detour_lookahead` 米内找到最窄点 `c`；若 `escDetourWanted(worst,
round)` 判定走廊确实很窄，就围绕它发出可通行格，并按轮次逐步扩大野心：

1. **垂线泊位**（round 0+）：沿局部切线两侧 ±`detour_berths_m`（后几轮放宽到
   `detour_berths_widened_m`）的可通行格。
2. **斜向摆幅**（round 2+）：把切线旋转 ±`detour_swing_rad`、在 `detour_swing_range_m`
   处取可通行格。从第三次尝试起，这些摆幅会**预留** `detour_max_seeds` 中的
   `2 × len(detour_swing_rad)` 个名额，以免垂线泊位占满全部名额把摆幅饿死。这一
   预留逐字节镜像进了 C++ 的 `detourSeedPoints()`。
3. **随机尾巴**（round 1+，仅仿真）：若干途经点，在整个窗口内随机取一个弧长位置，
   按随机一侧、随机 1.2–2.4 m 半径偏移，每次调用都用 `Math.random` 重摇。这就是
   打破局部最优的一环——上面的网格是确定性的且只聚集在唯一那个隘口 `c`，没有它
   一次卡死的每个节流爆发都会重规划*同一*候选池。`detourSeeds()` 返回 `{seeds, nGrid}`，
   于是 `replanGlobal()` 对确定性的头部在宽带+实时图上都规划、对随机尾巴只在实时图
   上规划（绕大圈在更宽的膨胀带上净空反而更差，且每条规划两遍会让单次 A\* 开销翻倍）。
   C++ 侧保持确定性：真机的一次逃逸必须可复现，便于现场排障。

> 随机化和扇出只是扩大了*尝试集*；它们造不出地图里本来就没有的净空。详见
> [仍然只在仿真侧](#仍然只在仿真侧诚实清单) 下的注意事项。

## 参数

`robots/<id>/neupan/config.yaml` 里的 `stuck_escape.route_rotation`（仿真通过
`build.py --profile` 读同一段；`mission_manager` 从自身的 `route_rotation/*` 参数
读取，通过 `escape_route_enable` 可选开启）。

| 键 | 默认 | 含义 |
|---|---|---|
| `enable` | true | 卡死期间探索备选路线 |
| `candidate_inflation_scales` | `[1.0, 1.5, 2.0]` | 每个膨胀倍率一次全局规划；仿真从第二次尝试起会在更宽膨胀带的兄弟图上重规划同一串站点（仅仿真扇出，见下） |
| `detour_berths_m` | `[0.5, 0.95]` | 横向绕行偏移，首次尝试 |
| `detour_berths_widened_m` | `[0.5, 0.95, 1.45]` | ……第二次尝试起 |
| `detour_lookahead_m` / `_widened_m` | 3.0 / 6.0 | 向前搜索最窄点的距离 |
| `detour_max_seeds` | 6 | 每轮绕行途经点上限；从第三次尝试起，斜向摆幅预留 `2 × len(detour_swing_rad)` 个名额，防止被垂线泊位占满而饿死 |
| `detour_swing_rad` | `[-0.6, 0.6]` | 相对垂线的斜向摆幅，第三次尝试起（每个角度 × 每侧各占一个预留途经点名额） |
| `detour_swing_range_m` | 1.3 | ……该摆幅的作用距离 |
| `install_gain_m` | 0.05 | 候选替换现路线所需的清除度增益（也是排序分档宽度） |
| `rescue_clearance_m` | 0.225 | 本体半宽 + 5cm；低于此值路线会卡住机器人 |
| `min_candidate_clearance_m` | 0.175 | 本体半宽；探索绝不安装低于此值的路线 |
| `max_rounds` | 3 | 尝试索引在此饱和 |
| `route_sig_grid_m` | 0.5 | "已试过"路线签名的量化栅格 |
| `max_tried_sigs` | 8 | 每回合记住的路线签名数 |
| `episode_reset_progress_m` | 1.2 | 又推进这么多即结束本次卡死 |
| `episode_reset_timeout_sec` | 20 | ……或距上次安装这么久 |
| `stair_commit_enable` | true | 地形规则开（false = 地形盲逃逸） |
| `escape_only_at_landing` | true | 中段不换路线，只在平台上换 |
| `landing_snap_radius_m` | 0.45 | 距 portal 多近算"在平台上" |
| `trigger_dwell_ms` | 3000 | 停住多久后逃逸触发 |
| `transition_dwell_ms` | 6000 | ……跨楼层切换时的阈值 |
| `replan_throttle_ms` | 1200 | 两次逃逸重规划之间的最小间隔 |
| `trace_enable` | true | 每个逃逸决策输出一行紧凑 JSON |

这些默认值就是仿真在参数化之前使用的常量，所以重新构建出的页面在配置改动之前
行为完全一致。

## 仿真 → 真机 符号对照表

| 仿真（`mesh_nav.template.html`） | 真机 |
|---|---|
| `replanGlobal()`（逃逸块） | `MissionManager::runEscapeEpisode()` |
| `routeSafety()` / `staticClearanceAhead()` | `ValidatePath` 式清除度；扫描走廊经 `escapeCorridorClearance()`（`corridorBlocked`） |
| `sigOf()` | `routeSignature()` |
| `cands.filter(...)` 接受判定 | `acceptCandidate()` |
| 排序 + 轮换遍历 | `pickCandidate()` |
| `detourSeeds()` | `detourSeedPoints()` |
| `state._bandMap`（禁行带兄弟图） | 膨胀倍率候选（见"仅仿真"） |
| tick 楼梯规则 / `state.onStairNow` | `terrainGate()`，由 `/stair_gait/status` 喂入 |
| `escDwellMs()` | `escapeDwellMs()` |
| `state.basePath` + `installPath(pts, ephemeral)` | `mission_manager` 反复重规划所依据的静态先验全局路线；逃逸安装不被提交进去 |
| `state.escape {try, sigs, at}` | `EscapeEpisode` |
| `window.__ESCAPE_TRACE`（数组） | `/neupan/mission_escape_trace`（JSON 行，`formatEscapeTrace`） |

两条遥测路径输出同一种记录形状——
`{stamp, episode_id, round, kind, clearance, current, sig, action, on_flight}`——
这正是让仿真调参结论可迁移到真机的关键。

## 仍然只在仿真侧（诚实清单）

- **多候选膨胀扇出（现已在仿真侧实现）。** 从第二次尝试起，`replanGlobal()` 会按
  `candidate_inflation_scales` 中大于 1.0 的每项各重规划一条（经 `ensureInflMaps()`），
  每条在膨胀带更宽的兄弟图上规划，但仍以*实时图*打分，所以清除度数值保持真实。在狭
  窄房间里，这些宽带的路线往往比标称规划得分*更低*（更大的禁行带会把 A\* 挤进更硬的
  物理 OCC 隘口），所以扇出只是扩大了尝试集、并不改变胜出候选——这是一次覆盖面改进，
  不是清除度提升。真机侧无法镜像：`ros_ws`/`ros2_ws` 的 `nav_interfaces/PlanPath.srv`
  只带 `{goal, shadow_only}`——**没有 `inflation_radius` 字段**——且本方案禁止新增服务
  字段，所以真机逃逸只对目标重规划那一条标称路线。接受、轮换、地形闸门、安装闸门
  和遥测全都照常运行；不同的只是候选集的*宽度*。绕行途经点理论上可作为临时单航点
  目标下发，但那段两阶段机动未经硬件验证，保持关闭。
- **随机化绕行种子——打破局部最优（仅仿真侧）。** `detourSeeds()` 里的垂线泊位/斜向
  摆幅网格是确定性的，且只聚集在前方可通行点最窄的那一处，所以一次卡死的每个节流爆发
  都会重规划*同一*候选池，容易陷入**候选空间**的局部最优：一条真正把隘口绕到一侧、
  净空更高的路线，因为没有任何网格点采样到能抵达它的走廊而始终被埋没。从第二次尝试起，
  `detourSeeds()` 还会额外返回一小撮随机途经点（`绕行·随机`）：在整个前瞻窗口内随机取
  一个弧长位置，按随机的一侧、随机 1.2–2.4 m 的半径偏移，每次调用都用 `Math.random`
  重摇，于是卡死期间的约 10 个爆发会扫过网格漏掉的那片可通行开敞空间。它们**只在实时
  图上规划**（宽带兄弟图会把绕大圈的净空吃掉，且每条规划两遍会让单次 A\* 开销翻倍却
  无益于接受）。`freeCellNear` 会丢弃落进墙里的、`planStops` 会丢弃不可达的，接受闸门
  仍要过机身半宽预算，所以这一步只会*增加*尝试——绝不删掉好候选或降低门槛。C++ 侧的
  `detourSeedPoints()` 保持确定性：真机的一次逃逸必须可复现，便于现场排障。
  **注意——随机性造不出净空。** 对上报卡死场景（`示例 单层阶梯` 起点→第一个途经点这段
  走廊）做过途经点全空间扫描：没有任何一条路线超过 0.175 m 的机身预算，全局最大只有
  ~0.15 m，被下游一处硬瓶颈卡死——而 `routeSafety` 取的是整条剩余链路的最小净空，于是
  所有候选打成平手。此时逃逸正确地拒绝安装（等价或更差的切换不可接受），扇出和随机种子
  都改变不了这一点；这种路段的真正杠杆在接受参数/打分视野，而不在更多探索。
- **`ValidatePath` 清除度排序。** ROS1 没有 `ValidatePath` 服务，ROS2 的返回也
  不含清除度指标，所以真机在有新鲜 360 扫描时按扫描走廊清除度排序，否则按折线长度
  排序。仿真的 `routeSafety` 代理是几何的、信息更丰富。
- **真机的地形抑制仅限 ROS1。** `/stair_gait/status` 是 ROS1 话题；`ros2_ws` 逃逸
  以地形盲方式运行（本分支的多楼层/楼梯范围是 ROS1/Go2）。
- **局部机动并未被完整仿真**（走廊 / 极限环 / 旋转闸门动力学）；仿真的只有逃逸的
  停留与地形判定。
- **临时绕行 / 已提交规划的分离靠循环体现。** 仿真在一趟结束时恢复
  `state.basePath`——这是它判断“隐患可能已消失”的唯一信号；真机经由
  `mission_manager` 基于静态先验的周期性重规划到达同样效果，而不是靠循环边界。

fixture 与记录形状都是这样设计的：将来每一条都可以在不重命名任何东西的前提下
搬到真机上。

## 在真机上启用

`escape_route_enable`（mission_manager 参数）默认 **false**，因此在启用之前现有
部署逐字节不变。启用方法：设 `escape_route_enable:=true`，并把 `route_rotation/*`
各键（来自机器人的 `neupan/config.yaml`）映射进 mission_manager 的参数命名空间。
建议与 `install_gate_enable`/`install_gate_shadow` 一同开启，这样逃逸经由同一个
安全闸门安装，且走廊清除度排序有数据可用。
