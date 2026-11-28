# ONS2 路线图 —— 可插拔地形识别与脱困策略槽位

[English](ROADMAP.md) | **中文**

本页跟踪在 [ONS2 架构 §2](architecture/ONS2_ARCHITECTURE.md) 的四个可插拔槽位之上新增的两个
扩展点：**地形识别槽位**（`terrain_recognizer.backend`）与**脱困策略槽位**
（`local_planner.escape_policy.backend`）。

两处槽位都是**按需开启**：`robot.yaml` 中不提及它们时，脱困策略仍是 `stuck_escape`，也不会
启动任何地形节点。§1–§2 描述当前代码树里已有的内容，§3 按范围递增列出后续里程碑，§4 固定
不得回退的兼容性规则。

---

## 1. 状态总览

下文术语：

- **已在代码树** —— 工作树中已有代码；按需开启，默认行为不变。
- **仅仿真** —— 只在 mesh 仿真里实现，真机上没有。
- **桩实现** —— 接口与参数已固定，不输出结论。
- **仅设计** —— 只在 §3 描述，尚无代码。

| 范围 | 状态 | 验证门槛 / 依据 | 下一步 |
|---|---|---|---|
| 地形槽位（ROS 1） | **已在代码树** —— `terrain_recognizer` 节点与后端工厂、`stair_gait` 与 `ramp_slope` 后端、多实例 `TerrainScene`、`/stair_gait/status` 旧格式重发布 | `multifloor/sim/test_recognizers.js`；Jetson 上的首次 `catkin_make` 仍待完成 | 消费方接入（§3.1.2）、逐实例参数（§3.1.3） |
| 地形槽位（ROS 2） | **仅设计** | — | §3.1.1 |
| `dino_v3` 后端 | **桩实现** —— 加载参数、不发布任何结论；没有随包 profile 选择它 | 以 `valid=false` 弃权 | §3.1.4 |
| 脱困槽位 | **已在代码树** —— `NeuPAN/neupan/escape_policies/` 注册表、`stuck_escape`、`null`、兼容 shim；来自 `/stair_gait/status` 的地形抑制仅 ROS 1 | `test_escape_policy_slot.py`；fixture 驱动的 `test_escape_route_gate.cpp`（首次构建待完成） | 轨迹字段对齐（§3.2.3） |
| SimReady 导入、3DGS 先验、世界模型、LingBot-Navi 适配器 | **仅设计** | 新槽位未配置时，仿真探针的判定必须完全不变 | §3.3–§3.6 |

**验证状态。** 脱困一致性闸门的设计目标是：两边一旦分歧就必须失败——同一份 fixture
`multifloor/sim/escape_contract.fixture.json` 被切进 JS 探针，也被编译进 C++ 测试。两个 ROS
工作区都尚未在开发机上构建，本仓库也没有负责构建或运行它们的 CI；目标 Jetson 上的首次
`catkin_make` 是尚未通过的构建门槛。硬件行为（切层、楼梯测试、步态交接）由多层文档跟踪，
步态驱动默认保持关闭。

---

## 2. 当前代码树已包含的内容

| 槽位 | 选择键 | 参考实现 | 运行位置 |
|---|---|---|---|
| 地形识别器 | `terrain_recognizer.backend` | `stair_gait`（几何高度剖面）、`ramp_slope`（几何平面拟合） | 仅 ROS 1 —— `terrain_recognizer` 节点，由 `go2.launch` 引入 |
| 地形识别器，多地形 | `terrain_recognizer.instances` | 同一批后端命名的 `(backend, region)` 实例 | 仅 ROS 1 —— 单个节点，`/terrain_recognizer/<name>/status` + `/terrain_recognizer/scene` |
| 地形识别器（学习型） | 同上 | `dino_v3` —— 只加载参数的桩，不发布任何结论 | 仅 ROS 1，没有任何随包 profile 选择它 |
| 脱困策略 | `local_planner.escape_policy.backend` | `stuck_escape`（默认）、`null` | ROS 1 与 ROS 2 —— 位于 NeuPAN 节点内 |

### 地形识别：包拆分

- `ros_ws/src/stair_gait` 用**不依赖** `unitree_sdk2` 的几何代码构建 `libstair_detector.so`；
  仅当找到 SDK 时，再额外构建旧版 `stair_gait_node`（驱动）。SDK 以 `QUIET` 方式查找，
  因此没装 SDK 的机器仍能构建该库。
- `ros_ws/src/terrain_recognizer` 拥有该槽位：后端工厂、`TerrainObservation` / `TerrainScene`
  消息、逐实例的 `/terrain_recognizer/<name>/status` 话题、`/terrain_recognizer/scene` 汇总，
  以及在 `/stair_gait/status` 上的旧版 JSON 重发布。重发布只汇总覆盖前方走廊的实例，因为
  NeuPAN 的 `stair_suppress_enable` 门控读的就是这个话题。

### 脱困策略：模块迁移

- `NeuPAN/neupan/escape_policies/` 存放契约（`EscapePolicy`、`EscapeDecision`、
  `as_escape_policy`）、注册表（`register_escape_policy` / `create_escape_policy`）、
  参考实现 `stuck_escape` 与 `null`。
- `NeuPAN/neupan/util/stuck_escape.py` 是弃用 shim，因此仿真 benchmark 与既有单元测试的
  import 不用改。

---

## 3. 后续里程碑

### 3.1 地形识别器

1. **ROS 2 对齐。** 节点今天是 ROS 1，因为参考后端直接消费 `/odin1/cloud_slam`。ROS 2 移植
   需要同样的工厂接口（除 `configure()` 里的 `ros::NodeHandle` 外，头文件与 ROS 无关），并在
   `bringup.launch.py` 里加一个 `terrain_recognizer` 条目。
2. **消费方接入。** `mission_manager` 与 `floor_manager` 尚未订阅。第一个消费方应做**软
   门控**——只标注当前路径段，不阻断——这样一次误判无法让机器人困住。`TerrainScene` 正是为此
   准备的消息：它已带上同一时刻的每种地形及其区域，恰好满足"走这段梯、避开那条坡"的需要。
3. **逐实例参数。** 后端参数今天是节点级的（`min_steps`、ROI 阈值），多实例站点只能按区域
   区分。下一步是逐实例参数命名空间（`~instances/<name>/params`），让两个楼梯实例可以使用
   不同的 `min_steps`。
4. **DINOv3 原型。** 补全 `dino_v3_backend.cpp`（教程：[04 · 添加自定义地形识别后端](usage/04_custom_terrain_backend.zh-CN.md)）：订阅 `image_topic`，按 `input_size` 裁剪，
   归入固定的 `type` 词汇，并把 logits 缩放为 `confidence`。验收门槛：模型不可用时必须发布
   `valid=false` 而不是猜测，与今天的桩完全一致。多区域场景要跑**一次**前向、按 ROI 输出多个
   裁剪结果，而不是每实例一次前向——实例数会成倍放大 CPU，平台 SPEC 里的算力板预算扛不住
   N 次推理。
5. **学习型后端护栏。** 学习型后端必须沿用同一套 `type` 词汇，始终发布 `geometry`（它所判定
   的区域），并且当 `confidence` 低于配置阈值时绝不发布 `stair_up` / `stair_down`——脱困槽位
   会在这些结论上自我抑制，一次误报就会把机器人困在楼梯脚下。

### 3.2 脱困策略

1. **`dino_escape` / 学习型策略。** 注册表已接受带 `create_policy(cfg)` 的点分模块，因此策略
   可以放在本仓库之外。此类策略必须通过与参考实现相同的验收套件
   （`NeuPAN/example/sw01_bench/test_escape_policy_slot.py`），包括 `neupan_ros.py` 直接读取的
   节点属性面。
2. **逐机器人 `config` 文件。** 该槽位把 `config` 叠加在旧版 `stuck_escape:` 块之上，站点
   文件只需覆盖自己改动的项。等 profile 用上之后，对应键应从
   `robots/<robot>/neupan/config.yaml` 迁出。
3. **轨迹字段对齐。** 任何替代实现都必须继续在 `/neupan/control_trace` 与 `/neupan/replan`
   上发布操作台读取的字段：`escape_active`、`escape_count`、`escape_direction`。

### 3.3 仿真资产 —— 导入 NVIDIA SimReady Central

1. **离线转换，而不是运行时加载器。** [sim/](multifloor/sim/README.md) 通过文件选择器接受
   `.obj`/`.stl`（单位与 up 轴已可由用户选择），`build.py` 把演示网格内联，保证构建出的页面
   自包含。SimReady Central 提供的是 USD/USDZ——PBR 材质、碰撞体与刚体物理，文件选择器和
   three.js 都读不了。因此转换器离线运行，输出 glTF/OBJ 加每资产 sidecar（单位、up 轴、
   碰撞盒、材质分类）。页面加载时不得抓取任何资源，所以大资产以场景 bundle 的形式放在
   `mesh_scene_data.json` 旁边，而不是内联。
2. **渲染世界 ≠ 导航世界。** 规划器消费的是 `buildMap` 的 walkable/surfaceCell 谓词、障碍
   圆柱以及楼梯踏面/平台标记——不是美术几何。每份导入资产都要映射到这套词汇（货架是立在
   可行走地面上的障碍体积；闸门是受限区域边界；一段楼梯需要踏面标记与平台连接）。没有映射
   的资产进入仅显示的 props 层。
3. **范围限制。** SimReady 强在工业道具、弱在导航关键地形，因此楼梯/坡道/地板的创作路径仍是
   `make_example_*.py`，SimReady 负责提供让脱困策略与仿真传感器更真实的杂物层。只有
   `multifloor/sim/test_recognizers.js` 在它上面得出与手工 fixture 相同的结论，该资产才算
   合格。
4. **验收门槛。** 导入 props 不得改变任何规划决策：对随包演示，带与不带资产层时
   `multifloor/sim/.probe/` 下的 `probe_plan.js`、`probe_sweep.js` 以及 Chrome 探针必须返回
   完全相同的路径；转换失败必须让构建明确失败——与 `build.py` 遇到未知 `route_rotation` 键的
   规则相同，因为被静默跳过的资产与"规划器主动避开"无法区分。

### 3.4 面向局部规划器与脱困的前馈 3DGS 先验

1. **预计算，绝不在机器人上做栅格化。** "前馈"指 splat 地图离线建好，运行时作为带标签的点
   先验消费——算力板上不做逐帧高斯栅格化，它没有这份预算（与把 DINOv3 原型限制为一次前向是
   同一个理由）。`scripts/map_processing/ply_to_pcd.py` 就是参考转换器：它丢弃低不透明度的
   漂浮点与超大高斯并写出 PCD，正是局部层需要的形状。
2. **它覆盖的盲区。** 三个反应式点流今天解决不了的失效：玻璃、镜面与深色表面返回稀疏
   LiDAR，NRMP 因此认为空间空闲；传感器高度带以上的几何不可见（站在踏面上的狗看不到自己
   所在的梯段——传感器最高只到 +0.30）；脱困的走廊判定（`in_corridor_now`、
   `corridor_half_width`、有界旋转上限）只读实时点，一个人挡住墙面就会翻转判定，要么解除
   脱困，要么过度触发。
3. **与实时点集相同的契约。** 先验以 map 坐标系进入，经过同样的滤波，并带逐点 `source`
   标签。脱困与 `adaptive_d_max` 以米为单位度量、用低分位数归约，正是为了不让一个杂点决定
   `d_max` 或旋转上限；未滤波的 splat 噪点会重新引入这种失效，不能豁免滤波。
4. **先验可以塑形，绝不触发。** 只有实时点可以累计停滞计数或锁存阻塞判定；先验只贡献间隙、
   侧墙与高度带以上的几何。一张早已搬走的托盘留下的陈旧 splat 绝不能发出倒车指令；实时
   自由空间射线穿过先验点时，该点必须消亡——这正是占据层用过的"回收"纪律：当时扫描回写
   没有与膨胀环一起重建，一次擦掉一格。
5. **槽位与验收门槛。** 通过既有的 `robot.yaml` → launch 参数 → 环境变量链路，用
   `local_planner.prior.backend` 选择它，与 `terrain_recognizer.backend`、
   `escape_policy.backend` 并列；不配置时行为与今天完全相同。先在 mesh 仿真里验证（先验扮演
   地图层，实时数据扮演 `state.obs` 层），再到真机：一段玻璃墙回放，纯实时规划会撞进玻璃，
   先验辅助的运行绕开它；以及一段移动障碍回放，只有先验时绝不能触发脱困。
   `test_escape_policy_slot.py` 必须原样通过。

### 3.5 世界模型三维导航（Amap ABot 级）

Amap 的 ABot 在可交互世界上叠加了一个**可导航世界模型**：数据层（`ABot-3DGS`，把扫描空间
变成可编程环境的数字孪生工厂；`ABot-PhysWorld`，保持生成数据的物理一致性）、导航基础模型
（`ABot-N`：跨场景室内外路线规划与空间理解）、操作基础模型（`ABot-M`），并由
predict → act → compare → self-correct 闭环收尾。ONS2 有数据层、没有学习模型层：今天的三维
导航从头到尾都是*几何的*（`map_planner` 在 `sliding_map` 上的膨胀 A\*、同一 `PlanPath` 契约
背后的 `octo_planner_ros1` 三维变体、识别器写入的踏面/平台标记）。目标是世界模型额外提供的
通行*可供性*——杂乱、无标识或从未见过的空间允许怎么走——而不是整体替换规划器。

1. **增量提议层，而不是新规划器。** 模型可以*排序或提议*路线与可供性区域；决定权仍在确定性
   链路。提议以额外的途经点 / 航点偏好交给既有 `PlanPath` 契约，于是同一套 `validatePath()`
   闸门、安装闸门与 `escape_route_gate.h` 验收规则会像对待 `mission_manager` 生成的路线一样
   筛它。幻觉路线必须*无法被安装*，而不只是"不太可能"——这与 §3.4 第 4 条是同一种权限划分
   （塑形，绝不触发），也是该特性能在默认不变规则下保持按需开启的原因。
2. **经由既有槽位进入，一个面一个槽。** 地形语义走学习型 `terrain_recognizer` 实例（工厂里
   的新后端，按固定 `type` 词汇发布 `TerrainObservation` / `TerrainScene`）；局部恢复走
   `local_planner.escape_policy.backend`（注册表已接受暴露 `create_policy(cfg)` 的点分模块）；
   路线可供性只需一个新槽位——`global_planner.affordance.backend`，与另外两个走同一条
   `robot.yaml` → launch 参数 → 运行时环境变量链路。
3. **`ABot-3DGS` 与 §3.4 是同一份资产。** 世界模型的数据层消费三维高斯场景；§3.4 已把
   splats 转成 PCD（`scripts/map_processing/ply_to_pcd.py`）喂给局部层，而
   [sim/](multifloor/sim/README.md) 是一座微型数字孪生工厂：`make_example_*.py` 创作地形、
   §3.3 的 SimReady 道具导入、以及可用探针验证的规划决策。训练/评估语料应建在这条流水线上
   而不是另起一条，这样一份场景无需重新导出就既是仿真回归 fixture，又是模型数据。
4. **推理留在权限路径之外。** VLA 级模型跑不动机器狗的计算板——SPEC 的板端预算正是把
   DINOv3 原型压到一次前向的同一约束（§3.1.4）。要么蒸馏到该预算内，要么跑在 VLM 终端已在
   用的边缘主机上，带硬超时；不可达、迟到或缺失时机器人照常导航。每个学习面都遵守弃权
   契约：`valid=false` / 不表态，绝不猜测（§3.1.5 的护栏——一个错误的 `stair_up` 会抑制
   脱困，把机器人困在真实楼梯脚下）。
5. **验收门槛。** 先在 mesh 仿真里证明——把 ABot 级可供性提议与随包演示的 A\* 基线对比，
   "更好"必须意味着更短或更宽，*并且*仍通过 `test_recognizers.js`——再到真机做两组回放：
   一条几何代价模型没有词汇描述的全新杂乱走廊，以及一次跨层交接。非回归是硬性的：新槽位
   未配置时，`probe_plan.js`、`probe_sweep.js`、`probe_nogo.py`、`probe_autonogo.py`、
   `probe_escape_contract.js`（均在 `multifloor/sim/.probe/` 下）与
   `test_escape_route_gate.cpp` 必须给出完全相同的判定。
6. **闭环最后做。** 定义 ABot 栈的 predict → act → 残差反馈自纠正，是 ONS2 最没准备好的
   部分：它需要能跨重启保留的逐次转移记录，而定位仍归 Odin1 MindSLAM 所有，因此任何
   预测-实际对比都必须在 `map` 坐标系里对一个不知道模型存在的位姿源进行。尽早记录残差
   （`/neupan/mission_escape_trace` 的形状是先例），但在有可纠正的存储语料之前，自纠正保持
   不在范围内。

### 3.6 融合 LingBot-Navi 做全局规划与局部动态规划

[LingBot-Navi](https://github.com/Robbyant/lingbot-navi) 把可复用的三维空间记忆、物体
grounding 与图像定位，与视觉语言骨干和用于坐标导航的轨迹解码器结合起来。其参考局部规划器
从当前渲染视图与位姿出发，跟随预测路线。ONS2 应把它集成为**既有规划器槽位背后的两个
适配器**，而不是把上游 demo 当作现成 ROS 规划器：已发布的 3DGS 执行示例只做运动学更新、
没有碰撞物理，因此动态障碍安全必须来自实时输入与 ONS2 的确定性闸门。

1. **`PlanPath` 背后的全局适配器。** 新增按需开启的
   `global_planner.backend: lingbot_navi` 适配器，对外提供既有的
   `nav_interfaces/srv/PlanPath` 契约。把 ONS2 的起点、目标、当前楼层坐标系与空间记忆翻译成
   路线请求，再返回 map 坐标系的 `nav_msgs/Path`。无路线、低置信度与超时都是失败，绝不编造
   航点。路线仍须通过 `validatePath()` 与 `mission_manager` 安装闸门；模型无法绕过它们注入
   路径。
2. **面向动态执行的局部适配器。** 以 `interface.mode: action` 选择
   `local_planner.backend: lingbot_navi`，保留既有 `nav_path` action、`stop_topic` 与
   `cmd_vel` 契约。用时间同步的实时相机/深度或投影 LiDAR 加 MindSLAM/TF 位姿替换上游 3DGS
   渲染器，并把每个预测的转向/前进动作转换成有界 `geometry_msgs/Twist`。实时点云间隙、
   footprint 约束与速度/加速度上限仍是权威。停止、抢占或无效观测必须把 `/cmd_vel` 置零；
   回退到 NeuPAN 需要显式交接，单发布者不变量绝不能破。
3. **空间记忆是先验，不是位姿权威。** 用 §3.3 与 §3.4 同一批场景语料与资产构建 LingBot-Map
   记忆，在模型需要的地方保留源 splats，并从同一来源派生 ONS2 几何。MindSLAM 仍是定位与
   楼层的所有者。LingBot-Navi 的图像定位估计只有在与当前 MindSLAM/TF 位姿的偏差处于配置
   范围内时，才可以参与路线请求；分歧、地图变化或记忆陈旧必须产生弃权，而不是重定位跳变。
4. **GPU 推理留在控制路径之外。** 上游版本要求 Linux x86_64、Python 3.10+、NVIDIA GPU 与
   CUDA 12.8，并在 H20-3E 硬件上报告评测结果。把模型跑在 ROS 适配器背后的边缘主机上，带
   硬性请求截止时间、序号与陈旧帧拒收；机器人侧适配器绝不能在推理上阻塞控制回调。主机
   不可达或迟到时，退化为失败的全局请求，或有控制的局部停车加确定性回退。在上游代码、
   权重与数据集的许可证完成审查之前，把它们保持为可选的外部依赖。
5. **验收门槛。** 先复现上游静态 SR/SPL 基线，再在 mesh 仿真里用移动障碍、遮挡、楼梯/坡道
   与受限/禁行区域，对照随包的 `map_planner` + NeuPAN 基线测试。比较成功率、SPL、最小
   间隙、安全干预次数与端到端延迟。最后做低速 ROS 1 与 ROS 2 回放，覆盖传感器掉线、推理
   延迟、记忆陈旧与移动障碍；后端未被选择时，所有既有探针与契约测试必须保持不变。

---

## 4. 兼容性规则（不得回退）

- **默认不变。** 不配置时仍是 `stuck_escape`，也不启动地形节点。
- **消息超集。** `TerrainObservation` 覆盖旧版 `/stair_gait/status` JSON 暴露的全部字段——
  `state`、`steps`、`rise`、`span` 对应 `legacy_state`、`steps`、`rise_m`、`span_m`——并额外
  提供 `confidence`、`geometry`、`backend` 与 `instance`；节点同时重发布该旧版 JSON
  （`state`、`steps`、`rise`、`span`）。只有在旧版驱动节点仍占用该话题时才关闭重发布
  （`publish_legacy: false`）；同一话题两个发布者是 bug，不是特性。
- **一份旧版结论。** 无论跑多少个实例，`/stair_gait/status` 始终是一份归约后的结论，且只有
  与前方走廊重叠的实例可以投票。否则侧向区域就能在机器人正走向楼梯时抑制脱困。
- **同一套测试。** 新脱困策略只有通过参考套件才被接受；新地形后端只有在其输入缺失时退化为
  `valid=false` 而不是给出猜测，才被接受。
