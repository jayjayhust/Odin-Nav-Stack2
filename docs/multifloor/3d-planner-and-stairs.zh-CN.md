# 3D 规划器移植与楼梯步态接入

本文记录"选项 2"（把 `jie_3d_nav` 的 `octo_planner` 移植成 ROS1 全局规划器）与"楼梯语义识别 + 步态切换"这两项的落地方式、边界和验证顺序。

---

## 1. 先明确移植的是什么、不是什么

`octo_planner` 的 3D 是**规划空间的 3D**，不是**运动空间的 3D**。证据在它自己的参数和控制器里：

| 证据 | 含义 |
| --- | --- |
| `tracking_point_reached_xy_tolerance: 0.40` | 到达判定只看 XY |
| `goal_position_tolerance: 0.1` / `goal_yaw_tolerance: 0.2` | 目标只有位置与偏航 |
| `d1_controller` 为纯 P 控制 + 横移 + final yaw + deadband | 输出是平面速度指令，不跟踪 z / pitch / roll |
| D1 为轮式底盘 | 物理上不能爬楼梯 |

所以移植后它提供：

- **3D 通行性判定**：`require_ground_support` + `ground_support_xy_radius_cells` 检查脚下是否有支撑面，能排除"桌面下方、悬挑、楼梯井空洞"这类 2D 投影会误判为可通行的区域；
- **单层内更安全的全局路径**：预阻塞半径 + 机器人半径，比 2D 栅格膨胀更少穿桌腿；
- **跨层拓扑**（仅在单张 3D 图成立时）：A\* 会自己绕到楼梯口。

它**不提供**：

- 楼梯运动（需要宇树运动控制层的爬楼步态）；
- 楼梯间定位（Odin1 掠射角点云退化）；
- 实时避障（`jie_3d_nav` 没有，本方案保留 ONS2 的 NeuPAN 做局部避障）。

---

## 2. 三个新组件

| 组件 | 位置 | 职责 |
| --- | --- | --- |
| `octo_planner_ros1` | `ros_ws/src/octo_planner_ros1` | 3D 体素地图 + 3D A\*，暴露与 `map_planner` **完全相同的** `nav_interfaces/PlanPath` 服务 |
| `stair_gait` | `ros_ws/src/stair_gait` | 楼梯语义识别（up / down / flat + 踏步高度）+ Go2 爬楼步态切换闸门 |
| `stairs_gait_hook.sh` | `scripts/transition/` | 把上面两者编排成 `FLOOR_TRANSIT` 的 `autonomous` hook |

`mission_manager` **一行都不用改**：它只消费 `plan_service` 指向的 `nav_interfaces/PlanPath`。

---

## 3. 3D 规划器怎么接

### 3.1 与 2D 规划器并存，不替换

`robots/go2/robot.yaml` 里有 `global_planner: map_planner`，`map_planner.cpp` 用 `service_name` 参数发布服务。因此切换只需要改服务名：

```yaml
# 2D（默认，现状）
global_planner: "map_planner"

# 3D（单层内 3D 通行性判定）
global_planner: "octo_planner_ros1"
```

或直接给 `mission_manager` 的 `plan_service` 参数：

```bash
roslaunch mission_manager mission_manager.launch \
  plan_service:=/map_planner/plan
```

> 两个节点**不能同时**占用同一个服务名。启动 3D 规划器时，`go2.launch` 里请用 `enable_map_server:=0` 关掉 2D 的 `map_server`，或把其中一个的 `service_name` 改成 `/octo_planner/plan`。

### 3.2 参数与语义对齐

`config/octo_planner.yaml` 的参数名刻意与 `jie_3d_nav` 保持一致，便于对照调参：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `resolution` | 0.10 | 体素边长 |
| `robot_radius` | 0.25 | XY 膨胀半径，`ceil(0.25/0.10)=3` 格 |
| `inflation_z_cells` | 2 | Z 方向膨胀 |
| `max_iterations` | 500000 | A\* 展开上限，超时返回失败而不是拖死服务 |
| `require_ground_support` | true | 每步检查脚下支撑 |
| `ground_support_xy_radius_cells` | 1 | 支撑检查的 XY 窗口 |
| `snap_search_radius_cells` | 12 | 起终点吸附到最近可通行体素 |
| `max_step_z_cells` | 3 | 单次展开的最大 z 跳变，防止顺着未观测的井道"飞"到别的层 |
| `z_bias` | 0.0 | 偏离目标 z 平面的额外代价，用于偏好当前层 |

### 3.3 地图来源

两种，二选一：

```bash
# A. 预建点云（可复现，推荐先跑通）
rosrun octo_planner_ros1 octo_planner_node _pcd_file:=/path/to/floor1.pcd

# B. 实时累积（跟随 Odin1）
roslaunch octo_planner_ros1 octo_planner.launch cloud_topic:=/odin1/cloud_slam
```

`/odin1/cloud_slam` 发布在**里程计系**，节点通过 TF 变换到 `map_frame` 后再入图；TF 不可用时降级为原始坐标并只告警一次。

### 3.4 已知限制

- 体素图**一旦建立就不再扩展**：第一帧点云的包围盒 + `grid_margin_m` 决定了可用范围，后续超出范围的点被丢弃。单层测试够用；如果是整栋楼单图，需要先离线合并点云再加载。
- `deinflate()` 每次规划前 O(n) 扫描一遍（千万格量级约 10–20 ms），换来的是不需要维护第二份地图。
- 3D 路径的 `z` 保留在 `nav_msgs/Path` 里，但下游（`mission_manager` / NeuPAN）只用 `x,y,yaw`。**这不是缺陷**，是这套跟踪器的能力边界。

---

## 4. 楼梯语义识别

`stair_gait` 的检测器不分割整段楼梯，只回答一个问题：**机器人正前方的地面是上台阶、下台阶还是平地，每级多高**。

做法：把点云变换到 `base_link`，在 `x ∈ [0.8, 3.2] m`、`|y| ≤ 0.6 m`、`z ∈ [-0.75, 0.6] m` 的 ROI 内按 0.05 m 分 bin，取每 bin 的**最低点**作为地面剖面，再做 3 点中值滤波，然后扫描相邻 bin 的高度跳变：

- 跳变幅度落在 `[0.08, 0.30] m` → 记一次踏步；
- 连续同向 ≥ 2 次 → 判为 `up` / `down`；
- 跳变幅度差超过 `0.06 m` → 判为 `unknown`（拒绝给出错误步态）；
- 连续 `confirm_frames` 帧一致才发布，抑制抖动。

输出 `/stair_gait/status`（`std_msgs/String`，JSON）：

```json
{"state":"up","steps":3,"rise":0.168,"span":0.93,"confidence":0.75,
 "api_ids":{"gait":-1,"mode":-1},"actuation":false}
```

设计取向：**宁可判 `unknown` 也不误触发步态切换**。踏步高度不一致、剖面噪声大时直接放弃。

---

## 5. Go2 爬楼步态接口：查证结果

### 5.1 固件/ROS2 侧确实存在

宇树 Go2 ROS2 文档（`unitree-go2-robot.github.io`）明确列出：

| 接口 | 参数 | 取值 |
| --- | --- | --- |
| `/switch_gait`（`go2_interfaces/srv/SwitchGait`） | `int32 gait` | 0 IDLE / 1 TROT / 2 TROT_RUN_NING / **3 FORWARD_CLIMBING_MODE** / **4 REVERSE_CLIMBING_MODE** |
| `/mode`（`go2_interfaces/srv/Mode`） | `string mode` | 含 **`walkstair`**（"climb stairs in a walking pattern"）、`walkupright`、`freewalk` 等 |

Go1 的 `gaitType` 定义（3 = Stairs climbing）也印证了这套编号的谱系。

### 5.2 `unitree_sdk2` 的 C++ 头文件里没有

`include/unitree/robot/go2/sport/sport_api.hpp` 列出的 API ID 为 1001–1063、2041–2058，**不含 `SWITCHGAIT` 也不含 `WALKSTAIR`**；`sport_client.hpp` 的 `SportClient` 方法全集里同样没有。

这不是"接口不存在"，而是**没在这个公开头文件里注册**。宇树自己的教程（`serviceconsole.unitree.com` `#/help/030504`）讲"如何在 unitree_ros2 中添加接口"时也强调：**API ID 必须与 `unitree_sdk2` 中的定义一致，不能随意编造**。

### 5.3 本项目的处理方式

`stair_gait/include/stair_gait/ext_sport_client.h` 做了三件事：

1. 继承 `unitree::robot::go2::SportClient`；
2. 利用其 **protected** 的 `Call(apiId, param, data)` 与 `RegistApi(apiId)` 发任意 API；
3. 把 API ID 做成 ROS 参数 `switch_gait_api_id` / `mode_api_id`，**默认 -1 = 不调用**。

```cpp
int32_t SwitchGaitById(int32_t api_id, int32_t gait) {
  if (api_id < 0) return -1;
  RegistApi(api_id, 0);
  const std::string param = std::string("{\"gait\":") + std::to_string(gait) + "}";
  std::string data;
  return Call(api_id, param, data);
}
```

这样既没有编造 ID，也在接口存在时能真正调通。API ID 从你手上的 `unitree_ros2` 源码（`ros2_sport_client.h` 中 `ROBOT_SPORT_API_ID_SWITCHGAIT` / `WALKSTAIR` 常量）或宇树技术支持处取得后填入即可。

### 5.4 安全闸门

- `enable_actuation` 默认 **false**：所有服务直接返回 `success=false`，不碰机器人；
- `auto_switch` 默认 **false**：即使检测到楼梯也不自动切步态，必须由 hook 显式调用；
- 每个服务返回原始 `int32` 返回码，错误的 ID 会表现为非 0，而不是静默失效；
- `stairs_gait_hook.sh` 用 `trap` 保证异常退出时**先停车、再恢复 trot**。

---

## 6. 落地顺序（建议严格按此执行）

| 步 | 动作 | 通过标准 |
| --- | --- | --- |
| 1 | 只跑 3D 规划器，`pcd_file` 指向一层点云，用 `rosservice call /map_planner/plan` 打点 | 返回 `success=true`，`path.poses` 不穿墙、不悬空 |
| 2 | 把 `plan_service` 切到 3D，单层内跑一次完整任务 | 状态机正常，NeuPAN 仍在避障 |
| 3 | 只跑 `stair_gait`（`enable_actuation=false`），人走一遍楼梯看 `/stair_gait/status` | `up`/`down` 判定正确，踏步高度与实测一致 |
| 4 | 取得 API ID，`enable_actuation=true`，机器狗架空空跑（腿悬空）验证 `SwitchGait` 返回 0 | 三个服务返回码均为 0，机器人姿态按预期变化 |
| 5 | 楼梯口实地，`mode: manual` + 人工监护，跑 `stairs_gait_hook.sh` | 能上、能下、到层后重定位成功、端点校验通过 |
| 6 | 连续 20 次无失败后，才考虑去掉监护 | — |

**第 4 步不可跳过**。错误的 API ID 有可能落到别的 service 上，空跑是唯一安全的验证方式。

---

## 7. 尚未验证的部分

- 三个新包**未编译**（本机无 ROS1/Noetic，无 `unitree_sdk2`）；
- `SwitchGait` / `WalkStair` 的真实 API ID **未确认**，`stair_gait` 已按参数化处理；
- 3D 规划器在 Jetson Orin Nano 8GB 上的实际耗时未测。离线 x86 实测：单层 0.2–2 ms，三层 14 ms；按 Orin Nano 相对桌面 CPU 的 5–10 倍劣化估算，三层仍在一百至两百毫秒量级，未触及 2 s 的重规划周期；
- 楼梯检测的 ROI 与阈值按办公楼常规踏步（260–300 mm 深、150–180 mm 高）设定，实测需按现场微调。

---

## 8. 离线仿真（不需要 ROS）

`test/planner_sim.cpp` 直接链接 `src/voxel_grid.cpp` 和 `src/astar3d.cpp`——这两个文件只依赖 STL，所以能在一台没有 ROS、没有 PCL 的机器上用 `g++` 单独编译，跑的是和机器上完全相同的一份规划代码。

```bash
cd ros_ws/src/octo_planner_ros1
g++ -std=c++14 -O2 -I include test/planner_sim.cpp \
    src/voxel_grid.cpp src/astar3d.cpp -o /tmp/planner_sim
/tmp/planner_sim > ../../../../docs/multifloor/sim/planner_sim_data.json
python3 test/build_sim_report.py     # 生成 docs/multifloor/sim/planner_sim.html
```

HTML 是自包含的（数据内嵌，无外部库、无服务器），打开后可以切换场景、拖时间轴回放 A\* 的展开过程和出图过程。三个视图分工如下：

- **俯视 X-Y**：平面走向、A\* 的扩散范围。
- **侧视 X-Z**：按目标所在 y 切 ±3 格看高度剖面，路径的每次抬腿都对应一级台阶。
- **3D 点云**：体素中心做透视投影，拖拽旋转、滚轮缩放、双击复位。没有面遮挡，所以不需要画家排序，深度只用来决定点的大小和透明度。视锥按包围盒自动拟合，任意角度都不会裁出画布。颜色按高度从冷蓝到暖黄，楼层和爬升方向一眼可辨。

| 场景 | 构造 | 验证的语义 |
| --- | --- | --- |
| `office` | 8×6 m，两张悬空桌面 + 桌腿 | 能从桌面下方穿行，不是把整张桌子当墙 |
| `stairs` | 8 级（踏面 0.30 / 踢面 0.17）+ 上层平台 | 沿台阶逐级上升，不被踢面挡住 |
| `pit` | 地面 1×1 m 空洞，目标在洞中心 | 不把空洞当成可站立面，目标吸附到洞边 |
| `wall_start` | 起点距墙 0.15 m，整个人站在膨胀带里 | 仍能出图，不会一次展开就放弃 |
| `desk_barrier` | 桌面横跨整条走廊，只在桌下 0.75 m 留通道 | 2D 投影是死墙，3D 能穿过去 |
| `multi_floor` | 3 层办公楼，层高 3.4 m，折返楼梯井两段 18 级（踢面 0.189 / 踏面 0.30），起点 1 层、目标 3 层 | 跨 3 层出图：0 → 3.4 → 6.8 m，两次爬梯 |

六个场景共 14 条断言，当前全部通过（office expanded 1228 / stairs 650 / pit 97 / wall_start 1338 / desk_barrier 590 / multi_floor 11499，单次规划 0.2–14 ms）。

`multi_floor` 的几何值得单独记一笔：折返楼梯井里两段梯在**同一 x 区间、同一 y 区间**，只是 z 带不重叠（0–3.4 与 3.4–6.8）。这是真实办公楼把两跑塞进一个楼梯核心的做法，也说明 3D 规划器处理的不是"一张平面图加一个高度"，而是真正的三层体空间。在这个场景里 A\* 展开 1.15 万个节点、出 179 点路径，耗时 14 ms——放在 Orin Nano 上按 5–10 倍余量估算，仍在单次规划百毫秒内，可以接进 2 s 的周期重规划。

## 9. 仿真暴露的四个真实缺陷（已在代码里修掉）

### 9.1 Z 方向膨胀会吞掉脚下的站立格

`inflation_z_cells: 2` 的本意是防止机身擦到头顶的横梁。但在体素里，"地面"本身就是占一格高的 occupied，向上膨胀 0.2 m 正好把机器人站立的那一格变成 INFLATED —— 结果是地面和每一级踏步全部不可规划。

现在默认 `inflation_z_cells: 0`。头顶净空由 `clearance_cells` 保证：每插入一个表面点，就把它正上方 20 格（2 m）的 UNKNOWN 清成 FREE，这已经覆盖了机身高度。

### 9.2 XY 膨胀会吃掉下一级台阶的站立格（最关键的一条）

这一条只有真跑楼梯才暴露得出来。踏步升高 0.17 m ≈ 1.7 格，于是：

- 第 N 级踏步的表面占 `iz = 8`（0.51 m）；
- 第 N+1 级踏步的表面占 `iz = 9`（0.68 m），机器人站在第 N 级时正好也是 `iz = 9`。

第 N+1 级表面的 `robot_radius`（0.25 m = 3 格）XY 膨胀落回同一层 `iz = 9`，把第 N 级的站立格整片变成 INFLATED。搜索于是停在第三级，`expanded=684`、`no path found`。

修法不是把半径调小（调小就失去避障意义），而是给 `inflate` 加 `skip_standing_surfaces`：**正上方是 FREE 的 occupied 格是"可站立面"（地面、踏步、桌面），不参与膨胀；只有竖直结构（墙、桌腿、护栏）才膨胀**。这一条同时让单层走廊不再因为地面膨胀而损失 0.25 m 通行宽度。

### 9.3 起点落在膨胀带内时 A\* 一次都扩展不出去

机器人贴墙站（距墙 0.15 m）时，起点本身以及周围 0.3 m 全是 INFLATED。旧代码的端点判定是"INFLATED 也接受"，但邻居严格要求 `kFree`，于是 `expanded=1` 直接放弃。

改为**端点优先吸附到最近的 FREE 格**（`snap_search_radius_cells=12`，即 1.2 m 内找），实在找不到才退回 INFLATED。这样贴墙起步也能出图，代价是路径起点会平移 0.2–0.3 m —— 对 `mission_manager` 无影响，因为它只取路径做跟踪参考。

### 9.4 场景构建本身的两个坑（不是算法问题）

- `addWallY(x, ...)` 建的是**垂直于 x 轴**的墙，`addWallX(y, ...)` 建的才是沿 x 延伸的侧墙。楼梯场景一开始把侧墙写成了 `addWallY(0.0/2.6, ...)`，等于在梯段中间竖了两道横墙，把楼梯切断。
- 浮点误差：`(2.6 + 0.3) / 0.1 = 28.999…`，`floor` 之后落到 `ix = 28`，与踏步列重叠。场景里所有落在网格边界上的坐标都要留出余量，或者干脆用半格偏移。

## 10. 仿真的结论边界

仿真证明的是"几何上可规划"：3D A\* 能找到一条从一层走到另一层的、贴着踏步逐级上升的路径。它**不证明**机器人能执行这条路径——真正的爬楼动作仍然要靠 `stair_gait` 把步态切到 `FORWARD_CLIMBING_MODE`，由宇树的运动控制层完成。规划器给的是"往哪走、抬多高"，步态给的是"腿怎么动"，两者缺一不可。
