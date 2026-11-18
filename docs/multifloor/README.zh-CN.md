# 跨楼层导航（Go2 EDU + Odin1 + ONS2）

本目录说明如何在官方 ONS2 栈上启用**四层办公楼跨楼层导航**。基线能力（建图、重定位、
NeuPAN 动态避障、`map_planner` 全局规划、`mission_manager` 任务调度）全部复用 ONS2 原生实现，
本分支只补三件事：**楼层地图服务**、**切层编排**、**任务层跨层调度**。

```
                    ┌──────────────────────────────┐
  任务层  ────────► │ mission_manager              │  waypoint.floor 驱动
                    │  + FLOOR_TRANSIT 状态        │  过渡点停靠 / 等待 / 续跑
                    └───────────┬──────────────────┘
                                │ /floor_manager/status (latched)
                    ┌───────────▼──────────────────┐
  切层编排 ───────► │ floor_manager                │  停运动→改配置→重启驱动→
                    │  switch_floor / arrived_at_floor│  等TF→发 /map→清滑动地图
                    └───────────┬──────────────────┘
                                │ 改写 control_command.yaml + 重启 odin 驱动
                    ┌───────────▼──────────────────┐
  楼层地图 ───────► │ /map (latched) + /odin1 重定位 │  每层 <stem>.yaml/.pgm + <stem>.bin
                    └──────────────────────────────┘
```

> **红线**：每层是一张独立地图，各有独立原点。切层后 `map` 坐标系被**重新定义**为该层地图原点，
> 跨层坐标不可比较、不可拼接。不要在两层之间做坐标换算或把 `.bin` 拼成一张 3D 图。

---

## 1. 前置条件

| 项 | 要求 |
| --- | --- |
| 机器人 | Go2 EDU + Jetson Orin Nano 8GB 拓展坞（`192.168.123.18`，用户 `unitree`） |
| 系统 | Ubuntu 20.04 + ROS1 **Noetic**（Go2 官方只支持 ROS1） |
| 传感器 | Odin1 已 udev 授权、USB3 识别、`use_host_ros_time` 已配置 |
| 依赖 | NeuPAN 的 Python ≥3.10 环境（`NEUPAN_PYTHON` 已 export） |
| 地图 | 每层一套 `<stem>.yaml` + `<stem>.pgm` + `<stem>.bin`，三者同名 |

检查清单：

```bash
lsusb | grep -i 2207                    # Odin1 应出现 idProduct 0019
ls /etc/udev/rules.d/99-odin-usb.rules   # 权限规则已装
python -c "import neupan, torch; print(torch.__version__)"
```

---

## 2. 每层地图准备

### 2.1 建图（逐层，一次一层）

```bash
# 拓展坞上，逐层执行；每次只建一层，建完立刻保存
bash scripts/ons2_go2.sh --map=floor1          # 仅用于起 odin 驱动 + rviz 观察
# Odin1 侧：custom_map_mode: 1 (SLAM)，save_map 触发保存 -> customer_file/floor1.bin
```

建图时**电梯口必须走到**，并在电梯轿厢门口（不进轿厢）停下保存，作为该层电梯等待点坐标。

### 2.2 `.bin` → 2D 栅格

```bash
# MindCloud Studio 导出 floorN.pcd，或直接用 recorddata 数据导出
python3 scripts/map_processing/pcd_to_pgm.py \
    --pcd floor1.pcd --out ros_ws/src/map_planner/maps/floor1 \
    --min-height -0.10 --max-height 0.30 --resolution 0.05
# 产出 floor1.pgm + floor1.yaml，检查 yaml 里的 origin 是否落在 Odin1 建图起点附近
```

`pcd_to_pgm` 输出的 yaml 已经带 `resolution / origin / negate / occupied_thresh / free_thresh`，
`floor_manager` 用与 `map_server` 相同的规则解析（P5/P2 都支持、行序自下而上翻转）。

### 2.3 登记楼层

编辑 `robots/go2/floors.yaml`：

```yaml
default_floor: "1F"

floors:
  - id: "1F"
    name: "一层大厅"
    map_yaml: "${REPO_ROOT}/ros_ws/src/map_planner/maps/floor1.yaml"
    reloc_bin: "${REPO_ROOT}/customer_file/floor1.bin"
    elevator: {x: 6.20, y: 1.80, yaw: 0.0}     # 该层 map 系下的电梯等待点
  # 2F ~ 4F 同理，改 id / 文件名 / elevator 坐标
```

`${REPO_ROOT}` 由 `floor_manager` 展开（`~repo_root` 参数，兜底 `GO2_ROOT` 环境变量）。
`elevator` 必须离电梯门口 ≥1.2 m，给 NeuPAN 留停车和转向空间。

---

## 3. 编译

```bash
cd ros_ws
# 白名单必须包含新包 floor_manager
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES="nav_interfaces;odin_ros_driver;map_planner;sliding_map;neupan_ros;mission_manager;atomic_move;floor_manager;octo_planner_ros1;stair_gait;sw01_ros_control;dobot_ros_control"

source devel/setup.bash
rospack find floor_manager
rossrv show nav_interfaces/SwitchFloor
```

首次编译注意：

- `nav_interfaces` 新增 `FloorStatus.msg` / `SwitchFloor.srv` / `ArrivedAtFloor.srv`，必须先编它；
- `sliding_map` 新增了 `std_srvs` 依赖，已写进 `CMakeLists.txt` 与 `package.xml`；
- `odin_ros_driver` 是 submodule，未初始化时先
  `git submodule update --init ros_ws/src/odin_ros_driver`。

---

## 4. 运行

```bash
# 单层回归（不启用跨层，行为与官方一致）
bash scripts/ons2_go2.sh --map=floor1

# 跨楼层模式：map_server 关闭，/map 由 floor_manager 提供
bash scripts/ons2_go2_multifloor.sh --map=floor1
INITIAL_FLOOR=2F bash scripts/ons2_go2_multifloor.sh --map=floor2   # 从 2F 开始
```

tmux 会话 `ons2_go2` 内的窗口：

| 窗口 | 内容 |
| --- | --- |
| `roscore` / `odin` | 主节点与 Odin1 驱动（Gate1/Gate2 通过后才继续） |
| `go2` | `go2.launch`（TF、`unitree_control`、`pc_to_scan`、`sliding_map`、`map_planner`、`neupan`） |
| `mission` | `mission_manager`（`enable_multifloor:=true`） |
| `floor` | `floor_manager`（仅跨层模式） |
| `bridge` / `health` / `control` | rosbridge 9090、健康检查、控制桥 |

停止：`bash scripts/ons2_go2_stop.sh`

---

## 5. 下达跨层任务

`Waypoint.msg` 新增了 `string floor`（空 = 当前层，向后兼容）。跨层任务示例：

```bash
rosservice call /waypoint_manager/start_mission "{
  loop: false,
  temporary: false,
  waypoints: [
    {name: '一层前台', x: 3.0, y: 0.5, yaw: 0.0, floor: '1F', dwell_sec: 5.0},
    {name: '三层会议室', x: 8.5, y: -2.0, yaw: 1.57, floor: '3F', dwell_sec: 10.0},
    {name: '一层前台', x: 3.0, y: 0.5, yaw: 3.14, floor: '1F', dwell_sec: 0.0}
  ]}"
```

执行逻辑：

1. 航点 `floor` ≠ 当前层 → 先规划到**本层电梯口**（`floors.yaml` 的 `elevator`）；
2. 到达该层过渡点（容差 `transition_arrive_tolerance_m`，默认 0.30 m）→ 停 NeuPAN，
   进入 `FLOOR_TRANSIT`，事件 `WAITING_TRANSITION` 上浮到 Web/Foxglove；
3. 到达过渡点后，按 `floors.yaml` 里该层 `transitions` 的配置完成穿越：

| type / mode | 谁完成穿越 | 适用条件 |
| --- | --- | --- |
| `stairs` / `manual` | 人牵引或抱运过楼梯 | **无电梯 API 时的默认选择** |
| `stairs` / `autonomous` | hook 里的爬楼动作序列 | 楼梯踏步实测通过后才可启用 |
| `lift` / `api` | hook 调电梯协议（HTTP / Modbus / 干接点） | 物业提供接口或继电器板 |
| `lift` / `manual` | 人按电梯、推狗进出 | 最常见（无任何接口） |
| `ramp` / `autonomous` | 直接走过去 | 有坡道，可全自主 |

4. 无论哪种方式，结束动作都是同一条命令（`api` / `autonomous` 由 hook 内部调用）：

```bash
rosservice call /floor_manager/arrived_at_floor "floor: '3F'"
```

5. `floor_manager` 依次完成：确认静止 → 暂停任务（best effort）→ 改写
   `control_command.yaml`（`custom_map_mode: 2` + `relocalization_map_abs_path`）→
   `scripts/ons2_odin_restart.sh` 重启驱动 → 等 `map→base_link` TF 稳定 →
   发布该层 `/map`（latched）→ 等 `/inflated_map` 与 `/map_planner/plan` 就绪 →
   清 `sliding_map` → 发布 `FloorStatus{current_floor=3F, switching=false}`；
6. `mission_manager` 看到状态 → `current_floor_ = 3F` → 用 3F 地图继续原航点。

> **共同边界**：不管是电梯还是楼梯，**穿越段内 Odin1 都不可能有可靠定位**
> （电梯是金属封闭轿厢，楼梯是窄小垂直空间 + 台阶点云退化）。所以穿越段一律是
> 「短时开环 + 到层后重新重定位」，区别只在于"谁来驱动这段位移"：人、电梯 API，
> 还是机器人自己的爬楼步态。

---

## 6. 关键参数

| 参数 | 位置 | 默认 | 说明 |
| --- | --- | --- | --- |
| `enable_multifloor` | `mission_manager.launch` | `false` | 关闭时行为与官方单层完全一致 |
| `initial_floor` | `mission_manager.launch` / `floor_manager.launch` | `1F` | 两侧必须一致 |
| `transition_arrive_tolerance_m` | `mission_manager.launch` | `0.30` | 判定"已在过渡点"的半径 |
| `transition_wait_timeout_sec` | `mission_manager.launch` | `0.0` | 0 = 无限等；>0 超时跳过该航点 |
| `transition_hook_timeout_sec` | `mission_manager.launch` | `180.0` | 外部 hook 单次最长执行时间，超时被 `timeout` 打断 |
| `repo_root` | `mission_manager.launch` | `GO2_ROOT` | 展开 hook 里的 `${REPO_ROOT}` |
| `repo_root` | `floor_manager.launch` | `""` | 展开 `${REPO_ROOT}`，缺省用 `GO2_ROOT` |
| `reloc_stable_sec` | `floor_manager.launch` | 2.0 | 切层后 `map→base_link` 需连续稳定的时长 |
| `reloc_timeout_sec` | `floor_manager.launch` | 120.0 | 重定位 + TF 稳定的最长等待 |
| `planner_ready_timeout_sec` | `floor_manager.launch` | 8.0 | 等 `/map_planner/plan` 就绪 |
| `inflate_wait_timeout_sec` | `floor_manager.launch` | 6.0 | 等 `/inflated_map` 就绪 |
| `auto_publish_initial_map` | `floor_manager.launch` | `true` | 启动时发布初始层 `/map`（map_server 关闭时必须为 true） |
| `odin_restart_script` | `floor_manager.launch` | `scripts/ons2_odin_restart.sh` | 重启驱动 |
| `odin_restart_mode` | `floor_manager.launch` | `process` | `process` 或 `systemd`（`odin_restart_unit` 指定服务名） |

---

## 7. 验证顺序（务必按序）

1. **单层回归**：`ons2_go2.sh --map=floor1` 跑 2 个普通航点，确认与改动前一致；
2. **`/map` 归属**：跨层模式启动后 `rostopic info /map` 应只有 `floor_manager` 一个 publisher；
3. **切层空跑**：不下任务，直接
   `rosservice call /floor_manager/switch_floor "floor: '2F'"`
   看 `/floor_manager/status` 与 `rostopic echo /map` 的 `info.origin` 是否切到 2F；
4. **清图生效**：切层前后 `rostopic echo -n1 /sliding_map`，确认旧层残留格点被清空；
5. **单跨层**：1F→2F 一个航点，观察状态进入 `FLOOR_TRANSIT`、人工确认后恢复；
6. **多跨层**：1F→3F→1F，确认 `home_pose_valid_` 复位、返航只在当前层生效。

---

## 8. 已知限制

- **穿越段无定位**：电梯轿厢 / 楼梯间内 Odin1 必然退化，跨层一律"短时开环 + 到层重定位"，不做穿越段连续导航；
- **楼梯能否走要先实测**：踏步深度/踢面高度、防滑条、转角平台决定 Go2 能否稳定通过，代码不能替代实测；
- **`map` 原点随层变化**：跨层后历史坐标作废，任何缓存位姿（`home_pose_`）都要重置（已实现）；
- **Odin1 不能热切模式**：必须 stop → 改配置 → 重启，`floor_manager` 强制走这条路径；
- **SRU 局部规划器未适配跨层**：`local_planner=sru` 时电梯流程会停靠，但续跑目标仍是真实航点，
  跨层请用 `local_planner=neupan`（默认）；
- **返航 `return_home` 只在当前层有效**：`home_pose_` 是进入该层后的捕获点；
- **首次编译未做**：本分支代码在无 ROS 环境上完成，`catkin_make` 需在拓展坞上执行，
  编译报错优先查 `nav_interfaces` 生成顺序与 `std_srvs` 依赖。

---

## 9. 排障

| 现象 | 排查 |
| --- | --- |
| 状态卡在 `FLOOR_TRANSIT` | `rostopic echo /floor_manager/status` 是否 `current_floor==目标 && switching==false`；`/floor_manager/arrived_at_floor` 是否调用成功 |
| 切层后规划穿墙 | `/map` 的 `info.origin` 是否与该层 yaml 一致；`sliding_map` 是否清空；TF `map→base_link` 是否稳定 |
| `/map` 双 publisher | `enable_map_server` 必须为 `false`（`ons2_go2_multifloor.sh` 已设置） |
| 驱动重启后无点云 | `scripts/ons2_odin_restart.sh --dry-run` 检查命令；`go2_logs/odin_restart_*.log` 看 Gate1/Gate2 |
| 过渡点判定失败 | `transition_arrive_tolerance_m` 与 `floors.yaml` 里该层 `transitions[].pose` 是否与实际一致 |

---

## 10. 楼梯专项（没有电梯 API 时的主力路径）

办公楼的电梯 API 大多数谈不下来，楼梯是更现实的跨层通道。但楼梯不是"坡道"，
它把三个前提同时打破：定位、规划、制动。下面按"先判断能不能走 → 再决定谁来过 →
再定失败保护"的顺序处理。

### 10.1 先做物理可行性判断（代码之前）

| 指标 | 经验下限 | 为什么 |
| --- | --- | --- |
| 踏步深度 | ≥ 260 mm | Go2 机身近 700 mm，踏步太浅时后腿踩不到下一级 |
| 踢面高度 | ≤ 180 mm | 超过后抬腿高度不够，容易磕碰前沿 |
| 楼梯净宽 | ≥ 900 mm | 留出走偏余量；宽度不足会蹭扶手 |
| 转角平台 | ≥ 1200 × 1200 mm | 狗需要平台完成转向，不能在半级踏步上转身 |
| 防滑条 / 台阶包边 | 无高凸起 | 凸起条会卡爪，是最常见的失败原因 |
| 楼梯口地面 | 平整、有 ≥1.2 m 驻停区 | 供切层前停稳、切层后重定位 |

这六项必须**现场逐层实测**，同一栋楼不同楼梯间也可能不一致。任一项不满足，
这一层就只能走电梯或人工抱运，不要试图用代码绕过去。

### 10.2 三种爬楼策略（按风险从低到高）

| 策略 | 配置 | 谁驱动位移 | 什么时候用 |
| --- | --- | --- | --- |
| A 人工牵引 | `stairs` / `manual` | 人牵着或遥控器控制 | **第一次落地必须先走这条**，用来验证切层链路本身 |
| B 半自主（人监护） | `stairs` / `autonomous` + `STAIR_CLIMB_CMD` | 机器人自己爬，人跟在旁边随时接管 | 物理条件达标、A 已验证稳定后 |
| C 全自主 | `stairs` / `autonomous` + 标定过的动作序列 | 机器人自己爬，无人监护 | 需要踏步参数标定 + 端点位姿校验，办公楼场景不建议 |

建议：**A 跑通 → B 演示 → C 只在固定楼梯间、固定路线下考虑**。

### 10.3 楼梯段的定位处理（核心）

楼梯段必须按"短时开环"处理，具体四条：

1. **进入楼梯口前记录最后可信位姿**：此时 `map→base_link` 仍有效，记下当前位姿作为穿越起点；
2. **楼梯段只用相对推进**：里程计 / IMU 推算，禁止调用 `map_planner`、禁止依赖 `/scan` 建图；
3. **到层后立刻重新定死**：`floor_manager` 重启驱动 + 用该层 `.bin` 重定位，`map→base_link` 重新建立；
4. **端点位姿校验**：重定位后的位姿应与"该层楼梯口期望位姿"在 0.5 m 内，超出判为穿越失败。

第 4 条是唯一能自动发现"爬错半层/爬过头"的检查，**不要省**。

### 10.4 为什么不能"边爬边规划"

- `map_planner` 与 NeuPAN 都是 **2D 平面规划器**，台阶是三维结构，二维代价地图表达不了；
- Odin1 在踏步上以掠射角打点云，`/scan` 会大量缺失，`sliding_map` 会出现"假空洞"；
- 楼梯上**不能中途停**：Go2 在踏步中间重新起步的成功率远低于连续通过，所以"走走停停的避障"在楼梯上是负收益。

结论：楼梯段不做避障、不做重规划，只做**定速通过 + 端点校验**。

### 10.5 代码怎么接

1. `robots/go2/floors.yaml` 里给该层配 `stairs` 过渡点，坐标是**该层 map 系下的楼梯口**；
2. 先保持 `mode: manual`，用人工牵引验证 `FLOOR_TRANSIT → FloorStatus → 续跑` 全链路；
3. 验证通过后改成 `mode: autonomous` 并填 `hook: scripts/transition/stairs_auto_example.sh`；
4. 把真正的爬楼动作填进 `STAIR_CLIMB_CMD`（宇树步态/运动接口按你手上的 SDK 版本对接），
   hook 负责"停车 → 爬楼 → 到层停车 → 调 `arrived_at_floor`"的编排与失败兜底。

`mission_manager` 以 `FLOOR_TRANSITION_FROM/TO/TYPE/MODE` 环境变量把上下文传给 hook，
并用 `timeout --signal=INT` 限制单次执行时间，**hook 卡死不会拖死状态机**。

### 10.6 失败保护

| 失败 | 系统行为 |
| --- | --- |
| hook 退出非 0 | 记录 `/tmp/ons2_floor_transition.log`，状态保持 `FLOOR_TRANSIT` |
| hook 超时（`transition_hook_timeout_sec`） | `timeout` 打断，按失败处理 |
| 等待楼层确认超时（`transition_wait_timeout_sec > 0`） | 跳过该航点继续任务 |
| 重定位端点校验超 0.5 m | 建议人工介入，不要自动续跑 |

> 现场第一次上楼梯前，把 `transition_wait_timeout_sec` 设为 0（无限等），
> 让人能随时接管；跑熟之后再设超时。

---

## 11. 相关文档

| 文档 | 讲什么 |
| --- | --- |
| [3D 规划与楼梯](3d-planner-and-stairs.zh-CN.md) | `astar3d`、楼梯语义、`stair_gait`、离线仿真验证 |
| [设计说明](design.zh-CN.md) | 楼层地图服务、切层状态机、任务层调度 |
| [步态控制台](gait_console.html) | 楼梯步态参数调试页 |
| [**4G 远程调试**](remote-debug.zh-CN.md) | 车身交换机 + 4G 路由的拓扑、IP 规划、隧道、带宽与供电 |
| [**局域网调试**](lan-debug.zh-CN.md) | 网线接交换机 / 路由本地 WiFi 两条路，静态 IP、ROS_IP 切换与丢包排查 |
| [**点云转 mesh 视图**](mesh-viewer.zh-CN.md) | 把点云抽成表面网格再实体渲染，可调透明度看跨层结构 |
| [**mesh 导入与 2D 栅格导出**](mesh-nav-import.zh-CN.md) | 导入 OBJ/STL、楼梯语义、净空带硬/软、擦与增禁行、存档，以及导出 `pgm`+`yaml` 与 `--map=` 的衔接 |
| [**局部规划器切换：NeuPAN / SRU**](local-planner-sru.zh-CN.md) | 开关接在哪、SRU 还缺什么、为什么跨层任务别选它 |
