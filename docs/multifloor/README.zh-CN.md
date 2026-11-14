# 跨楼层导航（Go2 EDU + Odin1 + ONS2）

本目录说明如何在官方 ONS2 栈上启用**四层办公楼跨楼层导航**。基线能力（建图、重定位、
NeuPAN 动态避障、`map_planner` 全局规划、`mission_manager` 任务调度）全部复用 ONS2 原生实现，
本分支只补三件事：**楼层地图服务**、**切层编排**、**任务层跨层调度**。

```
                    ┌──────────────────────────────┐
  任务层  ────────► │ mission_manager              │  waypoint.floor 驱动
                    │  + ELEVATOR_TRANSIT 状态      │  电梯口停靠 / 等待 / 续跑
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
  -DCATKIN_WHITELIST_PACKAGES="nav_interfaces;odin_ros_driver;map_planner;sliding_map;neupan_ros;mission_manager;atomic_move;floor_manager;sw01_ros_control;dobot_ros_control"

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
2. 到达电梯口（容差 `elevator_arrive_tolerance_m`，默认 0.30 m）→ 停 NeuPAN，进入
   `ELEVATOR_TRANSIT`，状态与事件 `WAITING_ELEVATOR` 上浮到 Web/Foxglove；
3. 人工（或电梯联动）把机器人推进轿厢、到目标层推出，然后在目标层电梯口执行：

```bash
rosservice call /floor_manager/arrived_at_floor "floor: '3F'"
```

4. `floor_manager` 依次完成：确认静止 → 暂停任务（best effort）→ 改写
   `control_command.yaml`（`custom_map_mode: 2` + `relocalization_map_abs_path`）→
   `scripts/ons2_odin_restart.sh` 重启驱动 → 等 `map→base_link` TF 稳定 →
   发布该层 `/map`（latched）→ 等 `/inflated_map` 与 `/map_planner/plan` 就绪 →
   清 `sliding_map` → 发布 `FloorStatus{current_floor=3F, switching=false}`；
5. `mission_manager` 看到状态 → `current_floor_ = 3F` → 用 3F 地图继续原航点。

> 电梯轿厢内 Odin1 无法重定位（视野被金属轿厢封闭），**跨层必须人工或电梯 API 介入**，
> 这是本方案的既定边界，不做自动化。

---

## 6. 关键参数

| 参数 | 位置 | 默认 | 说明 |
| --- | --- | --- | --- |
| `enable_multifloor` | `mission_manager.launch` | `false` | 关闭时行为与官方单层完全一致 |
| `initial_floor` | `mission_manager.launch` / `floor_manager.launch` | `1F` | 两侧必须一致 |
| `elevator_arrive_tolerance_m` | `mission_manager.launch` | `0.30` | 判定"已在电梯口"的半径 |
| `elevator_wait_timeout_sec` | `mission_manager.launch` | `0.0` | 0 = 无限等；>0 超时跳过该航点 |
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
5. **单跨层**：1F→2F 一个航点，观察状态进入 `ELEVATOR_TRANSIT`、人工确认后恢复；
6. **多跨层**：1F→3F→1F，确认 `home_pose_valid_` 复位、返航只在当前层生效。

---

## 8. 已知限制

- **电梯内无定位**：跨层由人工/电梯 API 触发，不做无人值守电梯对接；
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
| 状态卡在 `ELEVATOR_TRANSIT` | `rostopic echo /floor_manager/status` 是否 `current_floor==目标 && switching==false`；`/floor_manager/arrived_at_floor` 是否调用成功 |
| 切层后规划穿墙 | `/map` 的 `info.origin` 是否与该层 yaml 一致；`sliding_map` 是否清空；TF `map→base_link` 是否稳定 |
| `/map` 双 publisher | `enable_map_server` 必须为 `false`（`ons2_go2_multifloor.sh` 已设置） |
| 驱动重启后无点云 | `scripts/ons2_odin_restart.sh --dry-run` 检查命令；`go2_logs/odin_restart_*.log` 看 Gate1/Gate2 |
| 电梯口判定失败 | `elevator_arrive_tolerance_m` 与 `floors.yaml` 的 `elevator` 坐标是否与实际一致 |
