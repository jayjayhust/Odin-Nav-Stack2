# 跨楼层设计与接口契约

## 1. 设计取舍

ONS2 原生已具备 Go2（ROS1）验证、Odin1 重定位、`map_planner` 全局规划、NeuPAN 动态避障、
`mission_manager` 任务调度。缺的只有"同一个任务跨多张独立地图继续跑"。

因此不引入第二套建图/规划栈（如 OctoMap + 3D A*），而是在原栈上补三个职责边界清晰的模块：

| 模块 | 职责 | 不做什么 |
| --- | --- | --- |
| `floor_manager` | 切层的**原子操作**与楼层地图发布 | 不规划、不驱动底盘、不决定任务 |
| `mission_manager`（扩展） | 任务里的**跨层调度**（电梯口停靠、等待、续跑） | 不改地图、不重启驱动 |
| `sliding_map`（扩展） | 暴露 `clear` 服务，切层后清空动态障碍缓存 | 不管地图切换 |

这样单层场景零行为变化（`enable_multifloor` 默认 `false`），跨层能力可独立开关与回滚。

## 2. 接口契约

### 2.1 `nav_interfaces/Waypoint.msg`

```
string floor      # 新增，末尾追加；空 = 当前层（向后兼容）
```

向后兼容：不填 `floor` 的老任务、`move_base_simple/goal`、`dynamic_goal` 一律视为当前层。

### 2.2 `nav_interfaces/FloorStatus.msg`（latched）

```
string  current_floor   # 当前生效的楼层 id
string  target_floor    # 切换中的目标楼层，空闲时为空
bool    switching       # true = 切层进行中，禁止规划/运动
string  detail          # 人类可读阶段描述
```

`mission_manager` 只依赖这一个契约判断"切层是否完成"：`!switching && current_floor == 目标`。

### 2.3 `nav_interfaces/SwitchFloor.srv`

```
string floor
bool   keep_mission_paused
---
bool   success
string message
string floor
float64 elapsed_sec
```

供人工/上层调度直接切层（不下任务）。

### 2.4 `nav_interfaces/ArrivedAtFloor.srv`

```
string floor
---
bool   success
string message
string floor
```

跨层任务的**人工确认点**：机器人被推进目标层电梯口后调用，触发切图全流程。

### 2.5 `sliding_map` 新增服务

`/sliding_map_node/clear`（`std_srvs/Trigger`）：把 `map_.data` 全置 `-1`、时间戳归零并重发
`/sliding_map`。切层后旧层的动态障碍残留必须清掉，否则新层规划会被"幽灵障碍"挡住。

## 3. 状态机

`mission_manager` 新增 `ELEVATOR_TRANSIT`，其余状态不变：

```
NAVIGATING ──到达电梯口──► ELEVATOR_TRANSIT ──FloorStatus 确认──► NAVIGATING(新层)
     │                          ▲
     │ 到达真实航点              │ 已在电梯口
     ▼                          │
  ARRIVED ──► ALIGNING ──► DWELLING ──► 下一航点 ──┐
                                                    │ floor 不同
                                                    └──► 先规划到本层电梯口
```

判定顺序（`startCurrentWaypoint()` 开头）：

1. `target_floor = waypoint.floor.empty() ? current_floor_ : waypoint.floor`
2. `target_floor != current_floor_` → 取当前层 `elevator` 位姿
3. 机器人已在电梯口（`dist ≤ elevator_arrive_tolerance_m`）→ 直接 `ELEVATOR_TRANSIT`
4. 否则 `floor_transit_active_=true`，`planCurrentWaypoint()` 的 goal 换成电梯口
5. 到达电梯口后**不进入** `ARRIVED/ALIGNING/DWELLING`，直接 `ELEVATOR_TRANSIT`
6. `ELEVATOR_TRANSIT` 中不发任何速度指令，等 `FloorStatus`
7. 确认后 `current_floor_ = target_floor_`、`home_pose_valid_ = false`、续跑原航点

`currentGoalPose()` 统一了"当前该往哪走"，已被 `planCurrentWaypoint`、周期重规划、
recovery shadow、`distanceToTarget`、状态上报复用，避免跨层时误用旧层坐标。

## 4. 切层时序（`floor_manager::doSwitchFloor`）

| # | 动作 | 失败处理 |
| --- | --- | --- |
| 1 | 校验目标楼层在 `floors` 中、`map_yaml`/`reloc_bin` 可读 | 拒绝，`success=false` |
| 2 | 发 `/neupan/stop` + 连续零速 `/cmd_vel` | 记录告警，继续 |
| 3 | `/waypoint_manager/pause`（best effort） | 忽略失败（可能本就空闲） |
| 4 | 备份 `control_command.yaml`，改写 `custom_map_mode: 2` 与 `relocalization_map_abs_path` | 回滚备份，失败返回 |
| 5 | `scripts/ons2_odin_restart.sh` 重启驱动（stop→config→start 是 Odin1 硬约束） | 失败返回，保留备份供排查 |
| 6 | 等 `map→base_link` TF 连续 `reloc_stable_sec` 稳定（上限 `reloc_timeout_sec`） | 超时失败，提示查 Gate1/Gate2 日志 |
| 7 | 解析该层 `yaml/pgm`，latched 发布 `/map` | 解析失败回滚配置 |
| 8 | 等 `/inflated_map`（`inflate_wait_timeout_sec`）与 `/map_planner/plan` 就绪（`planner_ready_timeout_sec`） | 超时失败 |
| 9 | 调 `/sliding_map_node/clear` | best effort |
| 10 | 再发一次 `/neupan/stop`（清掉切图期间的旧路径） | best effort |
| 11 | 发布 `FloorStatus{current_floor=目标, switching=false}` | — |

全程持有一把互斥锁，`switch_floor` 与 `arrived_at_floor` 同时到达只会执行一次；
`/floor_manager/status` 在 2~11 之间持续为 `switching=true`，任务层据此不规划。

## 5. 为什么不拼一张全局图

- 每层 `.bin` 的原点是**各自建图起点**，层间 Z 与旋转没有可靠外参标定；
- 拼图需要层间重叠特征与统一基准，办公楼楼梯/电梯井恰好是纹理稀疏区；
- 即使拼成 3D OctoMap，2D 规划仍需投影，投影误差会直接变成穿墙风险。

结论：**一层一图 + 切层重定位**是当前硬件条件下最稳的路径，代价是跨层需要人工确认。

## 6. 后续可扩展点

- `ArrivedAtFloor` 背后接电梯厂商 API（Modbus/干接点），把人工确认替换为自动确认；
- `FloorStatus` 增加 `map_origin` 字段，供上层做楼层间粗略拓扑（如"A 栋 2F 到 B 栋 2F"）；
- 跨层任务加"电梯等待超时自动返航"策略（当前 `elevator_wait_timeout_sec` 只支持跳过航点）。
