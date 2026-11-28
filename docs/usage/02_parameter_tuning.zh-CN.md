# 02 · 参数调优

[English](02_parameter_tuning.md) | **中文**

本文列出各支持平台上真正重要的参数、改动顺序，以及一张症状对照表。下面的数值是当前仓库里的值。

> **本文与[高级调参指南](../TUNING_GUIDE.zh-CN.md)的分工：**
> 本文回答「**我这台机器人该填什么值**」—— 各平台实测默认值 + 调参顺序；
> 高级调参指南回答「**这个参数到底干什么、该怎么调**」—— 全参数词典 + 症状排障，
> 改动本文未列出的参数前请先读它。

## 1. 参数都在哪

| 层 | Unitree Go2（ROS1） | ASTRALL SW01（ROS1 / ROS2） | Dobot Lumo（ROS1） |
|---|---|---|---|
| 机器人档案（TF、外形、A\*、扫描切片） | `robots/go2/robot.yaml` | `robots/sw01/robot.yaml` | `robots/dobot_lumo/robot.yaml` |
| NeuPAN 局部规划器 | `robots/go2/neupan/config.yaml` + `planner.yaml` | `robots/sw01/neupan/config.yaml` + `planner.yaml` | `robots/dobot_lumo/neupan/config.yaml` + `planner.yaml` |
| 底盘桥 | `ros_ws/src/unitree_control` | `ros2_ws/src/sw01_ros_control` | `ros_ws/src/dobot_ros_control/launch/dobot_control_py.launch` |

NeuPAN 档案在 `robots/<robot>/neupan/` 下，ROS1 与 ROS2 共用。需要时用 `--neupan-config=PATH` 覆盖。**Go2 与 SW01 的档案千万不要混用。**

## 2. 各平台参考值

### Unitree Go2（ROS1）

| 参数 | 文件 | 取值 |
|---|---|---|
| `robot.length` × `robot.width` | `planner.yaml` | 0.70 m × 0.35 m |
| `robot.max_speed`、`robot.max_acce` | `planner.yaml` | `[0.625, 0.875]`、`[0.625, 0.875]` |
| `ref_speed` | `planner.yaml` | 0.625 m/s |
| `receding` × `step_time` | `planner.yaml` | 10 × 0.1 s |
| `ipath.arrive_threshold` | `planner.yaml` | 0.45 m |
| `adjust`：`q_s`、`q_theta`、`p_w`、`eta`、`d_max`、`d_min` | `planner.yaml` | 1.0、2.0、0.8、9.0、0.10、0.02 |
| `w_deadband`、`adaptive_d_max`、`smith_predictor` | `config.yaml` | 全部 `enable: false` |
| `stuck_escape` | `config.yaml` | `enable: true` |

### ASTRALL SW01（ROS2）

| 参数 | 文件 | 取值 |
|---|---|---|
| `robot.length` × `robot.width` | `robots/sw01/neupan/planner.yaml` | 0.96 m × 0.55 m |
| `robot.max_speed`、`robot.max_acce` | `robots/sw01/neupan/planner.yaml` | `[1.0, 1.0]`、`[0.5, 0.5]` |
| `ref_speed` | `robots/sw01/neupan/planner.yaml` | 0.4 m/s |
| `receding` × `step_time` | `robots/sw01/neupan/planner.yaml` | 10 × 0.2 s |
| `ipath.arrive_threshold` | `robots/sw01/neupan/planner.yaml` | 0.25 m |
| `adjust`：`q_s`、`q_theta`、`p_w`、`eta`、`d_max`、`d_min` | `robots/sw01/neupan/planner.yaml` | 1.0、1.0、0.8、12.0、0.4、0.05 |
| `w_deadband` | `robots/sw01/neupan/config.yaml` | `enable: true`、`enter_rad: 0.03`、`exit_rad: 0.067` |
| `adaptive_d_max`、`smith_predictor`、`output_gain` | `robots/sw01/neupan/config.yaml` | 全部 `enable: false` |
| `stuck_escape.rotate_clearance_m` / `rotate_open_clearance_m` | `robots/sw01/neupan/config.yaml` | 0.85 / 0.57 |
| `stuck_escape.strafe_enable`、`corridor_guard_enable` | `robots/sw01/neupan/config.yaml` | 都是 `true` |
| `inflation_radius`、`line_of_sight_safe_distance` | `robots/sw01/robot.yaml`（`navigation.*`） | 0.35 m、0.35 m |
| `footprint.length`、`footprint.width`、`footprint.margin` | `robots/sw01/robot.yaml`（`geometry.footprint`） | 0.96 m、0.55 m、0.10 m |

### Dobot Lumo（ROS1，桥接层）

| 参数 | 文件 | 取值 |
|---|---|---|
| `max_linear_vel`、`max_linear_backward`、`max_angular_vel` | `dobot_control.yaml` | 0.30、0.20、0.50（m/s、m/s、rad/s） |
| `max_accel`、`max_yaw_accel` | `dobot_control.yaml` | 0.40 m/s²、1.00 rad/s² |
| `cmd_vel_timeout` | `dobot_control.yaml` | 0.40 s |
| `enable_strafe` | `dobot_control.yaml` | `false`（Lumo 不能侧移） |
| `fsm_prepare`、`fsm_standing`、`fsm_walk` | `dobot_control.yaml` | **占位值**：请用 `--getfsm` 读到的实际 ID 替换 |

## 3. 调参顺序

一次只改一组，并把结果记下来。

1. **先定外形。** 填入真实的 `length` 和 `width`（以及 `sliding_map` 里的 `footprint_length_m` / `footprint_width_m`）。几何错了，后面每一个症状都会跟着错。

> **换机器人尺寸：改长宽即可，不用重训 DUNE。**
> 把 `robot.length` / `robot.width` 改成新机器人的实测值就够了（同时更新 `sliding_map` 的
> `footprint_length_m` / `footprint_width_m`）。出厂默认 `pan.dune_use_analytic: true` 走的是
> **解析闭式解**，会按 `length × width` 自动适配任意矩形外形，**不需要重新训练 DUNE 模型**。
> 只有当你显式关掉 `dune_use_analytic`、退回神经网络版 DUNE 时，才需要用新尺寸重新训练。
2. **速度。** 降低 `ref_speed` 与 `max_speed`，直到机器人能贴着路径走而不摆动。
3. **净空。** 按你最窄的通道来设 `d_max` 和 `inflation_radius`。`d_max` 应当保持在走廊净空的一半或更低。
4. **跟踪。** 调 `q_s`（位置）与 `q_theta`（航向），再调 `p_w`（角速度阻尼）。
5. **到达与恢复。** 设 `ipath.arrive_threshold`，再设 `stuck_escape` 的净空值。
6. **求解器。** 保持 `solver: 'SCS'`、`nrmp_inference_only: true`、`nrmp_warm_start: true`。不要为了提速而放松求解精度 —— 那等于放弃安全余量。

### 旋转净空规则

矩形外形的外接圆半径是

`R_circ = sqrt(L² + W²) / 2`

把 `stuck_escape.rotate_open_clearance_m` 设为 `≥ R_circ + 0.02 m`，机器人旋转时扫过的圆才不会扫进障碍。

| 机器人 | `R_circ` | `rotate_open_clearance_m` 最小值 |
|---|---|---|
| Go2（0.70 × 0.35） | 0.39 m | 0.41 m |
| SW01（0.96 × 0.55） | 0.55 m | 0.57 m（当前值） |

## 4. 在线调参（ROS1）

NeuPAN 节点运行时会订阅下面这些话题，所以你可以不重启就试一个值：

| 话题 | 对应参数 |
|---|---|
| `/neupan/q_s`、`/neupan/q_theta` | 位置与航向跟踪权重 |
| `/neupan/p_u`、`/neupan/p_w`、`/neupan/p_dw` | 控制量与角速度惩罚 |
| `/neupan/eta`、`/neupan/d_max`、`/neupan/d_min` | 障碍净空项 |

```bash
rostopic pub -1 /neupan/q_theta std_msgs/Float32 "data: 2.0"
rostopic pub -1 /neupan/d_max   std_msgs/Float32 "data: 0.15"
```

调到满意后，把值写回 planner 的 YAML。在线改的值重启就没了。

ROS2 用的是同一套 NeuPAN 内核。依赖这些话题之前，先在你自己的构建上确认（`ros2 topic list | grep neupan`）。

## 5. 症状对照表

| 症状 | 可能原因 | 先改哪个 |
|---|---|---|
| 窄门口犹豫或死锁 | `d_max` 或 `inflation_radius` 相对缝隙太宽 | 降 `d_max`（如 0.40 → 0.20）、降 `eta`、提 `q_theta` |
| 直线走廊里左右摆头 | 角速度振荡 | SW01 上保持 `w_deadband` 开启；提 `p_w`（0.8 → 1.0）；**只有实测到延迟**才开 `smith_predictor` |
| 90° 转角后大幅过冲 | 位置跟踪太弱或视野太短 | 提 `q_s`；降 `ref_speed`；增大 `receding` 或 `step_time` |
| 在终点附近绕圈停不下来 | `arrive_threshold` 小于转弯半径 | 提 `ipath.arrive_threshold`（SW01 0.25 → 0.45 m），让 `atomic_move` 更早接管 |
| 脱困旋转时转进墙里 | 旋转净空低于外接圆半径 | `rotate_open_clearance_m ≥ R_circ + 0.02`；保持 `corridor_guard_enable: true` |
| 为已经消失的障碍刹车 | `sliding_map` 记忆残留，或打到自身外形 | 降 `cell_decay_sec`；把 `footprint_margin_m` 提 0.05 |
| 求解耗时长（动作一顿一顿） | 障碍点太多或视野太长 | 降 `receding`；确认 `solver: 'SCS'` 与 `nrmp_warm_start: true` |
| 脱困时朝着楼梯倒车 | 机器人在楼梯上但地形门控没开 | 设置 `terrain_recognizer.backend` 并打开 `stair_suppress_enable: true`（见 §8） |

## 6. 各平台注意事项

- **Go2：** `w_deadband`、`adaptive_d_max`、`smith_predictor` 默认都是关的。要开就一个一个开。
- **SW01：** `output_gain` 是 SW01 专用补偿，其他机器人上保持关闭。
- **Dobot Lumo：** `/cmd_vel` 断流时，桥会在 `cmd_vel_timeout` 后下发零速，再经过 `cmd_vel_damp_timeout` 让机器人阻尼停下。**阻尼是终态**：需要发 `/dobot/reset` 才能恢复。除非机器人架在台架上、且有人守在急停旁边，否则不要用 `dry_run:=false` 运行。

## 7. 脱困策略槽（`local_planner.escape_policy`）

局部规划器卡住时由**谁来接管** `/cmd_vel`——不是 MPC 怎么算。两个键，都可选：

```yaml
local_planner:
  escape_policy:
    backend: stuck_escape              # stuck_escape | null | <name> | <dotted.module>
    config: neupan/escape_policy.yaml  # 可选；从 robots/<robot>/ 解析
```

- **不改行为就别写它。** `stuck_escape` 仍是默认值，仍然读 `neupan/config.yaml` 里的 `stuck_escape:` 块，所以之前调好的参数一个都不动。
- **`null`** 彻底关闭脱困（MPC 输出原样下发）。只在怀疑脱困逻辑干扰时用它做隔离——此时机器人卡住就是卡住。
- **`config`** 叠加在 `stuck_escape:` 块之上：文件里只覆盖它包含的键，所以现场配置文件可以只有两行。
- **调参顺序**：`trigger_frames`（等多久）→ `front_obs_threshold`（何时算被堵）→ `back_speed` / `rotate_speed`（动作多猛）→ `exit_obs_distance` / `duration_sec`（何时放弃）。推荐用 `criterion: progress` 配 `progress_threshold_m` 做卡住判据，`displacement` 是旧判据。
- **地形感知**：`stair_suppress_enable: true` 会让节点读取 `/stair_gait/status`，机器人处在楼梯上时压住脱困。前提是有人发布这个结论——见 §8。

启动时看日志确认接线：`[navigation_mode] mode=... escape=... policy=<backend>`。

## 8. 地形识别槽（`terrain_recognizer`）

顶层键，按需开启。不写就完全不启动地形节点。

```yaml
terrain_recognizer:
  backend: stair_gait                              # stair_gait | ramp_slope | dino_v3 | <custom>
  config: terrain_recognizer/stair_gait.yaml       # 可选，从 robots/<robot>/ 解析
```

一个后端只看一个区域，所以这种简写只适合“只关心正前方地面”的机器人。输出有三个：`/terrain_recognizer/status` 上的 latched 类型化结论、同一帧的 `/terrain_recognizer/scene`，以及精简后的旧版 JSON `/stair_gait/status`。

**同一帧里出现多个地形时**（前方楼梯、侧边坡道、中间平台），把 `backend` 换成实例文件——每个区域一个具名 `(backend, region)`。复制 `ros_ws/src/terrain_recognizer/config/instances.example.yaml` 修改：

```yaml
# robots/<robot>/robot.yaml
terrain_recognizer:
  backend: stair_gait                          # 省略 backend 的实例默认用它
  instances: terrain_recognizer/instances.yaml
```

```yaml
# instances.yaml —— 加载到节点的私有命名空间
instances:
  front:
    backend: stair_gait          # 可选；缺省用 terrain_recognizer.backend
    roi: {x_min: 0.6, x_max: 3.2, y_min: -0.6, y_max: 0.6}
  left:
    backend: stair_gait
    legacy: false                # 永不参与 /stair_gait/status 投票
    roi: {x_min: 0.6, x_max: 3.2, y_min: 0.4, y_max: 1.8}
```

- 每个实例发布 `/terrain_recognizer/<name>/status`；本帧所有有结论的实例一起出现在 `/terrain_recognizer/scene`（`terrain_recognizer/TerrainScene`）。多地形看的是这个话题；该模式下不再使用 `/terrain_recognizer/status`。
- `roi` 在 `base_link` 下先裁点云再交给后端，所以同一个后端的两个实例可以看不同方向的同一块地面。后端自己的 `roi_*` 仍会在这个裁切内生效，因此几何 ROI 至少要和实例区域一样宽。
- 后端**参数**是节点级的（只有一个 `min_steps`、一个 `detect_rate_hz`）。实例之间差的是区域，不是调参。
- `/stair_gait/status` 仍是单值，因为 NeuPAN 的楼梯门控只能接一个：只有 `legacy` 开启（默认）且区域与前方走廊（`legacy_corridor_half_width`，`0.6 m`；要求至少覆盖区域与走廊中较窄者的一半）真正重叠的实例参与投票，`stair_up/down` 优先于 `flat`，同优先级比 `confidence`。所以朝左/朝右的实例不可能把脱困按住。

`stair_gait` 几何参数（包内默认值在 `ros_ws/src/terrain_recognizer/config/stair_gait.yaml`）：

| 键 | 作用 | 配错时的症状 |
|---|---|---|
| `roi_x_min` / `roi_x_max` | 往前看多远的地面 | 楼梯发现太晚（x_min 过大）或把近处家具算进来（x_min 过小） |
| `roi_y_half` | 参与投票的横向走廊宽度 | 机器狗稍微走偏结论就翻转 |
| `roi_z_min` / `roi_z_max` | `base_link` 下接受的高度范围 | `z_min` 切掉下面几级踏步时会漏掉下楼梯 |
| `bin_m`、`min_bin_points` | 高度剖面的分辨率与支撑点数 | 远处稀疏点云上结论抖动或缺失 |
| `min_rise_m` / `max_rise_m` | 单级踏步必须落在此区间 | 薄踏步被忽略 / 墙面被当成台阶 |
| `min_steps` | 要求连续踏步数 | 单个马路牙子被当成楼梯 |
| `max_rise_spread_m` | 各级踏步之间的容差 | 台阶不平时结论变成 `unknown`（这是故意的） |

`ramp_slope` 几何参数（包内默认值在 `ros_ws/src/terrain_recognizer/config/ramp_slope.yaml`）—— 户外场景的另一半，针对连续斜坡而不是踏步（装卸平台坡度板、车库坡道、场地到院子 的高差过渡、两个铺装面之间的排水沟）。它用同一份高度剖面做直线拟合，从不说 `stair_*`：踏步过不了它的平面拟合。参数键带 `ramp_slope/` 前缀，所以两个后端可以是同一个节点的两个实例。

| 键 | 作用 | 配错时的症状 |
|---|---|---|
| `ramp_slope/x_min` / `x_max` | 测坡度的窗口 | 坡度发现太晚（x_min 过大）或把坡尾平地一起平均进来（x_max 过大，残差变大） |
| `ramp_slope/y_half` | 参与投票的横向走廊宽度 | 机器狗稍微走偏结论就翻转 |
| `ramp_slope/z_min` / `z_max` | `base_link` 下接受的高度范围 | `z_min` 切掉低端时会漏掉下坡；`z_max` 太小时上坡被截断 |
| `ramp_slope/bin_m`、`min_bin_points` | 剖面分辨率与支撑点数 | 远处稀疏点云上坡度抖动 |
| `ramp_slope/min_bins`、`min_span_m` | 判断所需的最小证据 | 只看 0.5 m 就报 `ramp_*`（把 `min_span_m` 调大） |
| `ramp_slope/max_rms_m` | “是不是一个平面”的残差门限 | 楼梯和马路牙子被判成斜坡（调小） |
| `ramp_slope/max_step_m` | 相邻 bin 允许的最大高度跳变 | 单个路缘或装卸边缘被读成“有点斜的平地”（调小） |
| `ramp_slope/min_slope_deg` | 低于此角度算 `flat` | 3% 的缓坡也报 `ramp_up`，把速度上限压低 |
| `ramp_slope/max_slope_deg` | 高于此角度结论是 `unknown` | 楼梯被当成坡道开上去（保持明显低于楼梯坡度，~25–30°） |

户外常态是两个一起跑 —— 前方楼梯、侧面坡道 —— 必须用实例，因为一个后端只能描述一个区域：

```yaml
instances:
  front: {backend: stair_gait, roi: {x_min: 0.6, x_max: 3.2, y_min: -0.6, y_max: 0.6}}
  ramp:  {backend: ramp_slope, legacy: false,
          roi: {x_min: 0.5, x_max: 3.0, y_min: -1.6, y_max: -0.4}}
```

`ramp_up` / `ramp_down` 故意把 `legacy_state` 报成 `flat`：坡道是可行驶、可倒退的地面，绝不能把脱困门控按住 —— 那个门控是为楼梯存在的。

`config/terrain_recognizer.yaml` 里的节点级键：`detect_rate_hz`（10 Hz 足够，检测器很便宜）、`confirm_frames`（镜子或玻璃门导致翻转时提到 5）、`publish_legacy`（保持 `true`，除非老的 `stair_gait` 节点还在跑、两边都会发布 `/stair_gait/status`）、`legacy_corridor_half_width`（能参与 `/stair_gait/status` 投票的前方走廊宽度）。

验证：

```bash
rostopic echo -n1 /terrain_recognizer/status     # 单实例模式
rostopic echo -n1 /terrain_recognizer/scene      # 所有实例，一条消息
rostopic echo -n1 /terrain_recognizer/front/status
rostopic echo -n1 /stair_gait/status             # NeuPAN 消费的旧版 JSON
```

`dino_v3` 目前是空壳：只加载自己的参数、打一条警告、不发布任何东西，所以选中它不会悄悄把错误类别喂给上层。

要接自己的模型，见 [04 · 增加自定义地形识别器](04_custom_terrain_backend.zh-CN.md)：后端契约、可编译骨架、构建与配置接线、离线测试。

