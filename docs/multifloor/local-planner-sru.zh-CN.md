# 局部规划器切换：NeuPAN / SRU

本文记录 ONS2 里"局部规划器可选"这件事到底接了多少、还缺什么，以及它和我们这条
**4 层跨层 + 3D A\*** 主线的关系。结论先行：

- **开关已经接好**，`--local-planner=neupan|sru` 从启动脚本一路传到 `mission_manager`。
- **SRU 那一侧的节点与模型不在本仓库**：`ros_ws/src` 里只有 `neupan_ros`，没有
  `sru_nav_go2_ros1`；ONNX 策略与 VAE 编码器需要按自己底盘在
  [ManifoldTechLtd/SRU-Odin](https://github.com/ManifoldTechLtd/SRU-Odin) 训练后放进外部工作空间。
  也就是说：**能选，但选了跑不起来**，官方 README 第 3 节也写明"融入导航的 SRU 将在近期发布"。
- 架构上是兼容的（两者都收敛到 `/cmd_vel`），但 **SRU 模式关掉全局规划**，因此
  `floor_manager` 的原子切层、`lift`/`stairs`/`ramp` 过渡、3D A\* 全局路径在 SRU 模式下**全部不生效**。

## 1. 仓库里已经有的东西

| 环节 | 位置 | 说明 |
|---|---|---|
| 启动入口 | `scripts/ons2_go2.sh`、`scripts/ons2_sw01.sh`、`scripts/ons2_dobot_lumo.sh` | `--local-planner=sru`，另有 `--sru-policy=FILE` |
| 参数下发 | 同上，`rosparam set /local_planner` + `roslaunch ... local_planner:=${LOCAL_PLANNER}` | 同时传给 `ons2_bringup` 和 `mission_manager` |
| 模式判定 | `ros_ws/src/mission_manager/src/mission_manager_node.cpp:247-256` | `local_planner_mode` 缺省时自动推导：`sru`/`goal_pose` → `goal_pose`，其它 → `action`（NeuPAN 走 `PlanPath` action） |
| 目标/速度通道 | `robots/go2/robot.yaml:86-95` | `goal_topic=/sru/goal_pose`、`cmd_vel_topic=/sru/cmd_vel`、`enable_topic=/sru/cmd_vel_enable`、`package=sru_nav_go2_ros1`、`launch=sru_nav_go2.launch` |
| 速度门控 | `ros_ws/scripts/sru/sru_cmd_vel_relay.py`、`ros_ws/scripts/sru/sru_nav_wiring.launch` | SRU 的 5 Hz `/sru/cmd_vel` 仅当 `/sru/cmd_vel_enable` 为真才转发到 `/cmd_vel` |
| 关停 | `scripts/ons2_go2_stop.sh` | 已带上 `sru_nav_node`、`sru_cmd_vel_relay` |
| 话题约定 | 见上表 | 航点按 odom 坐标系下发；到达判定用 `nav_params.yaml` 的 `sru_arrive_threshold_m`（默认 0.7 m） |

## 2. 还缺的东西

1. **`sru_nav_go2_ros1` 这个包**。`ros_ws/src` 下没有它；它属于外部工作空间，
   通过 `export SRU_WS=/path/to/sru_ros1_workspace` 引入（ROS 2 走 `SRU_WS_ROS2`，
   模型默认从 `$SRU_WS_ROS2/Deployment/models` 读取，可用 `SRU_MODELS_DIR` 覆盖）。
2. **训练好的 ONNX**（策略 + VAE 编码器）。README 明确写"不包含在本仓库中"，必须针对
   Go2 的运控单独训练；没有模型时节点即使起来了也不会产生有用的速度指令。
3. **机器人运控**。SRU 输出机体坐标系 `Twist`，需要厂家给的运控配合，Go2 侧还要保证
   `cmd_vel` 不被其它节点（NeuPAN、`stair_gait`、`unitree_control`）重复发布。
4. **机型/自定义包的映射**。`SRU_PKG`、`SRU_LAUNCH`，或在 `robots/<robot>/robot.yaml`
   的 `planner.local` 里指到自己的包与 launch。

## 3. 和我们跨层主线的关系（重要）

| 能力 | NeuPAN（默认） | SRU（实验性） |
|---|---|---|
| 全局路径 | 有（`map_planner` 3D A\*，`octo_planner_ros1` 挂在同一 `PlanPath` 契约） | **没有**，航点直接当 2D 目标 |
| 障碍输入 | `sliding_map` 的 `/scan_360` 点集 | Odin1 深度图像 |
| 路线分段 / 周期重规划 / 恢复 / 热路径 | 开 | 全关（README 明示） |
| 跨层（`floor_manager` 原子切层、floor 字段、`lift`/`stairs`/`ramp` 过渡） | 可用 | **不可用**：没有全局路径就没有分段与过渡点排序 |
| 楼梯 | 3D A\* 走斜面语义的梯面，`stair_gait` 负责步态交接 | 只有 `stair_gait` 那一段，SRU 本身没有楼层/楼梯语义 |
| `--navigation-mode` | 有效 | 无效 |

因此当前工程口径是：

- **同层巡检**：SRU 可以替 NeuPAN，安全门控与 `/cmd_vel` 接口共用，风险可控。
- **跨层巡检**：仍走"3D A\* 全局 + NeuPAN 平面/坡段 + `stair_gait` 梯段步态"。
  在 SRU 模式下，`mission_manager` 会把目标改成 `goal_pose`，切层流程没有全局路径可用，
  不建议在跨层任务里选它。

## 4. 官方放代码后的接入步骤

1. 按 SRU-Odin 的 `Deployment/` 说明构建 ROS 1（或 ROS 2）工作空间，`source` 后确认
   `sru_nav_node` 可执行文件存在。
2. 把训练导出的 `policy.onnx` 与 `vae.onnx` 放进 `$SRU_WS/Deployment/models`（或 `SRU_MODELS_DIR`）。
3. `export SRU_WS=...`，然后
   `bash scripts/ons2_go2.sh --local-planner=sru --map=NAME [--sru-policy=FILE]`。
4. 先只跑同层短航点，确认链路后再考虑把某一段任务切到 SRU。

## 5. 验收清单（照着勾，别凭感觉）

- [ ] `/sru/goal_pose` 能收到 `mission_manager` 下发的目标；
- [ ] `/sru/cmd_vel` 稳定 5 Hz，且只在 `/sru/cmd_vel_enable` 为真时出现在 `/cmd_vel`；
- [ ] 关掉 SRU 后 `/cmd_vel` 没有任何残留发布者（NeuPAN、`stair_gait` 必须已停）；
- [ ] `mission_manager` 日志里 `local_planner_mode=goal_pose`，路线分段/周期重规划确实没跑；
- [ ] 到达判定按 `sru_arrive_threshold_m`（默认 0.7 m）触发，不是按全局路径走完；
- [ ] 急停（`ons2_safety_gate_ros1.py` 的门控）在 SRU 模式下同样有效；
- [ ] 一次人为障碍（立柱/行人）下能停住或绕开，而不是直冲。

## 6. 仿真页现状

`docs/multifloor/sim/mesh_nav.html` 的图例行加了 `局部规划` 选择器：

- `NeuPAN`：可选项，页面画的就是它——全局亮蓝线 + 黄色局部约束线（点集 5 cm 硬阈值 + 2 s rollout）；
- `SRU（未开放）`：**disabled 占位**。选中时只会在 `say()` 里说明"需要外部 SRU 工作空间与
  ONNX 策略 + VAE，且它不走全局路径，本页无法演示"，不会假装切换成功。

调试句柄 `window.__MESH` 上另有 `localPlanner`（当前后端）与 `sruReady`（恒 `false`，
等接入后改成真实探测）。

> 注意：SRU 是端到端策略，页面里的 3D A\* 全局路径、途经点、楼层过渡对它都不适用。
> 真要演示 SRU，需要单独一页把深度图/目标→cmd_vel 的回路画出来，而不是复用这一页。
