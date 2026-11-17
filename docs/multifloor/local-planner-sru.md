# Choosing the local planner: NeuPAN / SRU

This note records how far "the local planner is selectable" actually goes, what is still
missing, and what it means for the **four-floor, 3D A\*** line we are building. Short version:

- **The switch exists**: `--local-planner=neupan|sru` is threaded from the launch scripts down
  to `mission_manager`.
- **The SRU side is not in this repository**: `ros_ws/src` ships `neupan_ros` only, there is no
  `sru_nav_go2_ros1` package, and the ONNX policy plus VAE encoder must be trained for your own
  chassis in [ManifoldTechLtd/SRU-Odin](https://github.com/ManifoldTechLtd/SRU-Odin) and deployed
  into an external workspace. So: **selectable, but selecting it does nothing yet** — the upstream
  README says the navigation integration of SRU is "coming soon".
- Architecturally it is compatible (both end at `/cmd_vel`), but **SRU turns global planning off**,
  which means atomic floor switching, `lift`/`stairs`/`ramp` transitions and the 3D A\* path
  **do not work in SRU mode**.

## 1. What is already wired

| Piece | Where | Notes |
|---|---|---|
| Launch entry | `scripts/ons2_go2.sh`, `scripts/ons2_sw01.sh`, `scripts/ons2_dobot_lumo.sh` | `--local-planner=sru`, plus optional `--sru-policy=FILE` |
| Parameter plumbing | same scripts: `rosparam set /local_planner`, `roslaunch ... local_planner:=${LOCAL_PLANNER}` | passed to both `ons2_bringup` and `mission_manager` |
| Mode selection | `ros_ws/src/mission_manager/src/mission_manager_node.cpp:247-256` | `local_planner_mode` defaults from `local_planner`: `sru`/`goal_pose` → `goal_pose`, otherwise `action` (NeuPAN via the `PlanPath` action) |
| Topics / packages | `robots/go2/robot.yaml:86-95` | `goal_topic=/sru/goal_pose`, `cmd_vel_topic=/sru/cmd_vel`, `enable_topic=/sru/cmd_vel_enable`, `package=sru_nav_go2_ros1`, `launch=sru_nav_go2.launch` |
| Velocity gate | `ros_ws/scripts/sru/sru_cmd_vel_relay.py`, `ros_ws/scripts/sru/sru_nav_wiring.launch` | SRU's 5 Hz `/sru/cmd_vel` reaches `/cmd_vel` only while `/sru/cmd_vel_enable` is true |
| Shutdown | `scripts/ons2_go2_stop.sh` | already lists `sru_nav_node` and `sru_cmd_vel_relay` |
| Waypoint contract | see above | goals are sent in odom; arrival is judged by `sru_arrive_threshold_m` (0.7 m default) |

## 2. What is still missing

1. **The `sru_nav_go2_ros1` package.** Not under `ros_ws/src`; it lives in an external workspace
   pulled in via `export SRU_WS=/path/to/sru_ros1_workspace` (ROS 2 uses `SRU_WS_ROS2`, models
   default to `$SRU_WS_ROS2/Deployment/models`, overridable with `SRU_MODELS_DIR`).
2. **Trained ONNX weights** (policy + VAE encoder). Explicitly not shipped; they have to be trained
   against your robot's locomotion. Without them the node runs but produces nothing useful.
3. **Robot locomotion support**. SRU emits a body-frame `Twist`; the vendor locomotion must accept it,
   and on Go2 nothing else may also publish `cmd_vel` (NeuPAN, `stair_gait`, `unitree_control`).
4. **Per-robot package mapping**: `SRU_PKG`, `SRU_LAUNCH`, or `planner.local` in
   `robots/<robot>/robot.yaml`.

## 3. Relation to the multi-floor line (important)

| Capability | NeuPAN (default) | SRU (experimental) |
|---|---|---|
| Global path | yes (`map_planner` 3D A\*; `octo_planner_ros1` sits behind the same `PlanPath` contract) | **none**, waypoints go straight out as 2D goals |
| Obstacle input | `/scan_360` point set from `sliding_map` | Odin1 depth image |
| Route segments / periodic replanning / recovery / hot path | on | all off (per upstream README) |
| Cross-floor (`floor_manager` atomic switch, floor field, `lift`/`stairs`/`ramp`) | available | **not available**: no global path, so no segmentation and no transition ordering |
| Stairs | 3D A\* walks the ramp-semantics treads; `stair_gait` handles the gait handoff | only the `stair_gait` part; SRU has no floor or stair semantics |
| `--navigation-mode` | effective | ignored |

Current engineering stance:

- **Same-floor patrol**: SRU can replace NeuPAN; the safety gate and the `/cmd_vel` interface are shared.
- **Cross-floor patrol**: stay on "3D A\* global + NeuPAN on flats and ramps + `stair_gait` on flights".
  In SRU mode `mission_manager` switches to `goal_pose`, and the floor-switch sequence has no global
  path to work with, so it is not recommended there.

## 4. Integration steps once the code is released

1. Build the ROS 1 (or ROS 2) workspace from SRU-Odin `Deployment/`, `source` it, and confirm the
   `sru_nav_node` executable exists.
2. Drop the exported `policy.onnx` and `vae.onnx` into `$SRU_WS/Deployment/models` (or `SRU_MODELS_DIR`).
3. `export SRU_WS=...`, then
   `bash scripts/ons2_go2.sh --local-planner=sru --map=NAME [--sru-policy=FILE]`.
4. Validate on same-floor short waypoints first; only then consider routing a segment through SRU.

## 5. Acceptance checklist (tick these, do not eyeball)

- [ ] `/sru/goal_pose` receives the goal from `mission_manager`;
- [ ] `/sru/cmd_vel` runs at a steady 5 Hz and reaches `/cmd_vel` only while `/sru/cmd_vel_enable` is true;
- [ ] with SRU stopped, no stale publisher remains on `/cmd_vel` (NeuPAN and `stair_gait` must be down);
- [ ] `mission_manager` logs `local_planner_mode=goal_pose`, and segmentation / periodic replanning really do not run;
- [ ] arrival fires on `sru_arrive_threshold_m` (0.7 m default), not on completing a global path;
- [ ] the safety gate (`ons2_safety_gate_ros1.py`) still applies in SRU mode;
- [ ] with a placed obstacle (column / pedestrian) it stops or goes around instead of driving through.

## 6. State of the simulator page

The legend row of `docs/multifloor/sim/mesh_nav.html` now carries a `局部规划` selector:

- `NeuPAN`: selectable, and it is what the page draws - global bright-blue line plus the yellow
  local constraint line (5 cm point-set hard threshold + 2 s rollout);
- `SRU（未开放）`: **disabled placeholder**. Choosing it only prints through `say()` that SRU needs an
  external workspace plus a trained ONNX policy and VAE, and that it bypasses the global path the
  page draws - so the page cannot demonstrate it.

`window.__MESH` also exposes `localPlanner` (current backend) and `sruReady` (hard `false` until the
integration lands).

> SRU is an end-to-end policy: the 3D A\* path, waypoints and floor transitions on this page do not
> apply to it. A real SRU demo needs its own page that draws the depth-image/goal → cmd_vel loop.
