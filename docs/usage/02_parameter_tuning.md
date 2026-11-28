# 02 · Parameter Tuning

**English** | [中文](02_parameter_tuning.zh-CN.md)

This guide lists the parameters that matter for each supported platform, the order in which to change them, and a symptom table. Values below are the ones in the repository today.

> **How this guide relates to the [advanced tuning guide](../TUNING_GUIDE.md):**
> this page answers "**what values should my robot use**" — measured per-platform defaults plus
> a tuning order — while the tuning guide answers "**what does this parameter actually do, and
> how do I tune it**" — a full parameter dictionary plus symptom-based troubleshooting. Read
> that one before changing anything not listed here.

## 1. Where the parameters live

| Layer | Unitree Go2 (ROS1) | ASTRALL SW01 (ROS1 / ROS2) | Dobot Lumo (ROS1) |
|---|---|---|---|
| Robot profile (TF, footprint, A\*, scan slice) | `robots/go2/robot.yaml` | `robots/sw01/robot.yaml` | `robots/dobot_lumo/robot.yaml` |
| NeuPAN local planner | `robots/go2/neupan/config.yaml` + `planner.yaml` | `robots/sw01/neupan/config.yaml` + `planner.yaml` | `robots/dobot_lumo/neupan/config.yaml` + `planner.yaml` |
| Chassis bridge | `ros_ws/src/unitree_control` | `ros2_ws/src/sw01_ros_control` | `ros_ws/src/dobot_ros_control/launch/dobot_control_py.launch` |

NeuPAN profiles live in `robots/<robot>/neupan/` and are shared between ROS1 and ROS2. Override with `--neupan-config=PATH` if needed. **Never mix the Go2 and SW01 profiles.**

## 2. Platform reference values

### Unitree Go2 (ROS1)

| Parameter | File | Value |
|---|---|---|
| `robot.length` × `robot.width` | `planner.yaml` | 0.70 m × 0.35 m |
| `robot.max_speed`, `robot.max_acce` | `planner.yaml` | `[0.625, 0.875]`, `[0.625, 0.875]` |
| `ref_speed` | `planner.yaml` | 0.625 m/s |
| `receding` × `step_time` | `planner.yaml` | 10 × 0.1 s |
| `ipath.arrive_threshold` | `planner.yaml` | 0.45 m |
| `adjust`: `q_s`, `q_theta`, `p_w`, `eta`, `d_max`, `d_min` | `planner.yaml` | 1.0, 2.0, 0.8, 9.0, 0.10, 0.02 |
| `w_deadband`, `adaptive_d_max`, `smith_predictor` | `config.yaml` | all `enable: false` |
| `stuck_escape` | `config.yaml` | `enable: true` |

### ASTRALL SW01 (ROS2)

| Parameter | File | Value |
|---|---|---|
| `robot.length` × `robot.width` | `robots/sw01/neupan/planner.yaml` | 0.96 m × 0.55 m |
| `robot.max_speed`, `robot.max_acce` | `robots/sw01/neupan/planner.yaml` | `[1.0, 1.0]`, `[0.5, 0.5]` |
| `ref_speed` | `robots/sw01/neupan/planner.yaml` | 0.4 m/s |
| `receding` × `step_time` | `robots/sw01/neupan/planner.yaml` | 10 × 0.2 s |
| `ipath.arrive_threshold` | `robots/sw01/neupan/planner.yaml` | 0.25 m |
| `adjust`: `q_s`, `q_theta`, `p_w`, `eta`, `d_max`, `d_min` | `robots/sw01/neupan/planner.yaml` | 1.0, 1.0, 0.8, 12.0, 0.4, 0.05 |
| `w_deadband` | `robots/sw01/neupan/config.yaml` | `enable: true`, `enter_rad: 0.03`, `exit_rad: 0.067` |
| `adaptive_d_max`, `smith_predictor`, `output_gain` | `robots/sw01/neupan/config.yaml` | all `enable: false` |
| `stuck_escape.rotate_clearance_m` / `rotate_open_clearance_m` | `robots/sw01/neupan/config.yaml` | 0.85 / 0.57 |
| `stuck_escape.strafe_enable`, `corridor_guard_enable` | `robots/sw01/neupan/config.yaml` | both `true` |
| `inflation_radius`, `line_of_sight_safe_distance` | `robots/sw01/robot.yaml` (`navigation.*`) | 0.35 m, 0.35 m |
| `footprint.length`, `footprint.width`, `footprint.margin` | `robots/sw01/robot.yaml` (`geometry.footprint`) | 0.96 m, 0.55 m, 0.10 m |

### Dobot Lumo (ROS1, bridge)

| Parameter | File | Value |
|---|---|---|
| `max_linear_vel`, `max_linear_backward`, `max_angular_vel` | `dobot_control.yaml` | 0.30, 0.20, 0.50 (m/s, m/s, rad/s) |
| `max_accel`, `max_yaw_accel` | `dobot_control.yaml` | 0.40 m/s², 1.00 rad/s² |
| `cmd_vel_timeout` | `dobot_control.yaml` | 0.40 s |
| `enable_strafe` | `dobot_control.yaml` | `false` (Lumo cannot move sideways) |
| `fsm_prepare`, `fsm_standing`, `fsm_walk` | `dobot_control.yaml` | **placeholders**: replace with the IDs reported by `--getfsm` |

## 3. Tuning order

Change one group at a time, and log the result.

1. **Footprint first.** Enter the real `length` and `width` (and `footprint_length_m` / `footprint_width_m` in `sliding_map`). Wrong geometry causes every later symptom.

> **Changing the robot's size needs no DUNE retraining.**
> Set `robot.length` / `robot.width` to the measured values of the new chassis (and update
> `footprint_length_m` / `footprint_width_m` in `sliding_map`). The shipped default
> `pan.dune_use_analytic: true` uses the **analytic closed-form** solver, which adapts to any
> rectangular footprint from `length × width` — **no DUNE model has to be retrained**. Only if
> you explicitly disable `dune_use_analytic` and fall back to the neural DUNE encoder do you
> need to train again with the new dimensions.
2. **Speed.** Lower `ref_speed` and `max_speed` until the robot tracks the path without swaying.
3. **Clearance.** Set `d_max` and `inflation_radius` to fit your narrowest passage. `d_max` should stay at or below half the corridor clearance.
4. **Tracking.** Adjust `q_s` (position) and `q_theta` (heading), then `p_w` (angular damping).
5. **Arrival and recovery.** Set `ipath.arrive_threshold`, then the `stuck_escape` clearances.
6. **Solver.** Leave `solver: 'SCS'`, `nrmp_inference_only: true`, `nrmp_warm_start: true`. Do not loosen solver precision to gain speed: it removes the safety margin.

### Rotation clearance rule

For a rectangular footprint, the circumscribed radius is

`R_circ = sqrt(L² + W²) / 2`

Set `stuck_escape.rotate_open_clearance_m ≥ R_circ + 0.02 m` so the robot does not rotate into an obstacle inside its swing circle.

| Robot | `R_circ` | Minimum `rotate_open_clearance_m` |
|---|---|---|
| Go2 (0.70 × 0.35) | 0.39 m | 0.41 m |
| SW01 (0.96 × 0.55) | 0.55 m | 0.57 m (current value) |

## 4. Online tuning (ROS1)

The NeuPAN node subscribes to these topics while it runs, so you can test a value without a restart:

| Topic | Parameter |
|---|---|
| `/neupan/q_s`, `/neupan/q_theta` | position and heading tracking weights |
| `/neupan/p_u`, `/neupan/p_w`, `/neupan/p_dw` | control and angular-velocity penalties |
| `/neupan/eta`, `/neupan/d_max`, `/neupan/d_min` | obstacle clearance terms |

```bash
rostopic pub -1 /neupan/q_theta std_msgs/Float32 "data: 2.0"
rostopic pub -1 /neupan/d_max   std_msgs/Float32 "data: 0.15"
```

Write the value you like back into the planner YAML. Online changes are lost on restart.

ROS2 uses the same NeuPAN core. Verify the topics on your build (`ros2 topic list | grep neupan`) before relying on them.

## 5. Symptom table

| Symptom | Likely cause | Change first |
|---|---|---|
| Hesitates or deadlocks at a narrow door | `d_max` or `inflation_radius` too wide for the gap | Lower `d_max` (e.g. 0.40 → 0.20), lower `eta`, raise `q_theta` |
| Sways left and right on a straight corridor | Angular oscillation | Keep `w_deadband` on (SW01); raise `p_w` (0.8 → 1.0); enable `smith_predictor` only if the lag is measured |
| Large overshoot after a 90° corner | Position tracking too weak or horizon too short | Raise `q_s`; lower `ref_speed`; increase `receding` or `step_time` |
| Circles near the final waypoint and never finishes | `arrive_threshold` smaller than the turning radius | Raise `ipath.arrive_threshold` (SW01 0.25 → 0.45 m) so `atomic_move` takes over earlier |
| Rotates into a wall during escape | Rotation clearance below the circumscribed radius | `rotate_open_clearance_m ≥ R_circ + 0.02`; keep `corridor_guard_enable: true` |
| Brakes for obstacles that are gone | Stale `sliding_map` memory or self-footprint hits | Lower `cell_decay_sec`; raise `footprint_margin_m` by 0.05 |
| Solve time is high (jerky motion) | Too many obstacle points or receding too long | Lower `receding`; check `solver: 'SCS'` and `nrmp_warm_start: true` |
| Escape reverses toward a staircase | Terrain gate off while the robot is on stairs | Set `terrain_recognizer.backend` and `stair_suppress_enable: true` (see §8) |

## 6. Platform-specific cautions

- **Go2:** `w_deadband`, `adaptive_d_max`, and `smith_predictor` are off by default. Enable them one at a time.
- **SW01:** `output_gain` is a SW01-only compensation. Keep it off on other robots.
- **Dobot Lumo:** when `/cmd_vel` goes stale, the bridge sends zero velocity after `cmd_vel_timeout` and damps the robot after a further `cmd_vel_damp_timeout`. Damping is terminal: send `/dobot/reset` to recover. Do not run with `dry_run:=false` unless the robot is on a gantry with an operator at the emergency stop.

## 7. Escape-policy slot (`local_planner.escape_policy`)

Which maneuver takes over `/cmd_vel` when the local planner stalls — not how the MPC works. Two keys, both optional:

```yaml
local_planner:
  escape_policy:
    backend: stuck_escape              # stuck_escape | null | <name> | <dotted.module>
    config: neupan/escape_policy.yaml  # optional; resolved from robots/<robot>/
```

- **Leave it out unless you are changing behavior.** `stuck_escape` stays the default and keeps reading the `stuck_escape:` block of `neupan/config.yaml`, so nothing you tuned before moves.
- **`null`** disables escape entirely (MPC output untouched). Use it only to isolate a suspected escape interaction, since a stalled robot then simply stays stalled.
- **`config`** layers over the `stuck_escape:` block: the file overrides only the keys it contains, so a site file can be two lines long.
- **Tuning order**: `trigger_frames` (how long it waits) → `front_obs_threshold` (when it considers itself blocked) → `back_speed` / `rotate_speed` (how violent the maneuver is) → `exit_obs_distance` / `duration_sec` (when it gives up). `criterion: progress` with `progress_threshold_m` is the recommended stall test; `displacement` is the legacy one.
- **Terrain awareness**: `stair_suppress_enable: true` makes the node read `/stair_gait/status` and hold the escape while the robot is on stairs. It only works if something publishes that verdict — see §8.

Check the wiring at startup: `[navigation_mode] mode=... escape=... policy=<backend>` in the neupan log.

## 8. Terrain-recognizer slot (`terrain_recognizer`)

Top-level key, opt-in. Absent means no terrain node is launched at all.

```yaml
terrain_recognizer:
  backend: stair_gait                              # stair_gait | ramp_slope | dino_v3 | <custom>
  config: terrain_recognizer/stair_gait.yaml       # optional, resolved from robots/<robot>/
```

One backend describes one region, so this shorthand is for robots that only care about the ground straight ahead. Three outputs are published: the latched typed verdict on `/terrain_recognizer/status`, the same frame on `/terrain_recognizer/scene`, and the reduced legacy JSON on `/stair_gait/status`.

**When several terrains are in view at once** (stairs ahead, a ramp to the side, a landing between them), replace `backend` with an instances file — one named `(backend, region)` pair per area. Copy `ros_ws/src/terrain_recognizer/config/instances.example.yaml`:

```yaml
# robots/<robot>/robot.yaml
terrain_recognizer:
  backend: stair_gait                          # default for instances that omit one
  instances: terrain_recognizer/instances.yaml
```

```yaml
# instances.yaml - loaded into the node's private namespace
instances:
  front:
    backend: stair_gait          # optional; defaults to terrain_recognizer.backend
    roi: {x_min: 0.6, x_max: 3.2, y_min: -0.6, y_max: 0.6}
  left:
    backend: stair_gait
    legacy: false                # never votes on /stair_gait/status
    roi: {x_min: 0.6, x_max: 3.2, y_min: 0.4, y_max: 1.8}
```

- Each instance publishes `/terrain_recognizer/<name>/status`, and every instance with a verdict appears together on `/terrain_recognizer/scene` (`terrain_recognizer/TerrainScene`). That is the multi-terrain view; `/terrain_recognizer/status` is not used in this mode.
- `roi` crops the cloud in `base_link` before the backend runs, so two instances of the same backend can look at different ground. The backend's own `roi_*` keys still apply *inside* the crop, so keep the geometry ROI at least as wide as the instance region.
- Backend **parameters** stay node-global (one `min_steps`, one `detect_rate_hz`). Instances differ by region, not tuning.
- `/stair_gait/status` stays a single verdict because NeuPAN's escape gate can only act on one: only instances with `legacy` enabled (default) whose region really overlaps the forward corridor (`legacy_corridor_half_width`, `0.6 m`; at least half of the narrower of region and corridor) vote, `stair_up/down` outranks `flat`, ties break on confidence. A left- or right-facing instance therefore cannot hold the escape.

`stair_gait` geometry knobs (package defaults in `ros_ws/src/terrain_recognizer/config/stair_gait.yaml`):

| Key | Effect | Symptom when wrong |
|---|---|---|
| `roi_x_min` / `roi_x_max` | How far ahead the ground is inspected | Stairs detected too late (x_min too large) or mixed with nearby furniture (x_min too small) |
| `roi_y_half` | Lateral corridor that votes | Verdict flips when the dog walks slightly off-center |
| `roi_z_min` / `roi_z_max` | Heights accepted in `base_link` | Down-stairs missed when `z_min` cuts the lower treads |
| `bin_m`, `min_bin_points` | Profile resolution and support | Noisy or missing verdict on sparse far returns |
| `min_rise_m` / `max_rise_m` | One tread must fall inside this band | Thin treads ignored / walls counted as steps |
| `min_steps` | Consecutive treads required | Single curbs classified as staircases |
| `max_rise_spread_m` | Tolerance between treads | Verdict becomes `unknown` on uneven stairs (deliberate) |

`ramp_slope` geometry knobs (package defaults in `ros_ws/src/terrain_recognizer/config/ramp_slope.yaml`) — the outdoor counterpart, for continuous slopes instead of treads (dock levellers, garage ramps, apron-to-yard transitions, drainage dips). It is the same height profile with a straight-line fit, and it never claims `stair_*`: treads fail the plane fit. Keys are prefixed `ramp_slope/` so both backends can run as instances of one node.

| Key | Effect | Symptom when wrong |
|---|---|---|
| `ramp_slope/x_min` / `x_max` | Window the slope is measured over | Slope read too late (x_min too large) or averaged over the ramp's end (x_max too large, rms grows) |
| `ramp_slope/y_half` | Lateral corridor that votes | Verdict flips when the dog rolls slightly off-centre |
| `ramp_slope/z_min` / `z_max` | Heights accepted in `base_link` | Down-ramps missed when `z_min` cuts the lower end; up-ramps cut when `z_max` is too small |
| `ramp_slope/bin_m`, `min_bin_points` | Profile resolution and support | Noisy slope on sparse far returns |
| `ramp_slope/min_bins`, `min_span_m` | Evidence needed before judging | Reports `ramp_*` from a 0.5 m glimpse (raise `min_span_m`) |
| `ramp_slope/max_rms_m` | "Is it one plane?" residual gate | Stairs and kerbs start being labelled as ramps (lower it) |
| `ramp_slope/max_step_m` | Largest jump between neighbouring bins | A single kerb or dock edge is read as a slightly tilted "flat" (lower it) |
| `ramp_slope/min_slope_deg` | Below this the ground is `flat` | A gentle 3 % apron reports `ramp_up` and drops the speed cap |
| `ramp_slope/max_slope_deg` | Above this the verdict is `unknown` | A staircase is driven as a ramp (keep it well under stair pitch, ~25–30°) |

Running both at once is the normal outdoor setup — stairs ahead, a ramp to the side — and needs instances, because one backend only describes one region:

```yaml
instances:
  front: {backend: stair_gait, roi: {x_min: 0.6, x_max: 3.2, y_min: -0.6, y_max: 0.6}}
  ramp:  {backend: ramp_slope, legacy: false,
          roi: {x_min: 0.5, x_max: 3.0, y_min: -1.6, y_max: -0.4}}
```

`ramp_up` / `ramp_down` report `legacy_state: flat` on purpose: a ramp is drivable and reversible, so it must never hold the escape gate — that gate exists for stairs.

Node-level keys in `config/terrain_recognizer.yaml`: `detect_rate_hz` (10 Hz is enough; the detector is cheap), `confirm_frames` (raise to 5 if a mirror or glass door causes flicker), `publish_legacy` (keep `true` unless the legacy `stair_gait` node still runs and both would publish `/stair_gait/status`), `legacy_corridor_half_width` (how wide the forward corridor is that may vote on `/stair_gait/status`).

Verify:

```bash
rostopic echo -n1 /terrain_recognizer/status     # single-instance mode
rostopic echo -n1 /terrain_recognizer/scene      # all instances, one message
rostopic echo -n1 /terrain_recognizer/front/status
rostopic echo -n1 /stair_gait/status             # legacy JSON consumed by NeuPAN
```

`dino_v3` is a stub today: it loads its parameters, logs one warning and publishes nothing, so selecting it cannot silently feed a wrong class to the stack.

Ready to bring your own model? [04 · Add a Custom Terrain Recognizer](04_custom_terrain_backend.md) covers the backend contract, a compiling skeleton, build/config wiring and offline tests.
