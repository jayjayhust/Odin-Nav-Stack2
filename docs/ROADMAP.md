# ONS2 Roadmap — Pluggable Terrain & Escape Slots

**English** | [中文](ROADMAP.zh-CN.md)

This page tracks the two extension points added on top of the four pluggable slots in
[ONS2_ARCHITECTURE §2](architecture/ONS2_ARCHITECTURE.md): the **terrain-recognizer slot**
(`terrain_recognizer.backend`) and the **escape-policy slot**
(`local_planner.escape_policy.backend`).

Both slots are **opt-in**: a `robot.yaml` that never mentions them keeps `stuck_escape` as the
escape policy and launches no terrain node. §1–§2 describe what is in the tree today, §3 lists
next milestones in increasing scope, and §4 fixes the compatibility rules that must not regress.

---

## 1. Status at a glance

Terms used below:

- **In tree** — code exists in the working tree; opt-in, default behaviour unchanged.
- **Sim only** — implemented in the mesh simulator, not on the robot.
- **Stub** — interface and parameters fixed, no verdict emitted.
- **Design only** — described in §3, no code yet.

| Area | State | Gate / evidence | Next step |
|---|---|---|---|
| Terrain slot, ROS 1 | **In tree** — `terrain_recognizer` node and backend factory, `stair_gait` and `ramp_slope` backends, multi-instance `TerrainScene`, legacy `/stair_gait/status` republish | `multifloor/sim/test_recognizers.js`; first `catkin_make` on the Jetson still open | consumer wiring (§3.1.2), per-instance params (§3.1.3) |
| Terrain slot, ROS 2 | **Design only** | — | §3.1.1 |
| `dino_v3` backend | **Stub** — loads parameters, publishes nothing; no shipped profile selects it | abstains with `valid=false` | §3.1.4 |
| Escape slot | **In tree** — `NeuPAN/neupan/escape_policies/` registry, `stuck_escape`, `null`, compatibility shim; terrain suppression from `/stair_gait/status` is ROS 1 only | `test_escape_policy_slot.py`; fixture-driven `test_escape_route_gate.cpp` (first build pending) | trace parity (§3.2.3) |
| SimReady import, 3DGS prior, world model, LingBot-Navi adapters | **Design only** | sim probes must return identical verdicts when the new slots are unconfigured | §3.3–§3.6 |

**Validation status.** The escape parity gate is designed to fail on divergence: one fixture,
`multifloor/sim/escape_contract.fixture.json`, is sliced into the JS probe and compiled into the
C++ test. Neither ROS workspace has been built on the development host, and this repository has no
CI that builds or runs them; the first `catkin_make` on the target Jetson is the open build gate.
Hardware behaviour (floor switching, stair test, gait handover) is tracked in the multi-floor
documents, and gait actuation stays off by default.

---

## 2. What is in the tree today

| Slot | Selected by | Reference implementation | Where it runs |
|---|---|---|---|
| Terrain recognizer | `terrain_recognizer.backend` | `stair_gait` (geometric height profile), `ramp_slope` (geometric plane fit) | ROS 1 only — `terrain_recognizer` node, included from `go2.launch` |
| Terrain recognizer, multi-terrain | `terrain_recognizer.instances` | named `(backend, region)` instances of the same backends | ROS 1 only — one node, `/terrain_recognizer/<name>/status` + `/terrain_recognizer/scene` |
| Terrain recognizer (learned) | same keys | `dino_v3` — parameter-loading stub, publishes nothing | ROS 1 only, never selected by a shipped profile |
| Escape policy | `local_planner.escape_policy.backend` | `stuck_escape` (default), `null` | ROS 1 and ROS 2 — inside the NeuPAN node |

### Terrain: package split

- `ros_ws/src/stair_gait` builds `libstair_detector.so` from geometry code with **no**
  `unitree_sdk2` dependency, plus the optional legacy `stair_gait_node` (actuation) when the SDK is
  found. The SDK is found `QUIET`, so a machine without it still builds the library.
- `ros_ws/src/terrain_recognizer` owns the slot: backend factory, the
  `TerrainObservation` / `TerrainScene` messages, per-instance `/terrain_recognizer/<name>/status`
  topics, the `/terrain_recognizer/scene` aggregate, and the legacy JSON republish on
  `/stair_gait/status`. The republish is reduced from forward-corridor instances only, because that
  is the topic NeuPAN's `stair_suppress_enable` gate reads.

### Escape: module move

- `NeuPAN/neupan/escape_policies/` holds the contract (`EscapePolicy`, `EscapeDecision`,
  `as_escape_policy`), the registry (`register_escape_policy` / `create_escape_policy`), the
  reference `stuck_escape`, and `null`.
- `NeuPAN/neupan/util/stuck_escape.py` is a deprecated shim, so the simulation benchmark and the
  existing unit tests import unchanged.

---

## 3. Next milestones

### 3.1 Terrain recognizer

1. **ROS 2 parity.** The node is ROS 1 today because the reference backend consumes
   `/odin1/cloud_slam` directly. A ROS 2 port needs the same factory surface (the header is
   ROS-agnostic apart from `ros::NodeHandle` in `configure()`), and then a `terrain_recognizer`
   entry in `bringup.launch.py`.
2. **Consumer wiring.** `mission_manager` and `floor_manager` do not subscribe yet. The first
   consumer should be a *soft* gate — annotate the active path segment, do not block it — so a
   misclassification cannot strand the robot. `TerrainScene` is the message for this: it already
   carries every simultaneous terrain with its region, which is exactly what "take this stair
   flight, avoid that ramp" needs.
3. **Per-instance parameters.** Backend parameters are node-global today (`min_steps`, ROI
   thresholds), so multi-instance sites differ by region only. Move them to a per-instance
   namespace (`~instances/<name>/params`) so two stair instances can use different `min_steps`.
4. **DINOv3 prototype.** Fill in `dino_v3_backend.cpp` (walkthrough: [04 · Add a Custom Terrain
   Recognizer](usage/04_custom_terrain_backend.md)): subscribe `image_topic`, crop with
   `input_size`, classify into the fixed `type` vocabulary, and scale the logits into
   `confidence`. Acceptance gate: publish `valid=false` rather than guess whenever the model is
   unavailable, exactly like the stub does today. For several regions, run **one** forward pass and
   emit per-ROI crops rather than one pass per instance — instances multiply CPU, and the
   compute-board budget in the platform SPEC does not survive N inferences.
5. **Learned-backend guardrails.** A learned backend must keep the same `type` vocabulary, always
   publish `geometry` (the region it judged), and must never publish `stair_up` / `stair_down`
   with `confidence` below the configured threshold — the escape slot suppresses itself on those
   verdicts, so a false positive strands the robot at the foot of the stairs.

### 3.2 Escape policy

1. **`dino_escape` / learned policies.** The registry already accepts a dotted module exposing
   `create_policy(cfg)`, so a policy can live outside this repo. Such a policy must pass the same
   acceptance suite as the reference one (`NeuPAN/example/sw01_bench/test_escape_policy_slot.py`),
   including the node attribute surface that `neupan_ros.py` reads directly.
2. **Per-robot `config` files.** The slot layered `config` over the legacy `stuck_escape:` block
   so a site file only overrides what it changes. Once profiles use it, the corresponding keys
   should migrate out of `robots/<robot>/neupan/config.yaml`.
3. **Trace parity.** Any replacement must keep emitting the `/neupan/control_trace` and
   `/neupan/replan` fields the operator console reads: `escape_active`, `escape_count`,
   `escape_direction`.

### 3.3 Simulation assets — import NVIDIA SimReady Central

1. **Offline conversion, not a runtime loader.** [sim/](multifloor/sim/README.md) accepts
   `.obj`/`.stl` through the file picker (unit and up-axis are user-selectable) and `build.py`
   inlines the demo meshes, so the built page stays self-contained. SimReady Central ships
   USD/USDZ — PBR materials, colliders and rigid-body physics that neither the picker nor three.js
   can read. The converter therefore runs offline and emits glTF/OBJ plus a per-asset sidecar
   (units, up-axis, collider boxes, material class). Nothing may be fetched at page load, so large
   assets ship as a scene bundle beside `mesh_scene_data.json` rather than being inlined.
2. **Render world ≠ nav world.** The planner consumes `buildMap`'s walkable/surfaceCell
   predicates, the obstacle cylinders and the stair tread/landing stamps — not artist geometry.
   Every imported asset needs a mapping onto that vocabulary (a rack is an obstacle volume that may
   stand on walkable floor; a gate is a restricted-area boundary; a stair flight needs tread stamps
   and landing ties). Assets with no mapping land in a display-only props layer.
3. **Scope limit.** SimReady is strong on industrial props and weak on nav-critical terrain, so
   `make_example_*.py` stays the authoring path for stairs/ramps/floors and SimReady contributes the
   clutter layer that makes the escape policy and the simulated sensors realistic. An asset
   qualifies only if `multifloor/sim/test_recognizers.js` reaches the same verdict on it as on the
   hand-built fixture.
4. **Acceptance gate.** Importing props must not move a planning decision: `probe_plan.js`,
   `probe_sweep.js` and the Chrome probes under `multifloor/sim/.probe/` must return identical
   paths for the shipped demos with and without the asset layer, and a failed conversion must fail
   the build loudly — the same rule as `build.py`'s unknown `route_rotation` keys, because a
   silently skipped asset is indistinguishable from "the planner avoided it".

### 3.4 Feedforward 3DGS prior for the local planner and the escape

1. **Precomputed, never rasterised on the robot.** "Feedforward" means the splat map is built
   offline and consumed at runtime as a point-and-label prior — no per-frame Gaussian rasterisation
   on the compute board, which has no budget for it (the same reasoning that caps the DINOv3
   prototype at one forward pass). `scripts/map_processing/ply_to_pcd.py` is the reference
   converter: it drops low-opacity floaters and oversized Gaussians and writes PCD, which is the
   shape the local layer wants anyway.
2. **The blind spots it covers.** Three failures the reactive stream cannot fix today: glass,
   mirrors and dark surfaces return sparse LiDAR, so NRMP believes the space is free; geometry above
   the sensor band is invisible (a stand on a tread cannot see the flight it is on — the sensor tops
   out at +0.30); and the escape's corridor verdict (`in_corridor_now`, `corridor_half_width`, the
   bounded-rotation cap) reads only live points, so one person occluding a wall flips it and either
   disarms or over-fires the escape.
3. **Same contract as the live point set.** The prior enters in the map frame with the same
   filtering and a per-point `source` tag. The escape and `adaptive_d_max` measure in metres and
   reduce with low percentiles precisely so a stray point cannot decide `d_max` or a rotation cap;
   unfiltered splat noise reintroduces exactly that failure and gets no exemption from the filter.
4. **The prior may shape, never trigger.** Only live points may latch a stall count or a blocked
   verdict; the prior contributes clearance, side walls and above-band geometry. A stale splat of a
   pallet that has since moved must never be able to command a reversal, and prior points must die
   when a live free-space ray passes through them — the take-back discipline the occupancy layer
   learned when a scan write-back silently erased the inflation ring one cell at a time because the
   two were not rebuilt together.
5. **Slot and acceptance gate.** Select it with `local_planner.prior.backend` through the existing
   `robot.yaml` → launch arg → env chain, next to `terrain_recognizer.backend` and
   `escape_policy.backend`; absent configuration behaves exactly as today. Prove it in the mesh
   simulator first (prior plays the map layer, live data the `state.obs` layer), then on the robot:
   a glass-wall replay where live-only planning enters the glass and the prior-assisted run routes
   around it, and a moved-obstacle replay where the prior alone must not fire an escape.
   `test_escape_policy_slot.py` must pass unchanged.

### 3.5 World-model 3D navigation (Amap ABot-class)

Amap's ABot stacks a **navigable world model** on an interactive world: a data layer
(`ABot-3DGS`, a digital-twin factory that turns scanned space into a programmable environment;
`ABot-PhysWorld`, which keeps generated data physically consistent), a navigation foundation model
(`ABot-N`: cross-scene indoor/outdoor route planning and spatial understanding), a manipulation
foundation (`ABot-M`), and a predict → act → compare → self-correct loop. ONS2 has the data layer
and no learned model layer: 3D navigation today is *geometric* end to end (`map_planner`'s inflated
A\* over `sliding_map`, the `octo_planner_ros1` 3D variant behind the same `PlanPath` contract, the
tread/landing stamps the recognizers write). The target is the traversal *affordance* a world model
adds on top — what a cluttered, signless or never-before-seen space permits — not a wholesale swap
of the planner.

1. **Additive proposal layer, not a new planner.** The model may *rank or propose* routes and
   affordance regions; the deterministic chain keeps authorship. A proposal enters as an extra
   via-point / waypoint preference handed to the existing `PlanPath` contract, so the same
   `validatePath()` gate, install gate and `escape_route_gate.h` acceptance rules screen it as if
   `mission_manager` had generated it. A hallucinated route must be *uninstallable*, not merely
   unlikely — the same authority split as §3.4 rule 4 (shape, never trigger), and what lets the
   feature stay opt-in under the default-unchanged rule.
2. **Enter through the existing slots, one per surface.** Terrain semantics go through a learned
   `terrain_recognizer` instance (new backend in the factory, publishing `TerrainObservation` /
   `TerrainScene` in the fixed `type` vocabulary); local recovery through
   `local_planner.escape_policy.backend` (the registry already takes a dotted module exposing
   `create_policy(cfg)`); route affordance needs one new slot only —
   `global_planner.affordance.backend`, wired through the same `robot.yaml` → launch arg → runtime
   env chain as the other two.
3. **`ABot-3DGS` and §3.4 are the same asset.** The world model's data layer consumes 3D Gaussian
   scenes; §3.4 already converts splats to PCD (`scripts/map_processing/ply_to_pcd.py`) and feeds
   them to the local layer, and [sim/](multifloor/sim/README.md) is a miniature digital-twin
   factory: `make_example_*.py` authored terrain, the SimReady prop import of §3.3, and
   probe-verifiable planning decisions. Build the training/evaluation corpus on that pipeline rather
   than a parallel one, so a scene is usable as sim regression fixture and as model data without
   re-export.
4. **Inference stays off the authority path.** A VLA-scale model does not run on the dog's compute
   board — the SPEC's board budget is the same constraint that caps the DINOv3 prototype at one
   forward pass (§3.1.4). Either distil down to that budget or run it on the edge host already used
   for the VLM terminal, behind a hard timeout, with the robot navigating normally when it is
   unreachable, late or absent. Every learned surface keeps the abstain contract: `valid=false` /
   no opinion, never a guess (the §3.1.5 guardrails — a false `stair_up` verdict suppresses the
   escape and strands the robot at the foot of a real staircase).
5. **Acceptance gate.** Prove it in the mesh simulator first — an ABot-class affordance proposal
   scored against the shipped demos' A\* baselines, where "better" must mean shorter or wider
   *and* still passing `test_recognizers.js` — then on the robot with two replays: an unseen
   cluttered corridor the geometric cost model has no vocabulary for, and a cross-floor handover.
   Non-regression is explicit: `probe_plan.js`, `probe_sweep.js`, `probe_nogo.py`,
   `probe_autonogo.py`, `probe_escape_contract.js` and `test_escape_route_gate.cpp` (all under
   `multifloor/sim/.probe/` except the C++ test) must return identical verdicts with the new slot
   unconfigured.
6. **Closed loop last.** The predict → act → residual-feedback self-correction that defines the
   ABot stack is the piece ONS2 is least ready for: it needs per-transition records that survive a
   restart, and localization stays Odin1 MindSLAM's, so any predicted-vs-actual comparison must be
   made in the `map` frame against a pose source that does not know the model exists. Log the
   residuals early (the `/neupan/mission_escape_trace` shape is the precedent), but keep
   self-correction out of scope until there is a stored corpus to correct against.

### 3.6 Fuse LingBot-Navi for global planning and local dynamic planning

[LingBot-Navi](https://github.com/Robbyant/lingbot-navi) combines reusable 3D spatial memory,
object grounding and image localization with a vision-language backbone and a trajectory decoder
for coordinate navigation. Its reference local planner follows the predicted route from the current
rendered view and pose. ONS2 should integrate it as **two adapters behind the existing planner
slots**, not treat the upstream demo as a drop-in ROS planner: the published 3DGS execution example
uses kinematic updates without collision physics, so dynamic-obstacle safety must come from live
inputs and ONS2's deterministic gates.

1. **Global adapter behind `PlanPath`.** Add an opt-in `global_planner.backend: lingbot_navi`
   adapter advertising the existing `nav_interfaces/srv/PlanPath` contract. Translate the ONS2
   start, goal, active floor frame and spatial memory into a route request, then return a map-frame
   `nav_msgs/Path`. No route, low confidence and timeout are failures, never invented waypoints.
   The route must still pass `validatePath()` and the `mission_manager` install gate; the model
   cannot inject a path around them.
2. **Local adapter for dynamic execution.** Select `local_planner.backend: lingbot_navi` with
   `interface.mode: action`, preserving the existing `nav_path` action, `stop_topic` and `cmd_vel`
   contract. Replace the upstream 3DGS renderer with time-synchronized live camera/depth or
   projected LiDAR plus MindSLAM/TF pose, and convert each predicted turn/forward action into a
   bounded `geometry_msgs/Twist`. Live point-cloud clearance, footprint constraints and velocity /
   acceleration limits remain authoritative. A stop, preempt or invalid observation must zero
   `/cmd_vel`; fallback to NeuPAN requires an explicit handover so the single-publisher invariant
   is never broken.
3. **Spatial memory is a prior, not a pose authority.** Build LingBot-Map memory from the same
   scene corpus and assets used by §3.3 and §3.4, retaining the source splats where the model needs
   them and deriving ONS2 geometry from that same source. MindSLAM remains the localization and
   floor owner. LingBot-Navi's image-localization estimate may condition a route request only when
   it agrees with the current MindSLAM/TF pose within a configured bound; disagreement, a map
   change or stale memory must produce an abstention rather than a relocalization jump.
4. **Keep GPU inference off the control path.** The upstream release requires Linux x86_64, Python
   3.10+, an NVIDIA GPU and CUDA 12.8, and reports evaluation on H20-3E hardware. Run the model on
   the edge host behind the ROS adapter with hard request deadlines, sequence numbers and
   stale-frame rejection; the robot-side adapter must never block a control callback on inference.
   An unreachable or late host degrades to a failed global request or a controlled local stop and
   deterministic fallback. Keep the upstream code, weights and dataset as an optional external
   dependency until their licenses are reviewed.
5. **Acceptance gate.** First reproduce the upstream static SR/SPL baseline, then test in the mesh
   simulator with moving obstacles, occlusion, stairs/ramps and restricted/no-go areas against the
   shipped `map_planner` + NeuPAN baseline. Compare success rate, SPL, minimum clearance, safety
   interventions and end-to-end latency. Finish with low-speed ROS 1 and ROS 2 replays covering
   sensor dropout, delayed inference, stale memory and a moving obstacle; all existing probes and
   contract tests must remain unchanged when the backends are not selected.

---

## 4. Compatibility rules (do not regress these)

- **Default unchanged.** Absent configuration keeps `stuck_escape` and launches no terrain node.
- **Message superset.** `TerrainObservation` carries everything the legacy `/stair_gait/status`
  JSON exposes — `state`, `steps`, `rise`, `span` map to `legacy_state`, `steps`, `rise_m`,
  `span_m` — plus `confidence`, `geometry`, `backend` and `instance`, and the node republishes the
  legacy JSON (`state`, `steps`, `rise`, `span`). Disable the republish (`publish_legacy: false`)
  only when the legacy actuation node still owns the topic; two publishers on one topic is a bug,
  not a feature.
- **One legacy verdict.** However many instances run, `/stair_gait/status` stays a single reduced
  verdict, and only instances overlapping the forward corridor may vote. Anything else lets a
  side-facing region suppress the escape while the robot walks into a staircase.
- **Same tests.** A new escape policy is accepted only if it passes the reference suite; a new
  terrain backend is accepted only if it degrades to `valid=false` on missing input instead of
  emitting a guess.
