# Multi-floor mesh navigation simulator

A self-contained browser simulation of the ONS2 global-planning chain over a
3D-scanned mesh scene (multi-floor, stairs, landings). It is the pre-flight
harness for the multi-floor branch — planning, the stuck-escape route rotation,
and the terrain recognisers are all exercised here first, then mirrored on the
robot (see [`../escape-exploration.md`](../escape-exploration.md)).

## Files

| file | role |
|---|---|
| `mesh_nav.template.html` | the source template (edit this, never the built page) |
| `mesh_nav.html` | **generated** — the template with assets inlined; tracked artifact |
| `build.py` | builds `mesh_nav.html` from the template + `--profile` escape config |
| `make_robot_models.py` → `robot_models.json` | bakes the 宇树 Go2 / 智身 钢镚 L1 URDF meshes the page inlines (see 机型) |
| `mesh_viewer.template.html` / `mesh_viewer.html` | mesh-only scene viewer |
| `escape_contract.fixture.json` | shared sim↔robot escape contract (drives the JS probe and the C++ gtest) |
| `inspection_task.example.json` | the sample inspection task inlined by `build.py`; run it with `#single,floor,insp` |
| `example_*.obj` + `make_example_*.py` | sample scenes and the scripts that generate them (see below) |
| `recognizers/`, `test_recognizers.js` | terrain recognisers + their node tests |
| `.probe/` | headless-Chrome / node probes (regression suite) |

## Samples

| button | file | what it is for |
|---|---|---|
| 示例 · 四层办公楼 | `example_four_floor.obj` | `make_example_four_floor.py`; four storeys of flights and landings, the cross-storey chain |
| 示例 · 单层办公间 | `example_single_floor.obj` | `make_example_single_floor.py`; one 18 x 10 m room, built for dropping obstacles on the route |
| 示例 · 单层多地形 | `example_multi_terrain.obj` | `make_example_multi_terrain.py`; two fenced lanes over stairs, a 12 deg ramp, a 1.24 m dock and a ramp back down - the terrain-class sample |

All three are generated, at 0.15 m, in metres, Z up: rerun the script after
changing a number, then `python3 build.py` (the OBJ is inlined into the page).
`make_example_four_floor.py` also checks its own lanes before it exits — see
[楼梯口不许砌墙](#楼梯口不许砌墙-no-wall-across-a-flights-throat).
`node test_recognizers.js` reports the flights each one yields, and
`.probe/probe_terrain.js` plans one route per lane of the multi-terrain sample.

## 工具行 (the toolbar row)

The 设起点 / 设终点 row is two `.cluster` blocks: **编辑** (route, obstacles, no-go,
save/load) and **导出** (map name, floor, clearance mode, pgm, stairs). Two rules
keep it usable at any window width:

- **It never scrolls sideways.** The row used to be `overflow-x:auto` with 1861 px
  of content, so a 1366 px window lost 495 px of it — and the macOS overlay
  scrollbar is invisible, so 导出 pgm / 导出梯段 were simply not clickable. The
  export cluster now wraps as one block instead of scrolling, and each block is a
  `flex:none` cluster so a wrap can never strand a divider on a line of its own.
- **The export block was compacted** rather than just moved: the three labels
  地图名 / 导出层 / 导出口径 became an input `placeholder`, a select `title` and a
  single 导出 label; the floor select sizes to its content instead of 230 px (its
  option text dropped the voxel row range, which moved to the option's `title`);
  and 导出 pgm / 导出梯段 became one `pgm | 梯段` segmented pair under that label.

From 1440 px up the two blocks share one line; at 1366 px and below the export
block sits on its own line (87 px tall instead of 52). `.probe/check_toolbar_row.py`
sweeps 1200 / 1366 / 1440 / 1600 / 1920 px and asserts `scrollWidth <= clientWidth`,
that no export control is unreachable (`elementFromPoint`), that the page never
scrolls sideways, and that the row is one line from 1440 up.

## 地形提示 (terrain hint)

The left column, under 地图信息, carries a debug panel that answers "what ground is
under the dog right now". Each time the class changes it appends one line —
`平地`, `台阶上行/下行 · n 级 · 共 h m`, `坡道上行/下行 · 约 a°`, `台面 · 高 h m`,
`低台` — and mirrors the same text to the browser console as `TERRAIN <text>`, so a
headless run can read the sequence without the DOM. The header keeps a live
badge of the current class; the toolbar chip 地形提示 shows or hides the panel
(on by default) and 清空 empties it. The panel boots **collapsed**, i.e. as a
single header bar that already carries the count and the live class — one line of
the screen instead of a third of it — and clicking ▸ opens the history. 地图信息
boots collapsed the same way. The column order is 地图信息 → 地形提示 → 脱困日志,
so the escape log (when its chip is on) sits directly under the terrain hint and
fills the rest of the column.

The class comes from the map, not from the body: the ground stamp under the body
is looked up in the voxel column, a flight is matched on its own claimed tread
cells (the flight frame overshoots on purpose), and the ramp angle is a
least-squares fit through the whole run of steep stamps. A height refinement of
less than 0.35 m on the same kind of platform rewrites the last line instead of
appending, so the first treads of a flight do not produce three 台面 lines, and a
new class has to hold for 0.3 s before it is logged - a landing that wraps round
the head of a flight sits inside the flight's box for a step or two.

`.probe/probe_terrain_hint.py` is the regression: it rides lane A of the
multi-terrain sample end to end, then the four-storey sample via `#demo`, and
asserts the sequences, the fold, the DOM/badge/count consistency and the panel's
hide / reopen behaviour. Collapsing does not stop the history: the lines keep
piling up in the retained array and in the hidden body, so expanding shows them.

## 场景透明 (scene transparency)

The imported OBJ is drawn at 42% opacity so the walkable plates, the orange flight
band and the route read through the walls. The toolbar chip 场景透明 (checked by
default, next to 地形提示) flips it to an opaque shell at 0.96 with `depthWrite`
on — a translucent surface that writes depth would hide the plates drawn a few
millimetres above it on the far side, and an opaque one that does not write depth
lets the cells behind it leak through.

The flip only touches the material: the map, the recognised flights and the route
are computed from the vertices and are not rebuilt, so flipping mid-run keeps the
plan, the endpoints and the dog where they were. `transparent` selects a shader
variant in three.js rather than a uniform, so the material is recompiled on the
flip (`needsUpdate`). `mesh_nav.html#solid` boots with the switch off — the same
hash the older screenshot scripts already used. `.probe/probe_scene_alpha.py`
asserts the material both ways, the map/path identity across the flip, and the
`#solid` boot (it also writes `shot_alpha_off.png` / `shot_alpha_on.png` from one
camera for the visual check).

## 站在地面上 (feet on the ground)

Two separate things could draw the dog below the terrain, and both were fixed for
the 单层多地形 sample:

- **The route, not the drawing.** The ground-run shortcut merges a run of ground
  cells into straight chords when a walkable cell sits within one layer of the
  chord. A slope is not straight in z — the raster rises a layer every few cells —
  so a chord over the 12° ramp sagged up to 0.30 m into the wedge. Worse than the
  drawing: the same chord flattened the route's apparent grade to 7.7%, under
  `isStair()`'s 8% hand-off, so the local layer took the ramp for open floor, read
  the ramp's own cells inside its `base_z +0.12 .. +0.55` band, and parked the dog
  on the slope (`blocked` for the rest of the run). The shortcut now refuses a
  sample whose cell is `steep`: a chord may not cut a slope, so the ramp is walked
  cell by cell, z follows the raster, and the graded hand-off works again.
- **The height, not the route.** A flight spine point is stamped at the tread
  *top*, and the dog's feet reach 0.077 m below its origin, so drawing the route's
  z put the feet a foot-width into every tread. The tick now resolves the height
  from the map before drawing: the first stamped top face at or below the body's
  own layer, plus the foot drop — which is the cell centre to within 2 mm on flat
  ground, so nothing moves there. Two passes settle the body onto the step it is
  over (one pass left it 7 cm inside the next riser's toe), a sub-2 cm difference
  is left alone so a voxel boundary cannot jitter, and the scan is bounded to
  three layers so a body inside a wall or a post shaft is not hoisted onto the
  roof.

`.probe/probe_ground_clearance.py` is the geometry test: it drives `prog` from
outside (`__MESH.hold`), lets a frame run, and measures the *drawn* body against
the stamped surface under it — worst sink +0.018 m on lane A and +0.020 m on the
four-storey control (was −0.374 m on the ramp). `.probe/probe_lane_walk.py` is the
behaviour test: 22.5 s of real ticking must complete lane A with no blocked
station and no sustained frame inside a solid cell — the symptom the sagging
chord produced was a dog parked on the ramp for the whole run.

## 楼梯口不许砌墙 (no wall across a flight's throat)

With 场景透明 off, the four-storey sample showed a solid panel in front of the
first riser of 2→3 and 3→4 — but not 1→2. It was the guard rail around the
stairwell opening, and the west lip of the opening is not only rim: both flights
meet the floor there. Flight B lands on it (`y 3.50..4.60`) and flight A leaves
from it — its first riser *is* the lip (`x = SX0`, `y 1.30..2.40`). The rail
spanned `SY0..BY0` as one box, so it covered the throat of the flight climbing
out of that floor. Floor 1's rim carries no rail at all, which is why 1→2 was
clean. The west lip is now split into the spans between the lanes, and only the
top floor — where nothing climbs above — keeps it closed end to end.

Nothing in the planner could have reported it: the rail is 0.06 m thick, under
half a 0.15 m cell, so it only shades one voxel column, and the chord shortcut
skipped that column. The route and the dog walked straight through a wall that
the map had not fully seen; only the opaque model drew it. Hence the check is on
the **geometry**, at generation time: `make_example_four_floor.py` now walks every
tread column and the floor strip each flight meets (`check_lanes()`), ignores what
a walker stands on (ground, slab, treads, landing), and exits non-zero naming the
offending box. It flagged exactly the two floors the user reported and nothing on
storey 0.

Freeing that column widened the lane by one cell, so the demo route shifted
slightly rather than breaking: 88 pts / 43.83 m → 91 pts / 44.29 m, still 0 steep
edges, `probe_plan` maxDev 0.0000, `probe_sweep` bad tasks 0. Body-vs-geometry
penetration lost the `rail_hole` hit at `v=15.75` and keeps only a 2.8 cm corner
brush against flight B's own handrail at `v=8.30`, i.e. the dog's flank touching
the rail it walks beside — not a wall in the way.

## 脱困要能把尝试范围放大 (the escape must widen its attempt range)

On the four-storey sample, from `(0.81, 2.13, 0.08)` to `(7.01, 1.49, 10.8)`, the
dog stalled at the top-floor corners and no escape attempt could install: at the
first corner (west lip) only with 合成传感 off, at the second (goal pocket) with
it on or off. The escape log showed the same thing every round —
`结果：无可安装候选`. It was not a threshold tuning problem.

**Why the old pool could not win.** Every candidate was a nudge: one berth, one
via, `planStops([dog, via, goal])`. `routeSafety` is a minimum over the WHOLE
chain, so a candidate that fixes one corner still scores the corners it did not
move, and A* re-plans each leg short, so it cuts the next corner back to the
same 0.10..0.15 m. The top floor hugs the opening's rail band three times in a
row (leaving the stair head, the west lip, the goal corner) inside 0.15 m, while
the gate wants `curD + 0.05` (≈0.156) on round 0 and
`min_candidate_clearance_m` (0.175) after that. Measured: `curD` 0.106..0.112,
best single-via candidate anywhere 0.150. A deadlock, not a tuning miss.

**What replaced it** (all sim-only — the gate arithmetic between
`ESC_CONTRACT_BEGIN/END` is untouched, `probe_escape_contract.js` still passes):

| kind | shape |
|---|---|
| `走廊中线·单` | corridor recentre: one station's free interval, measured with the body band (`bandClear`, dz 0.12..0.55 — `walkable()` forgives the map builder's one-cell sheet lift and reported 0.67/0.90 m across a rail 0.15 m away) |
| `中线链` | the route rebuilt through the clearance-maximising cell of every station: a 2D chamfer field of the body band (`bandField`), scored `min(clr, 0.35) - 0.10·dev`, so it centres where the room is tight and stays near the route where it is already clear |
| `中线链·宽` | same chain with no distance penalty — a robot stuck *at* a corner has to back off to round it, and the distance-aware chain never proposes that first step |
| `贴边进场` | goal bracket: two vias straddling the goal, so neither final leg is free to cut the corner |

Rounds escalate the amplitude: reach 0.9 → 1.7 → 2.6 m each side, look-ahead 10
→ 22 → 45 m, vias 4 → 6 → 8, and the wide chain appears from round 1. The whole
pool is deterministic per round.

Two predicates had to be added, both earned by measurement:

- **A via must be on known ground** (`knownGround`). The top storey's walls end
  at its walking plane, so the roof rim is body-band clear and a
  clearance-maximising escape scored 0.473 m by driving the dog along the edge
  of the map. Requiring the 4-neighbourhood to be known keeps proposals inside
  the mapped floor.
- **The robot's own footprint is not scored** (`CLR_SELF` = 0.35 m in
  `staticClearanceAhead`). The cell the dog stands on reads 0.10 m from the band
  while the route ahead is 0.45 m clear; counting it pinned EVERY candidate to
  the cell the dog was stuck in — a medial chain 0.45 m clear scored 0.134 only
  because sample 0 sat 0.09 m from the dog. The escape is now judged on the
  route it will drive. Sim-only: the ROS twin takes clearance from the planner's
  `validatePath` and only shares the gate arithmetic.

**Measured after** (same endpoints, `probe_escape_widen.py`): installs at every
stall station — 0.106 → 0.184 at `s=40.0` round 0, 0.112 → 0.237 at `s=45.6`,
0.106 → 0.335 at `s=47.2`, and 0.107 → 0.450 at `s=42.8` on the retry (round 0
there sits out: the dog is *inside* the corner, so the first 0.35 m of any
candidate is that corner). End to end the dog now completes the pass and loops —
two escape attempts on the top floor, no stall — and a second pass needs none.
A burst costs 0.3..0.9 s for 12..17 candidates (was ~0.4 s), which is why the
many-via corridor chain and the `中线链·近` prefix were dropped once the medial
chain subsumed them.

`.probe/probe_escape_widen.py` is the guard: it parks the dog at each stall
station, fires the round the escape would reach there, and requires an install
whose clearance beats the route it replaced.

## 面板要自己点得动 (panels must catch their own clicks)

`#leftCol` is `pointer-events:none` so a drag in the empty part of the column still
orbits the camera, which means every panel in it has to opt back in with
`pointer-events:auto`. 地形提示 was added after the column was already a flex stack
and was missing from that list, so the whole panel was transparent to the mouse:
the ▾ and 清空 buttons could be seen but not clicked — and because calling
`.onclick()` from a probe still worked, it read as "the collapse button does
nothing" rather than as a hit-testing bug. When a panel is added, add its id to
that one rule.

`.probe/probe_panel_hits.py` is the guard: it dispatches real
`Input.dispatchMouseEvent` presses (not `.onclick()`), and asserts the boot state
(地图信息 and 地形提示 collapsed, 脱困日志 expanded), the column order and that the
escape log starts below the terrain hint, that each control's centre hit-tests to
itself, that the three ▾/▸ buttons flip class, body visibility and arrow together
and flip back, that 清空 empties the log, and that the gap between two panels still
reaches the canvas.

Collapsing is two rules, not one: hide the body **and** drop the `flex-grow`.
脱困日志 is `flex:1 1 0` so it fills the column; with only `.ebody{display:none}`
the panel kept its full height as an empty box, which looks like the button did
nothing — hence `#escLog.min{flex:0 0 auto}`.

## 巡检任务示例 (inspection task demo)

The page can run a **structured inspection task**: a list of stations, each with a
sensor and an action. The dog plans the route through them, parks at every station
(the route arc is pinned, so it really stands still instead of gliding past), runs
that station's action, and appends the result to a report panel in the top-right
corner. The task file speaks the schema the inspection backend pushes to the real
console — see [`../../inspection.md`](../../inspection.md) and
`web_console/inspection/schema.py` — so `step_id` / `name` / `sensors` /
`actions[].type` / `actions[].params` / `dwell_sec` mean exactly what they mean on
a robot: `measure_scan`, `capture_image`, `capture_burst`, `speak`, `wait`, and the
sensors `camera` / `lidar_scan`. **The one sim-only field is `target`**: the robot
takes a `point_id` or `x`/`y`, the page also accepts where on the route to stop
(see the table below), which is what lets the same file run on any mesh. Swapping
`target` back is the only edit a task needs before it goes to a real console.

A step's action list is a list on the robot; the page runs the **first** action of
the step (one station = one action in this version) and shows the sensor that goes
with it.

While a task is running, a badge is pinned above the **dog**: the top centre of its
body box is projected every frame, so it follows the dog while you orbit, zoom or
fly the camera, and it carries the live station, the state and a progress bar (same
`state.insp` the report panel reads — one task state, not a second copy). It is a
screen-space element, not a 3D sprite: the type stays readable at any distance and
the Odin1 photo never catches it. It hides itself when the task is cleared or the
dog leaves the canvas.

One-click demo (loads the one-room sample, sets its own start/goal, plans, then
runs the task):

```
mesh_nav.html#single,floor,insp
```

or press **巡检示例** on a route that is already planned. Without a route the
button says so instead of silently doing nothing.

The task itself is a standalone file, inlined into the page by `build.py` (the
page is opened from `file://` as often as from a server, and a `fetch` there fails
silently):

| file | role |
|---|---|
| `inspection_task.example.json` | the example task (4 stations, all four actions) — edit this, then `python3 build.py` |

`target` accepts four forms, which is what lets one task file run on any imported
model:

| target | meaning |
|---|---|
| `{"from": "start"}` / `{"from": "goal"}` | bind to the planner's endpoints |
| `{"route_at": 0.42}` | 42 % of the planned route's arc length, model-independent |
| `{"waypoint": 1}` | bind to the 2nd 途经点 placed in the panel |
| `{"xyz": [x, y, z]}` | an explicit map coordinate (becomes a routing stop) |

The four actions are the V1 set; every reading comes from data the page is
already using, not from a decorative formula:

| action type | sensor | what happens |
|---|---|---|
| `capture_image` / `capture_burst` | `camera` | shoots a 640x480 JPEG from the dog's head-mounted Odin1 (same 0.30 m mount and FOV constants as the synthetic scan); the row shows a 4:3 thumbnail, clicking it opens the full frame in a window-sized overlay (`Esc` closes) |
| `measure_scan` | `lidar_scan` | one occluded raycast through the page's own Odin1 model; reports returns inside a ±60° / 3 m front sector, the nearest distance, and the lateral free width the local layer would get |
| `speak` | — | writes `params.text` to the status line and into the report |
| `wait` | — | dwells for `dwell_sec` |

Behaviour worth knowing:

- The task ends `SUCCEEDED` with the dog parked at the goal. The pass loop that
  keeps the plain demo running is suppressed while finished, otherwise the report
  says "已完成" and the dog drives off again.
- Any 规划 click, model reload or parameter rebuild invalidates the station arcs, so
  the task is marked `INTERRUPTED` rather than pretending it is still running.
- Pause holds the dog at the current arc; resume continues to the next station.
- The panel's ✕ drops the task and its report; `重跑` starts the same task again.

The probe drives the whole cycle headlessly and asserts parking, the four report
entries, the capture's mount geometry and JPEG size, the panel rows, the
click-to-zoom overlay and the no-loop rule:

```
python3 .probe/probe_inspection.py
```

## 机型：实物 URDF 模型 (the dog is the machine)

The dog is display-only - planning never reads its mesh, the heading comes from
the route tangent - but the body is now the vendor model, not an abstraction:

| 机型 | source | visual shell |
|---|---|---|
| 宇树 Go2 | `unitree_ros/robots/go2_description` | `base/hip/thigh/calf/foot.dae` (+ mirrors) |
| 智身 钢镚 L1 | `genisom_model/zsl-1` | `BASE_LINK` / `*_ABAD` / `*_HIP` / `*_KNEE` / `*_FOOT` `.STL` |

`make_robot_models.py` keeps the URDF's **visual** meshes (Go2's collision set is
coarse boxes and cylinders - not what the robot looks like), decimates each one by
vertex clustering on a 4–5 mm grid and writes `robot_models.json` (0.9 MB, from
37 MB of raw DAE/STL). `build.py` inlines that JSON, so the real shell appears
with **no network** and from `file://`; a missing `robot_models.json` fails the
build instead of silently drawing the procedural stand-in.

```
python3 make_robot_models.py                 # local vendor checkouts (default)
python3 make_robot_models.py --go2 DIR --l1 DIR
python3 build.py
```

Details that bite when regenerating: `assimp export` writes Y-up (the COLLADA
importer applies `<up_axis>Z_UP</up_axis>` and an OBJ cannot convert back), so
the bake un-rotates the DAE output; the URDF's kinematic ancestors of every
visual link must survive the trim or limbs detach; and the trot rig classifies
joints by **axis** (abduction = x) rather than by name, because Go2's
`FL_hip_joint` and L1's `FL_HIP_JOINT` are different joints. The stand pose
(`crouch`) is FK-matched — the URDF rest pose is straight-legged.

```
python3 .probe/probe_dog_model.py      # baked models: tris, rig, grounding, swap
python3 .probe/probe_urdf.py           # parser + GitHub-fallback branches
```

## Build

```
python3 build.py
```

This regenerates `mesh_nav.html` (byte-comparable in behaviour until a profile
changes a constant). The built page is tracked, so commit it with the template.

### `--profile` (escape tuning)

`build.py` injects the `stuck_escape.route_rotation` block from a robot's NeuPAN
config into the page as `window.__ESCAPE_CFG`, so the simulator's stuck-escape
decisions are tuned by the **same** parameters the robot reads — the point being
that a tuning claim made in the simulator transfers.

```
# default profile is robots/go2/neupan/config.yaml
python3 build.py

# point it at another robot
python3 build.py --profile ../../../robots/sw01/neupan/config.yaml
python3 build.py --profile=../../../robots/dobot_lumo/neupan/config.yaml
```

The parser reads only `stuck_escape.route_rotation` (hand-rolled scalar / inline
list parsing, no PyYAML dependency), mirrors how `build.py` already inlines
assets, and **fails loudly** if the marker is missing or a key is unknown, so a
typo in the profile cannot silently ship as "the sim ignored my tuning". Every
`route_rotation` default in the page equals the pre-config simulator constant, so
a profile that omits the block behaves exactly as before it was configurable.

## Test

The regression suite lives in `.probe/` (headless `google-chrome` + a small CDP
harness, plus node-only probes). Run the probes after any rebuild; the full list
and current pass counts are recorded in
[`../escape-exploration.md`](../escape-exploration.md) and
[`../../architecture/RESTRICTED_AREA_MODEL.md`](../../architecture/RESTRICTED_AREA_MODEL.md).

The inspection demo has its own end-to-end probe:

```
python3 .probe/probe_inspection.py     # #single,floor,insp, screenshots verify_inspection.png
                                     # + verify_inspection_zoom.png (the opened photo)
python3 .probe/check_head_badge.py     # head badge: follows an orbit, pause styling;
                                     # verify_inspection_head.png / _paused / _closeup
python3 .probe/check_insp_zoom.py      # same overlay, but clicked with a real mouse over HTTP
```
