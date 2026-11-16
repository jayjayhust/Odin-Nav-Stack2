# Imported mesh · global planning (`mesh_nav.html`)

Load any OBJ / STL - multi storey stairs included - and let the page work out which faces are floor and which are wall, then plan over what is walkable.

**This page does not invent an algorithm.** The map and the search are ports from `ros_ws/src/octo_planner_ros1`:

| In the page | Source in the repo |
|---|---|
| Surface voxelisation, upward clearance | `voxel_grid.cpp: insertSurfacePoint()` |
| Inflation | `voxel_grid.cpp: inflate(rxy, 0, ...)` - flat XY disk, no z extent |
| Ground support test | `voxel_grid.cpp: hasGroundSupport()` |
| Start snapping | `voxel_grid.cpp: nearestWalkable()` |
| 26-neighbour A*, no pure vertical step, cost, adjacent-goal acceptance | `astar3d.cpp: plan()` |
| Parameters | `nav_sim.cpp:1071` (`max_step_z=3`, `snap=12`, `require_ground_support`) |

Added on top, specific to a mesh source:

- Triangles sampled at 0.8 cell; clearance is stamped upward only through faces whose normal points up (`nz > 0.35`). **This one matters**: a slab's underside points down, and stamping clearance there opens the whole floor plate so the search walks through a storey.
- Horizontal faces are stamped over a 3x3 neighbourhood, matching `nav_sim.cpp: stampPoint()`. Without it the plate arrives as a sieve, the 5x5 support test counts fewer than 8 cells and most of a storey is rejected as unsupported.

## What works

OBJ / STL import, voxelisation, inflation, standable surface in yellow and the standable surface the inflation band costs in red, start and goal picked on the model, resolution and inflation selectable.

Four storey sample (`example_four_floor.obj`, 293 boxes, 3,516 faces): 5,952 standable cells, strata `5:600 28:518 51:541 73:342 ...`.

## Port pitfalls already fixed

1. **Clearance stamped through downward faces too.** The slab underside emptied the floor plate. Now only `nz > 0.35` faces stamp upward.
2. **Plate arrived as a sieve.** 0.8 cell sampling is coarse against a 0.15 cell grid, the 5x5 support test wants 8 cells and almost everything was rejected - 312 cells per storey, 12%. With 3x3 spreading it is 600.
3. **Overhanging treads all rejected.** `require_ground_support` wants something solid underneath, and a tread has nothing under it because nothing was ever measured there. Relaxed the way `nav_sim` does with `allow_unknown_cells`: an UNKNOWN cell underneath counts as "may be supported".
4. **Start point in a corner.** A stratum centroid lands on a pillar or inside a rail; now the cell with the most open neighbours on that stratum is taken.

## Cross storey planning: it works

Four storey sample, start on 1F, goal on 4F:

| Inflation | Path | Length | Expanded |
|---|---|---|---|
| 0 m | 134 cells | 30.0 m | 4,453 |
| 0.30 m | 140 cells | 31.4 m | 3,604 |

30 m is plausible for four storeys: three flights are about 21 m, the rest is level running at both ends.

## Why it looked broken for several rounds

The repeated "no path" was never the planner. It was where the demo put the goal:

- The old demo chose the goal by widest stratum, and picked a small detached plate on 4F.
- A reachability probe (`flood()`, same edge rules as the planner) measured it: **the start component is 5,148 cells and reaches from storey 0 to storey 3** (`{0:831, 1:1531, 2:1442, 3:1344}`), so the stairs were connected all along; **the goal component is 162 cells, all on storey 3** - an island.
- The start could not walk to that island, so A* exhausted its frontier and reported no path. The report was correct; the test point was wrong.

Fix: run `flood()` from the start first, then choose the goal only inside that reachable set, on the highest stratum that is still a real plate (`>=150` cells). Both inflation settings now plan.

This probe is worth keeping: **when a plan fails, measure the two components before suspecting the algorithm**.

## Global blue, local yellow: the local leg follows NeuPAN, not a homemade A*

- **Global** `0x4fc3ff`, `renderOrder 10`, `depthTest:false`: planned once, drawn end to end.
- **Local** `0xffcf3f`, `renderOrder 11`, under the constraints of the deployed NeuPAN (`robots/go2/neupan/planner.yaml`): differential drive, `max_speed 0.625 m/s / 0.875 rad/s`, `receding 10`, `collision_threshold 0.05`.
- **Window** `0x6fd3ff` wireframe, shown on level segments only.

The previous version drew "3D A* re-run inside a 2.5 m box on the inflated grid", and that was wrong: `nav_sim.cpp` is explicit that NeuPAN constrains the robot against the point set itself - "no occupancy grid and no inflation" - and that `half width + 0.05 m` is a hard limit, not a cost term. What runs now:

- obstacles are **un-inflated OCC voxel centres**, filtered to the body height band `base_z +0.12 .. +0.55` (the floor drops out of the band, otherwise standing on it reads as distance zero);
- candidates `(vx, vy, wz)` are sampled and rolled out 10 steps of 0.2 s;
- any candidate that comes within `halfW(0.175) + 0.05` is **discarded**, not penalised;
- cost = distance to the look-ahead point + 0.55 x heading error - 0.25 x clearance. Without the heading term "stand still" is always cheapest.

**What it is not**: the DUNE network and the PAN optimisation. Those are a `.pth` checkpoint and torch; the page has neither. This is the constraint skeleton, not a learned policy. Real NeuPAN runs in ROS.

### No NeuPAN line on a flight, and that is not a bug

NeuPAN's band is `base_z +0.12 .. +0.55`, and the next riser sits inside that band right under the dog - every candidate reads as a collision. That is exactly why the repo hands stairs to `stair_gait` and leaves the landings to the local planner (`Scenario::flights_only`).

The page follows the same split: NeuPAN on level segments, and on a flight the yellow line simply follows the route's centre line with the window hidden.

### There are two local planners in the simulator, not one

`Scenario::planner` in `nav_sim.cpp` has two, defaulting to `neupan`:

- `neupan` - the deployed one: differential drive, 1.0 s MPC, hard point-set threshold, **no grid and no inflation**;
- `dwa` - the sampling planner the simulator grew, which **does use the inflated grid**.

The page now matches the first. The comparison lives in `3d-planner-and-stairs`.

## Dropping your own obstacles on the route

- **Post**: click the button, then click a walkable face and a red post appears there (r=0.30 m, h=1.1 m).
- **Pedestrian**: r=0.22 m, h=1.65 m, walking at **0.28 m/s** (the speed the simulator's scenes use), bouncing off the map bounds.
- **Clear**: removes all. A drag never places one - a pointer that moved more than 5 px is treated as orbiting.

Obstacles enter the local layer the way a live return does: **no inflation**, filtered to the body band `base_z +0.12 .. +0.55`, and anything within `halfW 0.175 + 0.05 m` is illegal.

### The dog stops, and that is the right answer

When no candidate survives in the local window, `state.blocked` latches and the dog stops advancing - exactly what NeuPAN does when PAN finds no feasible solution: it parks and keeps receding rather than pushing through. The yellow line falls back to the centre line.

### One simplification, stated plainly

The dog's position is still driven by **the global route**; the local layer only answers go / stop and **does not take over steering**. Real NeuPAN publishes `/cmd_vel` and the dog genuinely swerves. Here the swerve shows up as the yellow leg deflecting while the dog still hugs the blue line. To see the dog itself turn, run real NeuPAN in ROS.

`#demo,obs` and `#demo,ped` drop one of each automatically.

## Two simulators, one robot: the same planning stack

The point cloud simulator (`nav_sim.cpp`), this mesh page and the real robot run one three layer structure, not three separate implementations:

| layer | what the repo implements | point cloud sim | mesh page |
|---|---|---|---|
| global | `astar3d.cpp`: 26 neighbours, no pure vertical step, at most `max_step_z = 3` cells per expansion, cost `res·|d| + z_bias·res·|Δz| + lateral` (defaults `z_bias = 0`, `lateral = 0`) | used directly | copied rule for rule, default parameters |
| standable | `voxel_grid.cpp` `hasGroundSupport` (a thin sheet is separated from a wall hit by 8 occupied cells in a 5x5 box) plus `hasTreadSupport` for stairs | `stair_tread_support = false` (default) | on, with the same test |
| local | `robots/go2/neupan/planner.yaml`: point set obstacles, `halfW + 0.05` hard reject, `receding 10 × 0.1 s` | `Scenario::planner = "neupan"` | the same constraint skeleton, no DUNE / PAN |
| dynamic avoidance | candidate `(vx, vy, wz)` rollouts with a hard clearance threshold | same | same, plus obstacles and pedestrians you drop in |

**The one deliberate difference is `stair_tread_support`, and that default is measured**: switching it on and running the ten `nav_sim` scenarios takes `four_floor · DWA` from `reached=1` to `reached=0 collided=1` (NeuPAN still arrives). So the robot and the point cloud sim keep it off, and the mesh page has to turn it on: its map is a voxelised triangle shell, with none of the 3x3 ground return stamping that leaves the point cloud flights plannable.

With the flag off, all ten scenarios are **identical to before the change** - `office_loop / corridor_block / narrow_gate / stair_two_floor / four_floor` under both NeuPAN and DWA, matching `t / len / mincl / replans / swerves`, an empty `diff`.

## A one storey sample, built for obstacle tests

The four storey model is awkward for this: the camera has to dive through three plates to see anything you just dropped. So there is a second sample, **one storey with stairs**: a 12 x 8 m room, 30 boxes and 360 triangles, **all mathematical boxes, nothing reconstructed**.

- four walls at 3.4 m, no ceiling so the interior stays readable;
- two 0.4 m columns, a 1.2 x 0.8 x 0.75 desk, a 1.6 m partition, a 0.4 m barrel;
- **up**: four treads, 0.15 m riser / 0.45 m going, onto a 0.60 m platform;
- **down**: the same four treads, into a -0.60 m pit (the floor is split into four slabs around it).

Generator: `make_example_single_floor.py`, rerun it after changing any number; it writes `example_single_floor.obj` (10,570 B).

5,725 standable cells on the floor, of which the 0.30 m band costs 115 (2.0%) - cleaner than the four storey 3.7%, because there is no dense stair core.

`#single` plans **floor to floor**: `(3.0, 4.2)` to `(9.6, 2.2)`, 44 cells / 7.26 m / 466 expanded, going round the columns and the desk.

### The flight plans now: 27 cells / 4.73 m, climbing from z 0.13 to 0.73

`#single,stair` (floor to the 0.60 m platform) plans. Measured: 27 cells, 4.73 m, 160 expanded, path z from 0.13 to 0.73 - it really walks up the treads rather than going round the outside of the wall.

**Why it took so long, measured rather than guessed**: `hasGroundSupport` demands an occupied cell exactly one layer underneath. A flight does not satisfy that - every tread is one cell thick, the next riser occupies the same layer, so the cell under a standing position is often empty or inflated. The flight then connects only through the sheet of cells floating above the treads: on the way up that sheet follows the flight and climbs to the platform, which is why ascent looked fine; on the way down it collapses to floor height and never reaches the pit bottom.

**The fix**: on a **stair column**, accept a cell when some occupied cell within `max_step_z` (3) below is part of a **wide horizontal face** - 8 or more occupied cells in the 5x5 window, the same span test `hasGroundSupport` already uses to tell a slab from a wall hit - and the four cells above are clear. A desk top is a wide face too, but a desk column holds only two or three occupied layers, so it never counts as a stair column and cannot be stood on from above.

Two real bugs from the previous round stay fixed: `snap` accepted a free cell it could not stand on (the goal sat 0.35 m in the air above a tread), and an 0.18 m riser is 1.2 cells so the grid phase never lined up (now 0.15 m, exactly one cell).

### The pit still does not plan - a known open item

`#singlepit` still reports no path. Measured: the pit floor is an isolated **74 cell component** at k=5, while the start component holds 4,799 cells across k 9..16. The east lip of the hole is a sheer 0.6 m wall with no treads; on the west side the four descending treads voxelise into one solid block, and the pit floor never connects to it.

The next measurement is not another geometry parameter - it is which single layer is missing between those 74 cells and the foot of the flight. It was left alone this round because the relaxation above already bought the ascent and the four storey case, and locking that in was worth more than reopening the pit.

## Limits worth knowing## Limits worth knowing

- 5,952 standable cells, of which the 0.30 m band costs 218 (3.7%) - a fraction of the plate, not the old version's striped floor.
- The probe shares the planner's edge rules, so it proves "reachable under these rules", not that the dog can walk it: `stair_gait`, the 0.42 m body, and the site upstands are not modelled.
- The thin strips in the view are the model's own counter tops, desk tops and stair treads - real horizontal faces, not breaks.
