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

## Limits worth knowing

- 5,952 standable cells, of which the 0.30 m band costs 218 (3.7%) - a fraction of the plate, not the old version's striped floor.
- The probe shares the planner's edge rules, so it proves "reachable under these rules", not that the dog can walk it: `stair_gait`, the 0.42 m body, and the site upstands are not modelled.
- The thin strips in the view are the model's own counter tops, desk tops and stair treads - real horizontal faces, not breaks.
