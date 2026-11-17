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



### The 25 cm limit flight now runs at 0.15 m - two real bugs, not resolution

25 cm is the Go2's climbing limit. Last round it was made to work by dropping to a 0.05 m grid (5.7 million cells, 10,380 expanded nodes). This round the actual causes turned up, and **0.15 m works with no rule changed**:

| item | 0.15 m standard flight | 0.15 m 25 cm flight |
|---|---|---|
| grid | 80x53x37, about 150 thousand cells | 89x63x40, about 220 thousand cells |
| standable cells | about 5,900 | 5,683 |
| open floor | 44 points / 7.26 m / 466 expanded | - |
| climbing | 27 points / 4.79 m, z 0.13 to 0.73 | **22 points / 3.65 m, z 0.17 to 1.22** (1.00 m platform) |
| dropping into the pit | 19 points / 3.11 m, z 0.13 to -0.47 | **19 points / 3.54 m, z 0.17 to -0.88** |

The two causes:

1. **The boxes are hollow shells.** Each box in the OBJ contributes six faces and nothing else, so the inside of a solid block reads as free space with an occupied cell directly underneath - which is exactly what a standable cell looks like. The 0.60 m platform is only four cells tall, so the four cell headroom test caught it and nothing ever showed; the 1.00 m platform is about 6.7 cells, so the dog simply stood inside the block and the goal stopped being on the tread. Fix: flood the outside once from the bounding box boundary, then fill any enclosed pocket whose **bounding box spans at most 6 m**. Filling only small pockets is mandatory - a storey is enclosed too (slab below, slab above, four walls), and solidifying indiscriminately turns every storey into a brick and the four storey map instantly has no route.
2. **The z datum was wrong.** The single storey branch used `Z0 = m.lo[2] + 1.30`, and that 1.30 was calibrated against the 0.15 m model, whose `lo[2]` is -1.30. The 25 cm model has a 1.00 m pit, `lo[2] = -1.70`, so every z target sat 0.40 m low: the goal at 1.02 became 0.62, inside the 1.00 m platform. Now `Z0 = 0` - in model coordinates the floor is z = 0 - and both samples share one set of endpoints.

Built, measured, then deleted: **a semantic stair ramp**. It detected the flight (neighbouring column tops within 0.25 m), fitted a plane through the tread tops by least squares, painted one walkable layer above the fitted surface, and exempted those cells from inflation. It ran, but switching the whole layer off left 25 cm working exactly the same (22 points / 3.65 m, 114 vs 115 expanded), while it pushed the four storey search from 5,944 to 8,007 expansions. With the two bugs as the real cause, the layer was dead weight and is gone; the map is back to "support rule plus inflation".

0.05 m remains in the resolution dropdown for manual use, but the `示例 · 单层（25 cm 极限梯）` button is now 0.15 m, and the four storey sample has always been 0.15 m.

### Trying a step-edge test instead of a layer count (rejected, kept for the record)

25 cm is the Go2's climbing limit and the sample sits at 0.15 m. To support 25 cm directly, `stairCol` was rewritten to look for a step edge - a lateral neighbour whose top differs by one riser - instead of counting occupied layers. Two versions were measured:

| version | 25 cm flight | four storey demo |
|---|---|---|
| one-sided (any neighbour differs by 1..3 cells) | ascent **works**: 22 cells / 3.65 m, z 0.17 up to 1.22 (a 1.00 m platform); the pit works too, 19 cells / 3.35 m down to -0.88 | **broken**: path 1 cell / 0.0 m, inflation eating 2,668 cells against the usual 218 |
| two-sided (a step must have height on both sides) | the flight **breaks at both ends**, no path up or down | back to normal |

One-sided is too loose: it calls every plate rim a step and scrambles the four storey terrain. Two-sided is too tight: the top and bottom tread of any flight is necessarily one-sided, so the flight snaps off at both ends. Reverted to the layer count - conservative, but the 0.15 m flight, the four storey run and the pit are all stable under it.

The conclusion drawn then was that only 0.05 m could carry 25 cm. This round shows that was wrong: 0.15 m is enough, and what actually blocked it was the hollow shell and the z datum, two bugs covered in the section above.

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

The four storey model is awkward for this: the camera has to dive through three plates to see anything you just dropped. So there is a second sample, **one storey with stairs**: an **18 x 10 m** room, **44 boxes and 528 triangles**, **all mathematical boxes, nothing reconstructed**.

- four walls at 3.4 m, no ceiling so the interior stays readable;
- two 0.4 m columns, a 1.2 x 0.8 x 0.75 desk, a 1.6 m partition, a 0.4 m barrel;
- **up**: four treads, 0.15 m riser / 0.45 m going, onto a 0.60 m platform;
- **down**: the same four treads, into a -0.60 m pit (the floor is split into four slabs around it);
- **a crossing**: eight risers up, a 1.50 m landing, eight risers down - next section.

The room grew from 12 x 8 m to 18 x 10 m to fit that last piece. The grid is **130 x 76 x 37 cells**, **9,891 standable cells** on the floor, of which the 0.30 m band costs 210 (2.1%) - cleaner than the four storey 3.7%, because there is no dense stair core.

Generator: `make_example_single_floor.py`, rerun it after changing any number; it writes `example_single_floor.obj` (44 boxes / 15,721 B), and `--rise 0.25 --tread 0.28 --out example_single_floor_25cm.obj` writes the 25 cm tier (44 boxes / 15,665 B).

`#single` plans **floor to floor**: `(3.0, 4.2)` to `(9.6, 2.2)`, 44 cells / 7.26 m / 466 expanded, going round the columns and the desk.

### The flight plans now: 26 cells / 4.62 m, climbing from z 0.13 to 0.73

`#single,stair` (floor to the 0.60 m platform) plans. Measured: 26 cells, 4.62 m, 169 expanded, path z from 0.13 to 0.73 - it really walks up the treads rather than going round the outside of the wall. Enlarging the room did not change it.

**Why it took so long, measured rather than guessed**: `hasGroundSupport` demands an occupied cell exactly one layer underneath. A flight does not satisfy that - every tread is one cell thick, the next riser occupies the same layer, so the cell under a standing position is often empty or inflated. The flight then connects only through the sheet of cells floating above the treads: on the way up that sheet follows the flight and climbs to the platform, which is why ascent looked fine; on the way down it collapses to floor height and never reaches the pit bottom.

**The fix**: on a **stair column**, accept a cell when some occupied cell within `max_step_z` (3) below is part of a **wide horizontal face** - 8 or more occupied cells in the 5x5 window, the same span test `hasGroundSupport` already uses to tell a slab from a wall hit - and the four cells above are clear. A desk top is a wide face too, but a desk column holds only two or three occupied layers, so it never counts as a stair column and cannot be stood on from above.

Two real bugs from the previous round stay fixed: `snap` accepted a free cell it could not stand on (the goal sat 0.35 m in the air above a tread), and an 0.18 m riser is 1.2 cells so the grid phase never lined up (now 0.15 m, exactly one cell).

### The pit plans too: 19 cells / 3.11 m, from z 0.13 down to -0.47

`#singlepit` reported no path for a long time, and the attempts along the way - an edge test, a tread inflation exemption, a semantic ramp - were all wrong. The real cause was dull: **the 6 cm coping ring around the hole has its top face at exactly z = 0**, and the floor's 3x3 stamping then laid one occupied layer right across the hole, sealing the descending flight at floor height.

**The fix**: keep only the real retaining wall on the east lip (`PX1-0.06..PX1`) and delete the other three decorative 6 cm rings. The pit plans immediately: 19 cells, 3.11 m, path z from 0.13 to -0.47, and the 25 cm tier does the same.

### The crossing: eight up, a 1.50 m landing, eight down

A bridge straight across the room: from x = 4.00 m, eight treads each one `RISE` higher, an 8 x `RISE` landing 1.50 m long and 1.10 m wide, then eight treads back to the floor. At the 0.15 m tier the deck sits 1.20 m up and reaches x ~ 12.70; at the 25 cm tier it is 2.00 m up and reaches x ~ 9.98. Every box is solid from z = 0, nothing floats.

| hash | terrain | 0.15 m tier | 25 cm tier |
|---|---|---|---|
| `#single,ridge` | floor at the foot of the flight to the **landing deck** (eight risers are unavoidable) | 38 cells / 6.56 m / 421 expanded / z 0.13 to 1.33 | 29 cells / 5.13 m / 292 expanded / z 0.17 to 2.12 |
| `#single,ridgedn` | landing deck to the floor east of the bridge (eight risers down) | 35 cells / 6.05 m / 784 expanded / z 1.33 to 0.13 | 26 cells / 4.62 m / 198 expanded / z 2.12 to 0.17 |

Why those endpoints: the first attempt was "floor west of the bridge to floor east of it", and it gave **76 cells / 11.93 m with z flat at 0.13** - A* went round the bridge, because climbing eight risers does not beat walking 1.1 m around it. Putting the goal **on the landing deck** (1.20 m up, reachable only up a flight) turns "must climb" into something a measurement can confirm. The bridge is still walkable round; that is stated rather than hidden.

### Why the bridge is now 1.10 m wide: the dog used to climb along the nosing

Recording the walk showed the dog tracking the **south edge** of the treads instead of the middle of the flight. A CDP probe of `dogMesh.position` reproduced it: over `prog` 0 to 6 m, y went 7.28 -> 6.60 -> 6.89 -> 7.12 -> 7.28, and 6.60 is exactly the south boundary of the deck (7.20 in model coordinates). Tagging every point of the path showed `stairCol` = 0 throughout - A* stands in the **free cell above the tread**, level with the whole bridge band, so the cheapest cell lands on the column nearest the edge.

Two post-processing centring passes were tried, both failed, both removed:

1. Filtering on `stairCol`: a single tread is only two cells thick and never counts as a tall column, so the flag is 0 for the whole path and the pass did nothing.
2. Centring on "same layer walkable plus equal column top, take the midpoint": the column top of that free cell is flattened all the way to the north floor boundary, so the climb points were pushed to the north edge (y 7.88) - worse than the original.

The fix is geometric: the bridge is narrowed from 1.40 m to **1.10 m** (a normal indoor flight), so after 0.30 inflation plus 0.175 half body only 2-3 cells are left and A* has no edge to hug. Re-measured: y becomes 7.28 -> 6.67 -> 7.07 -> 7.28, and 7.28 is the band centre (7.30 in world), i.e. the climb and the deck are back on the centre line.

## Limits worth knowing

- 9,891 standable cells (one storey) and 5,940 (four storey); the inflation band costs 2-4% of the plate, not the old version's striped floor.
- The probe shares the planner's edge rules, so it proves "reachable under these rules", not that the dog can walk it: `stair_gait`, the 0.42 m body, and the site upstands are not modelled.
- The thin strips in the view are the model's own counter tops, desk tops and stair treads - real horizontal faces, not breaks.
