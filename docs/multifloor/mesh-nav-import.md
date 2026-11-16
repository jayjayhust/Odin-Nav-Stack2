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

## What still does not work

On the four storey sample, start on 1F and goal on 4F, A* exhausts 5,148 nodes and reports no path.

Confirmed: the standable surface is continuous (about 600 cells per storey), stair treads are recognised, and inflation at zero is not the cause.

Unknown: whether the 4F standable stratum joins the top of the flight. Next step is to cut that segment out and check the gap - `max_step_z=3` at 0.15 m is 0.45 m, so half a cell of misalignment between the flight top and the plate is enough to break it.

## Fallback

If that segment never joins in a flat grid, partition by storey and use the stairs as the only inter-storey edge - which is what `floor_manager` in the repo already does, and far cheaper than searching a flat grid.
