# Point Cloud to Mesh Viewer

An offline page that turns the simulated LiDAR sweeps into a surface mesh and renders it with solid faces.
The point-cloud counterpart is [`sim/nav_sim.html`](sim/nav_sim.html); this page is [`sim/mesh_viewer.html`](sim/mesh_viewer.html).

## 1. Why a second view

The point-cloud page answers "which points did the LiDAR hit"; the mesh page answers "what does this floor look like".
Same data, two readings, two different questions:

| | Point cloud `nav_sim.html` | Mesh `mesh_viewer.html` |
|---|---|---|
| Geometry | per-frame points, accumulated by playhead | voxel occupancy of the accumulated points, outer surface only |
| Four-floor scale | 161 k points at the final frame | 49 k vertices / 99 k triangles |
| Readability | has volume, but walls and floor blend together, doorways are unreadable | floor, walls and openings have distinct faces, room topology is legible |
| Transparency | not applicable to points | 8%–100%, translucent lets you see inside |
| Primary use | verify scan coverage, when pedestrians appear | verify mapping quality, provide a collision reference for upper layers |

## 2. Generation pipeline

Script: [`test/build_mesh_scene.py`](../ros_ws/src/octo_planner_ros1/test/build_mesh_scene.py).
One run covers 5 scenes × 4 progress steps:

1. Read `nav_sim_data.json`, strip the ` · NeuPAN` / ` · DWA` suffix from `name`, dedupe → 5 scenes.
2. For each step, gather every sweep frame with `time ≤ frac × duration` and merge their points.
3. Voxel occupancy: `idx = floor((p − [wx0, wy0, z0]) / VOX) + 1`; a cell is occupied if it holds a single point.
4. `binary_closing(3×3×3)` fills one-cell gaps so sparse regions do not shatter into islands.
5. Six-neighbour face culling: keep only faces whose neighbouring cell is empty, dropping every interior face.
6. Vertices are merged by corner key, so adjacent faces share vertices; `computeVertexNormals()` builds normals, then vertex colours are split by `normal.y` (up = floor, down = underside, rest = wall).
7. Packing: int16 ×3 for vertices, `uint16`/`uint32` indices depending on the vertex count, both base64-inlined into the page.

Key parameters:

| Parameter | Value | Rationale |
|---|---|---|
| Voxel size `VOX` | 0.12 m | fine enough against a 0.52×0.34 m body; at 0.07 m the file grows from 7.4 MB to 68.9 MB and becomes unusable |
| Steps `STEPS` | 0.25 / 0.5 / 0.75 / 1.0 | more steps = smoother playback, but size grows linearly |
| Closing kernel | 3×3×3 ones | one cell only; larger kernels seal doorways |
| Index type | uint16 / uint32 | the four-floor scene at 49 366 vertices still fits uint16, halving index size |

## 3. Per-scene scale (100% step)

| Scene | Swept points | Vertices | Triangles | Z range (m) |
|---|---:|---:|---:|---|
| `office_loop` | 22,049 | 14,596 | 29,224 | −0.30 … 2.50 |
| `corridor_block` | 32,781 | 9,879 | 19,760 | −0.30 … 2.50 |
| `narrow_gate` | 30,868 | 17,795 | 35,688 | −0.30 … 2.50 |
| `stair_two_floor` | 44,388 | 19,572 | 39,256 | −0.30 … 4.00 |
| `four_floor` | 160,727 | 49,366 | 99,012 | −0.30 … 10.80 |

Triangles = index count / 3. `mesh_scene_data.json` is 7.4 MB; with three.js r150 (613 KB) inlined the page is 8.0 MB and opens offline.

## 4. The robot and the local planner window

Two things on the page exist to read the planner, not the building.

**The robot** is assembled from primitives rather than loaded from the URDF's DAE set —
that artwork is 20 MB to describe a 0.7 m body. Proportions follow `go2_description`:
0.70 × 0.31 × 0.40 m overall, a 0.42 × 0.20 × 0.10 m trunk sitting 0.30 m above the paw line,
four two-segment legs (0.05 m thigh, 0.035 m shin), an Odin1 box on the back and two cyan head lamps.
The paws are at z = 0, matching the `z = 0` in `trace`, so nothing is lifted.

**The local planner window** is a 2 m disc that travels with the robot:

| Layer | Colour | Meaning |
|---|---|---|
| ring | cyan | the region the planner may use, centred on the sampled pose |
| solid cells | red | cells the sensor actually returned as occupied |
| translucent cells | amber | cells that are only solid because of the two-cell inflation (`inflate(2)`, 0.20 m) |

Keeping these apart is the whole point: without it you cannot tell what is really there from what is only a safety margin.
The window is sampled every 0.5 s (`step % 25`), giving 40 / 61 / 57 / 82 / 302 frames across the five scenes.

Two implementation notes worth keeping:

- The disc sits at the **sampled centre** `(cx, cy)`, not the robot's live pose. Cells are stored as offsets
  from the window corner; mixing the two makes the pattern slide off the ring as the dog moves.
- Cell size is 0.10 m. That is the planner's real resolution, not a drawing choice.

## 5. Using the page

- **Scene**: five buttons; switching resets progress to 100%.
- **Transparency**: slider 8–100% plus three presets — solid 100 / half 45 / x-ray 15.
  Below 99% the material stops writing depth; writing it would cull back faces behind front ones and the shell would seem to vanish.
- **Progress**: 0–100% with play / reset. Reset returns to 0 and keeps only the first mesh step.
- **Camera**: left-drag orbit, wheel zoom, right-drag pan, view-reset returns to the default oblique view.
  The camera is **Z-up** (ROS convention), not three's default Y-up.
- **Robot**: a 0.52×0.34×0.30 m green box with a yellow nose cone for heading. A **see-through green dot**
  at its centre disables depth testing so the robot stays findable inside a four-floor stack.
- **Global path**: a bright cyan line, also depth-test free, so the whole cross-floor route is visible in translucent mode.
- **Local window / follow**: two toggles. Follow locks the camera onto the robot for a close look indoors.
- **Deep link**: `mesh_viewer.html#3,30` → fourth scene `stair_two_floor` at 30% opacity. Two more
  parameters set progress and framing: `#0,100,55,2.6,1` = office, solid, 55% progress, camera 2.6 m, following.

## 6. Known limitations

1. **Only swept directions have faces.** The un-scanned side is empty, so this is a shell, not a closed solid —
   it cannot be used directly as a watertight collision body.
2. **0.12 m voxels eat detail.** A 15 cm stair tread is ~1.25 cells: steps are visible but their count gets flattened.
   Stair semantics still come from the point cloud and the planner.
3. **Four steps, not per-frame.** The slider is continuous; the mesh rebuilds at four progress values only
   (each rebuild re-voxelises the full accumulated cloud).
4. **Not ground-truth geometry.** This is a surface inferred from points: roughly one voxel thicker than the drawn wall,
   with rounded corners.
5. **Translucent means no depth writes.** Pretty, but with two shells overlapping the blend order is GPU-defined,
   so some spots look brighter or darker than they should.
6. **The window is sampled at 0.5 s.** Disc position and cell pattern jump twice a second, not per frame.
7. **The window is two-dimensional.** Sampling only looks at the `base_z + 0.24 … + 0.44 m` band, so the air above
   a stair tread reads as empty.
8. **The inflation layer belongs to the DWA grid.** In the simulator NeuPAN uses a point-set hard constraint
   (< 0.22 m) instead, so the amber ring is only a reference for it, not the criterion it actually applies.

## 7. Reproducing

```bash
cd ros_ws/src/octo_planner_ros1
python3 test/build_mesh_scene.py     # → docs/multifloor/sim/mesh_scene_data.json

cd docs/multifloor/sim
python3 - <<'PY'
s = open("mesh_viewer.template.html").read()
s = s.replace("/*__THREE__*/", open("three.min.js").read())
s = s.replace("/*__DATA__*/", open("mesh_scene_data.json").read())
open("mesh_viewer.html", "w").write(s)
PY
```

Requires `numpy` and `scipy` (`scipy.ndimage.binary_closing`); no `trimesh` or `open3d`.

## 8. Possible next steps

- Export OBJ / GLB as a static collider for Isaac Sim or MuJoCo (needs a closed mesh first).
- Marching cubes instead of blocky faces for a smooth surface (bigger file; drop to two steps).
- Per-floor slicing instead of relying on transparency.
- Eight steps instead of four for smoother playback, at twice the size.
