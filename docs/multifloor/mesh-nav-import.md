# Imported mesh · global planning (`mesh_nav.html`)

A standalone page: load any OBJ / STL - multi storey stairs included - let the page work out which faces are floor and which are wall, then run a 2.5D A* over what is walkable.

## What works

| Capability | State |
|---|---|
| OBJ / STL import (ASCII and binary) | works |
| Faces split into horizontal / vertical by normal | works |
| Per column floor heights, wall projection and inflation field | works |
| Walkable surface (yellow) and inflated wall (red) | works |
| Click the model to set start / goal | works |
| Resolution and inflation selectable | works |
| 2.5D A* across storeys | **does not work yet** |

## Rules

- Horizontal (`|nz| > 0.65`): this column has a floor at this height. A plate reports both its top and bottom; samples within 0.10 m are one plate and the top is kept.
- Vertical (`|nz| < 0.35` **and z span > 0.35 m**): a wall. A 0.19 m riser is not a wall, or it inflates away the flight it belongs to.
- Inflation: walls projected to XY, distance field spreading only into columns with no floor - a landing with walls on three sides is still somewhere you can stand.
- Neighbouring columns connect only if their heights differ by 0.40 m or less. One step (0.19 m) passes, one storey (3.42 m) does not.

## Three bugs already fixed on the sample

1. **Risers inflated as walls.** A 0.19 m vertical face inflated 0.45 m sealed the whole flight. Fixed with the z span threshold.
2. **Merge threshold 0.20 m larger than the 0.19 m riser.** Two steps fused into one, the jump became 0.38 m and the flight became unclimbable. Lowered to 0.10 m.
3. **No edge between the two heights of one column.** A plate's top and bottom were two heights, and a start point snapped to the bottom could never reach the surface under its own feet. Added a vertical edge within the column.

## Where it is stuck

On the four storey sample (`example_four_floor.obj`, 293 boxes, 3,516 faces, 3 flights), start on 1F and goal on 4F, A* exhausts about 3,700 nodes and reports no path.

Confirmed:

- The stair is read correctly: 516 columns in the well, a tread column reads `[0, 0.19, 1.09, 3.42, 3.61, 4.49, 6.84, 7.03, 7.89]`, 0.19 m per step.
- Floor heights are right: `[-0.1, 0, 3.3, 3.4, 6.7, 6.8, 10.2]`, four storeys of 3.4 m.
- With inflation at zero, `blocked` is 0, so inflation is not the cause.

Not confirmed: whether the start storey's walkable columns actually join up with the stair foot, and whether the within-column vertical edge is really participating in the search - the node count did not change when it was added, which is itself suspicious.

## Next

- Prove the within-column edge with a minimal case, so it is clear it reaches the search rather than being short-circuited.
- If connectivity is fine, replace the flat 2.5D search with a storey-partitioned graph that uses the stairs as the only inter-storey edges.
