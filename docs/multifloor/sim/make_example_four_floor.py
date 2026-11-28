"""Four floors, built as pure geometry - no point cloud, no voxel soup.

The point-cloud route gave a crawling surface: the stair's own risers and the
switchback turns came out as lumps, so the planner read "obstacle" at a corner
and read nothing along the flight sides. Both are the same bug: nothing was
ever put where a railing belongs. A railing is a wall; the corner of a landing
is floor.

Everything here is an axis-aligned box, and every level sits on the planner's
0.15 m grid:

    storey  3.60 m  = 24 cells
    riser   0.15 m  = 1 cell      (Go2 limit is 0.25 m)
    tread   0.30 m  = 2 cells
    12 treads per flight, two flights per storey
    slab    0.15 m  = 1 cell

    python3 make_example_four_floor.py --out example_four_floor.obj
"""

import argparse
import json

H = 3.60                 # storey: 24 x 0.15
SLAB = 0.15              # slab hangs one cell below the floor level
W, D = 8.0, 6.0          # room footprint
WALL = 0.15

# stairwell opening (upper slabs)
SX0, SX1 = 1.60, 6.40
SY0, SY1 = 1.20, 4.70

TREAD = 0.30
NSTEP = 12
RISE = 0.15
LAND_Z = NSTEP * RISE                  # 1.80, half a storey

AY0, AY1 = 1.30, 2.40                  # flight A, 1.10 m net width
BY0, BY1 = 3.50, 4.60                  # flight B
RAIL_T = 0.06
RAIL_H = 0.95
# A stair column only counts as a stair when four of its occupied cells sit
# within a few layers - that is what exempts a flight from the 0.30 m side
# inflation. A 0.60 m thick tread gives exactly four cells; thinner and the
# column reads as plain floor and the climb is refused.
TREAD_T = 0.60

BOXES = []


def box(x0, x1, y0, y1, z0, z1, tag):
    BOXES.append((min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1),
                  min(z0, z1), max(z0, z1), tag))


# ---- slabs ---------------------------------------------------------------
def slab_with_hole(z0, z1):
    box(0, W, 0, SY0, z0, z1, "slab")
    box(0, W, SY1, D, z0, z1, "slab")
    box(0, SX0, SY0, SY1, z0, z1, "slab")
    box(SX1, W, SY0, SY1, z0, z1, "slab")


for n in range(4):
    z1 = n * H
    z0 = z1 - SLAB
    if n == 0:
        box(0, W, 0, D, z0, z1, "ground")
    else:
        slab_with_hole(z0, z1)

# ---- perimeter walls -----------------------------------------------------
top = 3 * H + SLAB
box(0, WALL, 0, D, 0, top, "wall")
box(W - WALL, W, 0, D, 0, top, "wall")
box(0, W, 0, WALL, 0, top, "wall")
box(0, W, D - WALL, D, 0, top, "wall")

# ---- flights and landing -------------------------------------------------
for n in range(3):
    base = n * H
    # flight A: east-bound, from the floor up to the landing
    for i in range(NSTEP):
        x0 = SX0 + i * TREAD
        zt = base + (i + 1) * RISE
        box(x0, x0 + TREAD, AY0, AY1, zt - TREAD_T, zt, "tread")
        box(x0, x0 + TREAD, AY0 - RAIL_T, AY0, zt, zt + RAIL_H, "rail_a")
        box(x0, x0 + TREAD, AY1, AY1 + RAIL_T, zt, zt + RAIL_H, "rail_a")
    # landing: the whole stairwell width, half a storey up
    box(SX1 - 1.20, SX1, SY0, SY1, base, base + LAND_Z, "landing")
    box(SX1, SX1 + RAIL_T, SY0, SY1, base + LAND_Z, base + LAND_Z + RAIL_H, "rail_land")
    # flight B: west-bound, from the landing up to the next floor
    for i in range(NSTEP):
        x1 = SX1 - 1.20 - i * TREAD
        x0 = x1 - TREAD
        zt = base + (NSTEP + i + 1) * RISE
        box(x0, x1, BY0, BY1, zt - TREAD_T, zt, "tread")
        box(x0, x1, BY0 - RAIL_T, BY0, zt, zt + RAIL_H, "rail_b")
        box(x0, x1, BY1, BY1 + RAIL_T, zt, zt + RAIL_H, "rail_b")

# ---- guard rails round the opening on each upper floor -------------------
# The opening is a hole in the slab, so its rim gets a rail - but the west lip
# is not only rim, it is where the flights meet the floor. Flight B lands on it
# (BY0..BY1), and flight A leaves from it: its first riser is the lip itself
# (x = SX0, y = AY0..AY1). A rail across that lane walls the stairs off, which
# is why 2->3 and 3->4 had a slab of solid in front of their first step while
# 1->2 was clear - floor 1's rim carries no rail at all. The top floor has no
# flight climbing above it, so its lip is closed end to end.
for n in range(1, 4):
    z = n * H
    if n < 3:                      # flight A climbs out of this floor here
        west = [(SY0, AY0), (AY1, BY0), (BY1, SY1)]
    else:                          # top floor: nothing leaves, rail it all
        west = [(SY0, BY0), (BY1, SY1)]
    for y0, y1 in west:
        box(SX0 - RAIL_T, SX0, y0, y1, z, z + RAIL_H, "rail_hole")
    box(SX0, SX1, SY0 - RAIL_T, SY0, z, z + RAIL_H, "rail_hole")
    box(SX0, SX1, SY1, SY1 + RAIL_T, z, z + RAIL_H, "rail_hole")


# ---- lane check -----------------------------------------------------------
# Both obstructions found so far were guard rails: 0.06 m thick, invisible to
# a 0.15 m voxel map, and drawn straight across a flight's throat. The map
# planned happily through them, so nothing in the sim ever complained - only
# the opaque model showed the wall. Hence a geometry check, not a map check.

def blockers(x0, x1, y0, y1, z0, z1):
    """Boxes standing inside a probe box, ignoring what a walker stands on.

    Floor, treads and the landing legitimately fill the space below walking
    level, so they are not obstructions; rails and walls are."""
    standing = ("ground", "slab", "tread", "landing")
    out = []
    for (bx0, bx1, by0, by1, bz0, bz1, tag) in BOXES:
        if tag in standing:
            continue
        if bx1 <= x0 or bx0 >= x1 or by1 <= y0 or by0 >= y1:
            continue
        if bz1 <= z0 or bz0 >= z1:
            continue
        out.append((tag, round(bx0, 3), round(bx1, 3), round(by0, 3), round(by1, 3)))
    return out


def check_lanes():
    """Every flight must be enterable and climbable: the column over each
    tread, plus the floor strip the flight meets (flight A at the west lip of
    the opening, flight B one storey up), may hold nothing but the floor."""
    bad = []
    for n in range(3):
        base = n * H
        for i in range(NSTEP):
            # flight A climbs east out of floor n
            x0 = SX0 + i * TREAD
            zt = base + (i + 1) * RISE
            for b in blockers(x0, x0 + TREAD, AY0, AY1, zt + 0.02, zt + RAIL_H):
                bad.append(("flight A", n, i) + b)
            # flight B climbs west out of the landing to floor n+1
            x1 = SX1 - 1.20 - i * TREAD
            zt = base + (NSTEP + i + 1) * RISE
            for b in blockers(x1 - TREAD, x1, BY0, BY1, zt + 0.02, zt + RAIL_H):
                bad.append(("flight B", n, i) + b)
        # the floor strip in front of each end that meets a floor
        for b in blockers(SX0 - 0.60, SX0, AY0, AY1, base + 0.02, base + 0.60):
            bad.append(("foot of A", n, 0) + b)
        for b in blockers(SX0 - 0.60, SX0, BY0, BY1, base + H + 0.02, base + H + 0.60):
            bad.append(("head of B", n, 0) + b)
    return bad


def write_obj(path):
    verts, faces = [], []
    for (x0, x1, y0, y1, z0, z1, tag) in BOXES:
        base = len(verts)          # faces are 1-based; every box needs its own base
        verts += [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                  (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
        faces += [(base+1, base+3, base+2), (base+1, base+4, base+3),
                  (base+5, base+6, base+8), (base+6, base+7, base+8),
                  (base+1, base+2, base+6), (base+1, base+6, base+5),
                  (base+2, base+3, base+7), (base+2, base+7, base+6),
                  (base+3, base+4, base+8), (base+3, base+8, base+7),
                  (base+4, base+1, base+5), (base+4, base+5, base+8)]
    with open(path, "w") as f:
        f.write("# example_four_floor - four storeys, two flights per stairwell\n")
        for vv in verts:
            f.write("v %.4f %.4f %.4f\n" % vv)
        for fa in faces:
            f.write("f %d %d %d\n" % fa)
    return len(BOXES), len(faces)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="example_four_floor.obj")
    a = ap.parse_args()
    nb, f = write_obj(a.out)
    counts = {}
    for b in BOXES:
        counts[b[6]] = counts.get(b[6], 0) + 1
    print(json.dumps({"boxes": nb, "faces": f, "by_tag": counts}, indent=1))
    bad = check_lanes()
    if bad:
        print("BLOCKED LANES: %d" % len(bad))
        seen = set()
        for (what, storey, i, tag, bx0, bx1, by0, by1) in bad:
            k = (what, storey, tag, bx0, bx1, by0, by1)
            if k in seen:              # one line per distinct box, not per tread
                continue
            seen.add(k)
            print("  %-9s storey %d tread %2d: %-9s x %.2f..%.2f y %.2f..%.2f"
                  % (what, storey, i, tag, bx0, bx1, by0, by1))
        raise SystemExit(1)
    print("lane check: every flight entry and climb is clear")
