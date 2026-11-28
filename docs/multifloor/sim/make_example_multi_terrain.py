#!/usr/bin/env python3
"""Build the single storey multi-terrain demo - one floor, every terrain class.

The office room sample (make_example_single_floor.py) is about obstacles; this
one is about the ground itself. It is the mesh twin of the terrain recognizer
slot's vocabulary (docs/usage/02_parameter_tuning.md 8), so a route over it
meets every class the page can ride:

  flat       three open aprons: the west one, the middle aisle, and the raised
             dock floor each lane ends on
  stair_up   four treads, 0.15 m rise / 0.45 m run, onto the dock plate
  ramp_up    a 12 deg vehicle ramp, 3.00 m run, +0.64 m
  flat dock  the raised platform the ramp lands on
  ramp_down  a second 12 deg ramp, down onto the raised floor
  stair_up   a second flight, 0.60 -> 1.20 m, at the far end of lane B

Both lanes climb from the west apron and end on a raised floor that runs on to
the east wall. That is the shape of the sample this replaces, and it is the
shape this planner draws: the page's job is to show a route over the terrain,
and the terrain is a climb. Coming back down a plateau onto an open floor is
not one of them - the descent macro snaps its foot portal onto the flight's own
stamp, so no lane asks for it.

Two rules this raster imposes on the build, both found the hard way:

  * A lone riser is closed. 0.15 m of mass sits inside the floor's own walking
    layer, the inflation band seals the cells in front of it, and a 0.45 m deep
    top face is three cells wide, which cannot pass the 3x3 cap test. A
    recognised flight is exempt - and the recogniser wants four cells of rise,
    so 0.60 m (four risers) is the shallowest lip this page can climb.
  * A ramp needs solid mass under its surface. A wedge that tapers to zero has
    no occupied cell under its low end, so the support and body-width tests see
    a patchy plane and the foot cannot be entered. Both lanes therefore ramp
    between two flat levels, which is what a loading dock looks like anyway.

Each lane is a closed aisle, fenced wall to wall, so the only way from the west
apron to the raised floor is over the terrain. Walls are 0.06 m thick and flush
to the lane edge, so the walkable strip is uninterrupted: a 2.4 m lane is 16
cells.

Z is up, metres, floor at z = 0 - the page reads OBJ with Z as height.
"""

import math
import os

# prism faces, shared by boxes and wedges: bottom, top, and the four sides,
# wound so every normal points outward (the page stamps walk surfaces only
# under an upward normal)
FACES = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
         (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))

PRISMS = []          # (verts[8], tag)


def prism(verts, tag):
    PRISMS.append((verts, tag))


def box(x0, x1, y0, y1, z0, z1, tag=""):
    """Axis-aligned box; verts 0-3 are the bottom quad, 4-7 the top quad."""
    prism(((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
           (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)), tag)


def wedge(x0, x1, y0, y1, zb, z0, z1, tag=""):
    """Prism whose top face rises linearly from z0 at x0 to z1 at x1 - a ramp,
    not a staircase - standing on a base at zb. Same vertex order as box(), so
    the same six quads are correct and the top face still winds counter-
    clockwise seen from above."""
    prism(((x0, y0, zb), (x1, y0, zb), (x1, y1, zb), (x0, y1, zb),
           (x0, y0, z0), (x1, y0, z1), (x1, y1, z1), (x0, y1, z0)), tag)


RISE, TREAD = 0.15, 0.45     # the stock single-floor tier: 1 and 3 cells
DEG = 12.0                   # vehicle ramp pitch, ~21 %
RUN = 3.00                   # ramp run that gives that pitch
DOCK = RUN * math.tan(math.radians(DEG))            # 0.638 m
PLATE = 4 * RISE             # 0.60 m: four risers, the shallowest climbable lip
TOP = PLATE + DOCK                                  # 1.238 m
HIGH_B = PLATE + PLATE                              # 1.20 m, lane B's upper floor
W, D, H, T = 18.0, 10.0, 3.40, 0.16

# ---------------------------------------------------------------- room shell --
box(0, W, 0, D, -0.10, 0, "floor")
box(0, W, 0, T, 0, H, "wall")
box(0, W, D - T, D, 0, H, "wall")
box(0, T, T, D - T, 0, H, "wall")
box(W - T, W, T, D - T, 0, H, "wall")

# -------------------------------------------------- lane A: ramp up / dock --
AY0, AY1 = 1.00, 3.40                      # lane A spans the full width
A_ST = 3.00                                # first riser of the entry flight
PA0 = A_ST + 4 * TREAD                     # 4.80 - dock plate starts
RA0 = PA0 + 1.40                           # 6.20 - ramp foot
RA1 = RA0 + RUN                            # 9.20 - ramp top
AD1 = RA1 + 1.80                           # 11.00 - east edge of the dock
RB1 = AD1 + RUN                            # 14.00 - ramp back down
for i in range(4):
    box(A_ST + i * TREAD, A_ST + (i + 1) * TREAD, AY0, AY1, 0, RISE * (i + 1),
        "tread")
box(PA0, RA0, AY0, AY1, 0, PLATE, "plate-a")
wedge(RA0, RA1, AY0, AY1, PLATE, PLATE, TOP, "ramp-up")
box(RA1, AD1, AY0, AY1, 0, TOP, "dock")
wedge(AD1, RB1, AY0, AY1, PLATE, TOP, PLATE, "ramp-down")
box(RB1, W - T, AY0, AY1, 0, PLATE, "high-floor-a")
# guard walls: flush to the lane edges, wall to wall, so the aisle cannot be
# walked round. Tall enough that a 1.24 m rim is not a shortcut.
box(T, W - T, AY0 - 0.06, AY0, 0, TOP + 1.00, "rail-a")
box(T, W - T, AY1, AY1 + 0.06, 0, TOP + 1.00, "rail-a")

# --------------------------------------------- lane B: two flights, platform --
BY0, BY1 = 6.20, 8.60
SX = 3.00                                 # first riser
for i in range(4):
    box(SX + i * TREAD, SX + (i + 1) * TREAD, BY0, BY1, 0, RISE * (i + 1),
        "tread")
PLAT_X0 = SX + 4 * TREAD                  # 4.80
PLAT_X1 = PLAT_X0 + 3.20                  # 8.00
box(PLAT_X0, PLAT_X1, BY0, BY1, 0, PLATE, "platform")
SX2 = PLAT_X1                             # the upper flight starts off it
for i in range(4):
    box(SX2 + i * TREAD, SX2 + (i + 1) * TREAD, BY0, BY1, 0,
        PLATE + RISE * (i + 1), "tread")
UB0 = SX2 + 4 * TREAD                     # 9.80 - upper floor starts
box(UB0, W - T, BY0, BY1, 0, HIGH_B, "high-floor-b")
box(T, W - T, BY0 - 0.06, BY0, 0, HIGH_B + 1.00, "rail-b")
box(T, W - T, BY1, BY1 + 0.06, 0, HIGH_B + 1.00, "rail-b")

# --------------------------------------------------------------- middle aisle --
# Flat ground with a few things to go round - the lanes already carry every
# class this page can ride, and a raised strip here would be an obstacle whose
# top no body can reach (see the note on lone risers above).
box(5.40, 5.80, 5.60, 6.00, 0, H, "column")
box(9.80, 10.20, 5.60, 6.00, 0, H, "column")
box(13.20, 14.40, 3.60, 4.30, 0, 0.85, "pallet")
box(13.20, 14.40, 3.60, 4.30, 0.85, 0.87, "pallet-top")


# ------------------------------------------------------------------ OBJ out --
def write_obj(path):
    with open(path, "w") as fh:
        fh.write("# single storey multi-terrain sample - prisms only, "
                 "metres, Z up\n")
        fh.write("# flat / stairs / ramp up (%.1f deg) / dock / ramp down / stairs\n"
                 "# %d prisms\n" % (DEG, len(PRISMS)))
        n = 1
        for (verts, tag) in PRISMS:
            fh.write("o %s\n" % (tag.replace("-", "_") or "prism"))
            for (x, y, z) in verts:
                fh.write("v %.4f %.4f %.4f\n" % (x, y, z))
            for q in FACES:
                a, b, c, d = (q[0] + n, q[1] + n, q[2] + n, q[3] + n)
                fh.write("f %d %d %d\n" % (a, b, c))
                fh.write("f %d %d %d\n" % (a, c, d))
            n += 8


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "example_multi_terrain.obj")
    write_obj(out)
    print("%s  %d prisms  %d bytes" % (out, len(PRISMS), os.path.getsize(out)))
    print("lane A  stairs x %.2f..%.2f -> %.2f  plate x %.2f..%.2f  "
          "ramp up x %.2f..%.2f +%.3f m  dock %.2f x %.2f..%.2f  "
          "ramp down x %.2f..%.2f  raised floor x %.2f..%.2f at %.2f"
          % (A_ST, PA0, PLATE, PA0, RA0, RA0, RA1, DOCK, TOP, RA1, AD1,
             AD1, RB1, RB1, W - T, PLATE))
    print("lane B  stairs x %.2f..%.2f  platform x %.2f..%.2f at %.2f  "
          "stairs x %.2f..%.2f  upper floor x %.2f..%.2f at %.2f"
          % (SX, PLAT_X0, PLAT_X0, PLAT_X1, PLATE, SX2, UB0, UB0, W - T, HIGH_B))
