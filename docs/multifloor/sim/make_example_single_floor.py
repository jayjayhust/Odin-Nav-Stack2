#!/usr/bin/env python3
"""Build the one storey demo room as a box model, in metres.

The four storey sample is hard to use for obstacle tests: everything sits
inside a wall of stairs and the camera has to dive through three plates to see
what you just dropped. One room is the right size for that, so it gets its own
model - floor, four walls, two columns, a desk, a partition, a barrel, an
up-going flight onto a 0.60 m platform, a down-going flight into a 0.60 m
pit, and a crossing built as eight risers up, a flat landing and eight risers
down. All of it boxes: no point cloud, no reconstruction.

Z is up. The page reads OBJ with Z as height, same as example_four_floor.obj.
"""

import os

BOXES = []          # (x0, x1, y0, y1, z0, z1, tag)


def box(x0, x1, y0, y1, z0, z1, tag=""):
    BOXES.append((min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1),
                  min(z0, z1), max(z0, z1), tag))


# ---------------------------------------------------------------- room shell --
W, D, H, T = 18.0, 10.0, 3.40, 0.16
# floor is tiled around the pit: x 8.80..10.40, y 5.00..6.60 is open
PY0, PY1, PX0, PX1 = 5.00, 6.60, 8.20, 11.40
FO = -0.10                       # floor slab is 0.10 thick below z=0
box(0, PX0, 0, D, FO, 0, "floor")
box(PX1, W, 0, D, FO, 0, "floor")
box(PX0, PX1, 0, PY0, FO, 0, "floor")
box(PX0, PX1, PY1, D, FO, 0, "floor")

# walls: keep the four sides, no ceiling so the room stays readable
box(0, W, 0, T, 0, H, "wall")
box(0, W, D - T, D, 0, H, "wall")
box(0, T, T, D - T, 0, H, "wall")
box(W - T, W, T, D - T, 0, H, "wall")

# ------------------------------------------------------- up-going platform --
import sys
RISE = float(sys.argv[sys.argv.index('--rise')+1]) if '--rise' in sys.argv else 0.15
TREAD = float(sys.argv[sys.argv.index('--tread')+1]) if '--tread' in sys.argv else 0.45
OUT = sys.argv[sys.argv.index('--out')+1] if '--out' in sys.argv else 'example_single_floor.obj'
PLAT = RISE * 4
box(0.60, 2.00, 0.60, 2.60, 0, PLAT, "platform")
box(0.60, 2.00, 0.60, 2.60, PLAT, PLAT, "platform-cap")   # the walking face
# four treads going east off the platform edge, 0.30 tread / 0.18 rise
# 0.15 rise is one cell whatever phase the grid lands on; 0.45 tread is
# three. A 0.30 riser looked aligned but the grid origin comes from the
# model's own bounds, so the phase is never guaranteed and 2 cells became 3.
for i in range(4):
    x0 = 2.00 + i * TREAD
    box(x0, x0 + TREAD, 0.80, 2.40, 0, PLAT - RISE * (i + 1), "tread-up")

# --------------------------------------------------------- down-going pit --
box(PX0, PX1, PY0, PY1, -PLAT - 0.10, -PLAT, "pit-floor")
# No coping wall around the hole. It was 0.06 m of decoration, and its top
# face sits exactly at floor level: the 3x3 stamp that makes a floor plate
# solid then lays an occupied layer across the whole pit mouth, which caps the
# descending flight at floor height and severs it from the pit floor.
box(PX1 - 0.06, PX1, PY0, PY1, -PLAT, 0, "pit-wall")
# four treads dropping west into the pit from the open edge at x 8.80
for i in range(4):
    x0 = PX0 + i * TREAD
    box(x0, x0 + TREAD, PY0 + 0.10, PY1 - 0.10, -PLAT,
        -RISE * (i + 1), "tread-down")

# ------------------------------------------------ up 8 / flat / down 8 ridge --
# A whole crossing laid out as one terrain: eight risers up, a flat landing,
# eight risers down, all on the same line so a plan has to take the climb,
# walk the landing and descend again. Solid from the floor up, like the model
# staircase it stands in for.
NX, NY0, NY1 = 8, 7.35, 8.45
RX, LAND = 4.00, 1.50
for i in range(NX):
    x0 = RX + i * TREAD
    box(x0, x0 + TREAD, NY0, NY1, 0, RISE * (i + 1), "ridge-up")
LX0 = RX + NX * TREAD
box(LX0, LX0 + LAND, NY0, NY1, 0, RISE * NX, "ridge-landing")
for i in range(NX):
    x0 = LX0 + LAND + i * TREAD
    box(x0, x0 + TREAD, NY0, NY1, 0, RISE * (NX - i - 1), "ridge-down")

# ------------------------------------------------------------- obstacles --
box(3.80, 4.20, 3.30, 3.70, 0, H, "column")
box(7.40, 7.80, 1.60, 2.00, 0, H, "column")
box(5.20, 6.40, 5.20, 6.00, 0, 0.75, "desk")
box(5.20, 6.40, 5.20, 5.60, 0.75, 0.78, "desk-top")
box(6.90, 8.50, 6.40, 6.55, 0, 1.20, "partition")
box(2.60, 3.00, 5.80, 6.20, 0, 1.00, "barrel")
box(2.60, 3.00, 5.80, 6.20, 1.00, 1.02, "barrel-lid")


# ------------------------------------------------------------------ OBJ out --
def write_obj(path):
    with open(path, "w") as fh:
        fh.write("# one storey demo room - boxes only, metres, Z up\n")
        fh.write("# %d boxes\n" % len(BOXES))
        n = 1
        for (x0, x1, y0, y1, z0, z1, tag) in BOXES:
            fh.write("o %s\n" % (tag.replace("-", "_") or "box"))
            for (x, y, z) in ((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                              (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)):
                fh.write("v %.4f %.4f %.4f\n" % (x, y, z))
            for q in ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
                      (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
                a, b, c, d = (q[0] + n, q[1] + n, q[2] + n, q[3] + n)
                fh.write("f %d %d %d\n" % (a, b, c))
                fh.write("f %d %d %d\n" % (a, c, d))
            n += 8


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    out = OUT if os.path.isabs(OUT) else os.path.join(here, OUT)
    write_obj(out)
    print("%s  %d boxes  %d bytes" % (out, len(BOXES), os.path.getsize(out)))
