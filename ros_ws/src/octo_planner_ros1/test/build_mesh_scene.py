#!/usr/bin/env python3
"""Turn the simulated sweeps into surface meshes.

The cloud is what the sensor returns: a sparse shell with holes wherever the
room was never in view. Voxelising it, closing one-cell gaps and keeping only
the exposed faces gives a solid you can orbit and push a transparency slider
through - the same information as the cloud, but as surfaces.

Output is coarse integers in voxel units; the viewer multiplies by VOX.
"""
import base64, json, os, sys
import numpy as np
from scipy.ndimage import binary_closing

VOX = 0.12
# Rise per step in the survey stairs; only used to give the recovered tread
# the riser underneath it.
STEP_RISE = 0.19
SRC = "../../../../docs/multifloor/sim/nav_sim_data.json"
DST = "../../../../docs/multifloor/sim/mesh_scene_data.json"
STEPS = (1.0,)

NEIGH = ((1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1))
# each face: corner offsets in (i,j,k), wound CCW seen from outside
FACES = {
 (1,0,0): ((1,0,0),(1,1,0),(1,1,1),(1,0,1)),
 (-1,0,0): ((0,0,0),(0,0,1),(0,1,1),(0,1,0)),
 (0,1,0): ((0,1,0),(0,1,1),(1,1,1),(1,1,0)),
 (0,-1,0): ((0,0,0),(1,0,0),(1,0,1),(0,0,1)),
 (0,0,1): ((0,0,1),(1,0,1),(1,1,1),(0,1,1)),
 (0,0,-1): ((0,0,0),(0,1,0),(1,1,0),(1,0,0)),
}

def base_name(n):
    for suf in (" · NeuPAN", " · DWA"):
        if n.endswith(suf):
            return n[:-len(suf)]
    return n


def prune_floating(occ):
    """Drop short runs that float clear of the run below them in the same column.

    A sweep that grazes a balustrade returns a few cells above the tread it is
    standing on; voxelised, each becomes a fragment hanging in the air. A tread
    itself is a one- or two-cell run directly above whatever holds it up, so the
    discriminator is the gap, not the length.
    """
    nx, ny, nz = occ.shape
    for i in range(nx):
        for j in range(ny):
            col = occ[i, j]
            idx = np.flatnonzero(col)
            if idx.size < 2:
                continue
            runs = []
            a = p = int(idx[0])
            for v in idx[1:]:
                v = int(v)
                if v == p + 1:
                    p = v
                    continue
                runs.append((a, p))
                a = p = v
            runs.append((a, p))
            for n in range(1, len(runs)):
                lo, hi = runs[n]
                if hi - lo + 1 <= 2 and lo - runs[n - 1][1] >= 2:
                    col[lo:hi + 1] = False
    return occ


def fill_stair_columns(occ, lo, zones):
    """Fill empty columns inside a stair core from their neighbours.

    Standing on a tread, the next one sits above the sensor's +0.30 m window,
    so the return comes back as balustrade plus rubble and whole columns of the
    tread are missing. The lowest cell of a column is the tread the dog would
    stand on, so a hole is filled with the median of the eight neighbours that
    do have one - that keeps the flight's rake instead of flattening it.
    """
    if not zones:
        return occ
    nx, ny, nz = occ.shape
    for x0, x1, y0, y1, z0, z1 in zones:
        ix0 = max(0, int(np.floor((x0 - lo[0]) / VOX))); ix1 = min(nx, int(np.ceil((x1 - lo[0]) / VOX)) + 1)
        iy0 = max(0, int(np.floor((y0 - lo[1]) / VOX))); iy1 = min(ny, int(np.ceil((y1 - lo[1]) / VOX)) + 1)
        iz0 = max(0, int(np.floor((z0 - lo[2]) / VOX))); iz1 = min(nz, int(np.ceil((z1 - lo[2]) / VOX)) + 1)
        for i in range(ix0, ix1):
            for j in range(iy0, iy1):
                if occ[i, j, iz0:iz1].any():
                    continue
                lows = []
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        if di == 0 and dj == 0:
                            continue
                        ni, nj = i + di, j + dj
                        if not (0 <= ni < nx and 0 <= nj < ny):
                            continue
                        nb = occ[ni, nj, iz0:iz1]
                        if nb.any():
                            lows.append(int(np.flatnonzero(nb)[0]))
                if lows:
                    occ[i, j, iz0 + int(np.median(lows))] = True
    return occ


def bridge_stair_risers(occ, lo, zones):
    """Build the risers the sensor cannot see.

    A tread is a horizontal surface; the face between two treads looks straight
    down and away from the sensor standing on it, so no return ever lands there
    and the flight arrives as a staircase of detached treads. The rise is
    recoverable, though: where one column's standing cell is one to four cells
    above its neighbour's, the cells between them are the riser. Filling that
    span is not a guess - it is the only surface that can join the two treads.
    """
    if not zones:
        return occ
    nx, ny, nz = occ.shape
    for x0, x1, y0, y1, z0, z1 in zones:
        ix0 = max(0, int(np.floor((x0 - lo[0]) / VOX))); ix1 = min(nx, int(np.ceil((x1 - lo[0]) / VOX)) + 1)
        iy0 = max(0, int(np.floor((y0 - lo[1]) / VOX))); iy1 = min(ny, int(np.ceil((y1 - lo[1]) / VOX)) + 1)
        iz0 = max(0, int(np.floor((z0 - lo[2]) / VOX))); iz1 = min(nz, int(np.ceil((z1 - lo[2]) / VOX)) + 1)
        low = np.full((nx, ny), -1, np.int16)
        for i in range(ix0, ix1):
            for j in range(iy0, iy1):
                col = occ[i, j, iz0:iz1]
                if col.any():
                    low[i, j] = iz0 + int(np.flatnonzero(col)[0])
        added = 0
        for i in range(ix0, ix1):
            for j in range(iy0, iy1):
                a = low[i, j]
                if a < 0:
                    continue
                for di, dj in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                    ni, nj = i + di, j + dj
                    if not (ix0 <= ni < ix1 and iy0 <= nj < iy1):
                        continue
                    b = low[ni, nj]
                    if b <= a or b - a > 4:
                        continue
                    occ[i, j, a:b] = True
                    added += b - a
        if added:
            pass
    return occ


def stair_treads(obstacles, zones):
    """Recover the treads from the balustrade panels.

    Standing on a tread, the sensor cannot see the tread itself: it is under
    its own feet, and the face between two treads looks away. What it does see
    is the balustrade, and the simulator models one panel per step at the step's
    own height - so each panel carries the tread's x span and its level, and
    only the width has to be assumed. This is the same prior the live stack
    seeds (seed_stair_prior); it is geometry, not a return.
    """
    if not zones:
        return []
    out, groups = [], {}
    for x0, x1, y0, y1, z0, z1 in zones:
        cy = 0.5 * (y0 + y1)
        for o in obstacles:
            if o.get("k") != 0:
                continue
            ax0, ay0, ax1, ay1 = o["a"]
            bz0, bz1 = o["z"]
            # a step panel: narrow in x and y, tall in z, standing above the slab
            if ax1 - ax0 > 0.45 or ay1 - ay0 > 0.20 or bz0 < 0.06 or bz1 - bz0 < 0.70:
                continue
            if not (x0 - 0.2 <= ax0 <= x1 + 0.2 and y0 - 0.2 <= ay0 <= y1 + 0.2):
                continue
            level = bz0 + 0.04
            if not (z0 <= level <= z1 + 0.05):
                continue
            # A step often has a balustrade on both sides, and each panel would
            # otherwise yield its own tread - two blocks at the same level,
            # pointing into each other. Group by the step's x span and level
            # first, then take the union of the panels' y range: with both
            # panels that is the flight's clear width, with one it is the width
            # assumed above.
            key = (round(ax0, 3), round(level, 3))
            groups.setdefault(key, [ax1, []])[1].append((ay0, ay1))
    for (kx0, level), (kx1, ys) in groups.items():
        wy0 = min(a for a, _ in ys)
        wy1 = max(b for _, b in ys)
        if wy1 - wy0 < 0.90:
            if 0.5 * (wy0 + wy1) < cy:
                wy1 = min(y1, wy0 + 1.15)
            else:
                wy0 = max(y0, wy1 - 1.15)
        # A tread alone is a 12 cm plate on a 19 cm rise, so consecutive
        # plates leave a 7 cm slot that reads as a ramp. Carry each step down
        # to the one below it instead: the block is then the tread's top face
        # plus its riser, which is exactly the surface a flight has - and the
        # arris between two steps becomes visible.
        out.append((kx0, wy0, kx1, wy1, level - STEP_RISE, level))
    return out


def zone_slices(lo, zones, shape):
    nx, ny, nz = shape
    out = []
    for x0, x1, y0, y1, z0, z1 in zones:
        out.append((
            slice(max(0, int(np.floor((x0 - lo[0]) / VOX))), min(nx, int(np.ceil((x1 - lo[0]) / VOX)) + 1)),
            slice(max(0, int(np.floor((y0 - lo[1]) / VOX))), min(ny, int(np.ceil((y1 - lo[1]) / VOX)) + 1)),
            slice(max(0, int(np.floor((z0 - lo[2]) / VOX))), min(nz, int(np.ceil((z1 - lo[2]) / VOX)) + 1)),
        ))
    return out


def stair_panels(obstacles, zones):
    """The balustrade panels themselves, one per step per side.

    They are the only part of a flight that a sweep standing on it actually
    returns, so they stay: the panels give the flight its boundary, and each
    one also reports the level of the tread beside it.
    """
    if not zones:
        return []
    out = []
    for x0, x1, y0, y1, z0, z1 in zones:
        for o in obstacles:
            if o.get("k") != 0:
                continue
            ax0, ay0, ax1, ay1 = o["a"]
            bz0, bz1 = o["z"]
            if ax1 - ax0 > 0.45 or ay1 - ay0 > 0.20 or bz0 < 0.06 or bz1 - bz0 < 0.70:
                continue
            if not (x0 - 0.2 <= ax0 <= x1 + 0.2 and y0 - 0.2 <= ay0 <= y1 + 0.2):
                continue
            out.append((ax0, ay0, ax1, ay1, bz0, bz1))
    return out


def stair_landing(treads, zones):
    """The landing at the head of each flight, taken from the top of that flight.

    Every flight ends at the same x, so a tread sitting within half a metre of
    the core's far side is a flight head - one per side of the well, one per
    storey. Nothing is scanned on the landing itself (it is the floor the next
    flight starts from), so without this each storey loses the whole depth of
    the core between the two flights.
    """
    if not treads or not zones:
        return []
    out = []
    for x0, x1, y0, y1, z0, z1 in zones:
        for t in treads:
            if t[2] < x1 - 0.40:
                continue
            out.append((t[2], y0 + 0.12, x1 - 0.10, y1 - 0.12, t[5] - 0.12, t[5]))
    return out

def build(points, lo, hi, zones=None, treads=None, panels=None, landings=None):
    """lo/hi is the voxel-grid origin and its far corner, in world metres.

    The grid must cover every point in the scene, so the caller derives it from
    the whole cloud, not from the scenario's declared world box - otherwise
    points outside that box get clipped onto the border and fake a wall there.
    """
    nx = int(np.ceil((hi[0] - lo[0]) / VOX)) + 2
    ny = int(np.ceil((hi[1] - lo[1]) / VOX)) + 2
    nz = int(np.ceil((hi[2] - lo[2]) / VOX)) + 2
    occ = np.zeros((nx, ny, nz), bool)
    if len(points):
        idx = np.floor((points - lo) / VOX).astype(int) + 1
        idx[:,0] = np.clip(idx[:,0], 0, nx-1)
        idx[:,1] = np.clip(idx[:,1], 0, ny-1)
        idx[:,2] = np.clip(idx[:,2], 0, nz-1)
        occ[idx[:,0], idx[:,1], idx[:,2]] = True
    # Close holes per axis rather than with one 3x3x3 cube: the cube bridges
    # cells that only touch at a corner, which turns a sparse surface into a
    # rubble of blocks, and a cube costs 26 neighbours per cell against 4 here.
    # Twice, because the sweeps leave two-cell gaps between scan rings as often
    # as one, and a single pass leaves the wall looking like stacked shards.
    occ = binary_closing(occ, np.ones((3,3,1), bool))
    occ = binary_closing(occ, np.ones((1,1,3), bool))
    occ = binary_closing(occ, np.ones((3,3,1), bool))
    occ = binary_closing(occ, np.ones((1,1,3), bool))
    occ = prune_floating(occ)
    occ = fill_stair_columns(occ, lo, zones)
    occ = bridge_stair_risers(occ, lo, zones)
    # Inside a stair core the sweep contributes almost nothing usable: standing
    # on a tread, the tread is under the sensor and the riser faces away, so
    # what comes back is the balustrade plus rubble. Keeping that rubble next to
    # the recovered treads is what made the flights look broken. Clear the core
    # and put back only what is known exactly - treads, panels, landing.
    for sl in zone_slices(lo, zones or [], occ.shape):
        occ[sl] = False
    # The treads are added after the morphology, not before: the segments of a
    # flight are one cell thick and one cell apart in z, so any closing pass
    # fuses them into a single smooth ramp and the steps stop reading as steps.
    # They are the one thing here that is already exact - a flat box at the
    # level its own balustrade panel reports - so nothing needs to touch them.
    for tx0, ty0, tx1, ty1, tz0, tz1 in (treads or []) + (panels or []) + (landings or []):
        a0 = np.floor(np.array([tx0, ty0, tz0]) / VOX + 1e-9).astype(int) + 1
        a1 = np.ceil(np.array([tx1, ty1, tz1]) / VOX - 1e-9).astype(int) + 1
        sl = tuple(slice(max(0, a0[d]), min([nx, ny, nz][d] - 1, a1[d]) + 1) for d in range(3))
        occ[sl] = True
    verts, vmap, tris = [], {}, []
    for i, j, k in zip(*np.nonzero(occ)):
        for d in NEIGH:
            ni, nj, nk = i+d[0], j+d[1], k+d[2]
            if 0 <= ni < nx and 0 <= nj < ny and 0 <= nk < nz and occ[ni, nj, nk]:
                continue
            quad = []
            for off in FACES[d]:
                key = ((i+off[0]) + (j+off[1])*nx + (k+off[2])*nx*ny)
                if key not in vmap:
                    vmap[key] = len(verts)
                    verts.append((int(i+off[0]), int(j+off[1]), int(k+off[2])))
                quad.append(vmap[key])
            tris += [int(quad[0]), int(quad[1]), int(quad[2]), int(quad[0]), int(quad[2]), int(quad[3])]
    return verts, tris

def main():
    src = json.load(open(os.path.join(os.path.dirname(__file__), SRC)))
    out = {"vox": VOX, "steps": list(STEPS), "scenes": []}
    seen = {}
    for sc in src["scenarios"]:
        name = base_name(sc["name"])
        if name in seen:
            seen[name]["planners"].append(sc["name"].split(" · ")[-1] if " · " in sc["name"] else "single")
            continue
        pts = [np.array(c["p"], dtype=float) for c in sc["clouds"]]
        ts = [c["t"] for c in sc["clouds"]]
        n = len(pts)
        # One grid origin per scene, shared by every step, taken from the full
        # cloud unioned with the declared world box.
        allp = np.vstack(pts) if pts else np.zeros((0, 3))
        lo = np.minimum(allp.min(0), [sc["world"][0], sc["world"][1], float(sc["z_range"][0])])
        hi = np.maximum(allp.max(0), [sc["world"][2], sc["world"][3], float(sc["z_range"][1])])
        lo = lo - 1.5 * VOX
        hi = hi + 1.5 * VOX
        # cumulative cloud at each step of the run
        frames, cum_pts = [], 0
        for frac in STEPS:
            k = max(1, int(round(frac * n)))
            zs = sc.get("stair") or []
            tr = stair_treads(sc.get("obstacles") or [], zs)
            v, t_ = build(np.vstack(pts[:k]) if k else np.zeros((0,3)), lo, hi, zs, tr,
                          stair_panels(sc.get("obstacles") or [], zs),
                          stair_landing(tr, zs))
            cum_pts = int(sum(len(p) for p in pts[:k]))
            va = np.array(v, dtype=np.int16).reshape(-1)
            i16 = len(v) < 65535
            ia = np.array(t_, dtype=np.uint16 if i16 else np.uint32)
            frames.append({"frac": frac, "t": round(ts[min(k-1, n-1)], 2),
                           "nv": len(v), "ni": len(t_), "pts": cum_pts,
                           "v": base64.b64encode(va.tobytes()).decode(),
                           "vd": "i16", "i": base64.b64encode(ia.tobytes()).decode(),
                           "id": "u16" if i16 else "u32"})
        # Local-planner window: a 2 m disc around the dog, recorded every 0.5 s.
        # Three layers are kept apart - cells the sensor actually returned as
        # occupied, cells that are solid only because of the two-cell
        # inflation, and cells that band was never observed in at all. The
        # third matters: inflate(2) only recolours known-free cells, so the
        # shadow a wall casts stays unknown, and drawing that as empty ground
        # reads as a broken inflation ring.
        win = None
        lc = sc.get("local_costmap") or {}
        if lc.get("frames"):
            res = float(lc["res"]); r = float(lc["r"])
            side = int(np.floor(2 * r / res)) + 1
            boxes = []
            for f in lc["frames"]:
                # nav_sim ships each frame as a packed byte image already
                # (0 empty / 1 inflation / 2 occupied / 3 unseen), so it is
                # copied through rather than rebuilt from the cell lists.
                bm = np.frombuffer(base64.b64decode(f["m"]), dtype=np.uint8)
                boxes.append({"t": f["t"], "cx": f["cx"], "cy": f["cy"],
                              "m": base64.b64encode(bm.tobytes()).decode()})
            win = {"r": r, "res": res, "side": side, "frames": boxes}

        rec = {"name": name, "desc": sc["desc"],
               "org": [round(float(lo[0]), 4), round(float(lo[1]), 4), round(float(lo[2]), 4)],
               "world": [round(float(lo[0]), 3), round(float(lo[1]), 3),
                         round(float(hi[0]), 3), round(float(hi[1]), 3)],
               "z_range": [round(float(lo[2]), 3), round(float(hi[2]), 3)],
               "stair": [[round(float(v), 3) for v in z] for z in (sc.get("stair") or [])],
               "start": sc["start"], "goal": sc["goal"],
               "start_z": sc["start_z"], "goal_z": sc["goal_z"],
               "planners": [sc["name"].split(" · ")[-1] if " · " in sc["name"] else "single"],
               "trace": sc["trace"], "path": sc["global_path"], "frames": frames,
               "robot": {"half_len": 0.26, "half_wid": 0.17, "base_z": 0.32},
               "win": win}
        seen[name] = rec
        out["scenes"].append(rec)
    text = json.dumps(out, separators=(",", ":"))
    open(os.path.join(os.path.dirname(__file__), DST), "w").write(text)
    for r in out["scenes"]:
        f = r["frames"][-1]
        print("%-22s verts %7d tris %7d" % (r["name"], f["nv"], f["ni"]))
    print("total %.1f MB" % (len(text) / 1e6))

if __name__ == "__main__":
    main()
