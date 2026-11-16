#!/usr/bin/env python3
"""Bring a mesh the user already has into the multifloor pipeline.

    python3 mesh_import.py model.stl --name my_building --start 1,0.5,0.1 --goal 5,2,8

Reads STL / OBJ / PLY (the formats a CAD or scan export lands in), samples the
surface into the same 0.12 m voxel grid the rest of the toolchain uses, plans a
route across it with a stair-aware 3D A*, and writes a scene dict that
build_mesh_scene.py merges into mesh_scene_data.json. The viewer then lists the
model next to the five built-in scenes, with the plan drawn in it.

Why it is not a scan pipeline: a mesh is already a surface, so there is nothing
to close, denoise or fill. What it does need is a rule for climbing: a flight
is a staircase in the geometry, and the route has to be allowed to gain one
riser per step without declaring the tread unwalkable. That is the whole of the
stair handling here - a step may rise by at most STEP_RISE, and a cell is free
if it is not occupied and has headroom.
"""
import argparse
import json
import os
import struct
import sys

import numpy as np

VOX = 0.12                     # same grid as the rest of the toolchain
STEP_RISE = 0.22               # one riser, with a little slack for leaning
CLEARANCE = 0.35               # headroom a body needs above a cell
MAX_GRADE = 2                  # cells of z a step may span (2 x 0.12 = 0.24)
MAX_STEP = 3                   # cells of x/y a stride may span
HALF_WIDTH = 0.30              # robot half width, used to keep off walls


def load_stl(path):
    """Binary STL, or ASCII when the file does not start with the binary header."""
    with open(path, "rb") as f:
        raw = f.read()
    if raw[:5].lower().startswith(b"solid") and b"facet" in raw[:400].lower():
        verts = []
        for line in raw.decode("ascii", "ignore").splitlines():
            parts = line.split()
            if parts and parts[0] == "vertex":
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
        return np.array(verts, float).reshape(-1, 3, 3)
    n = struct.unpack("<I", raw[80:84])[0]
    tris = np.zeros((n, 3, 3), float)
    off = 84
    dt = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    data = np.frombuffer(raw, dtype=dt, count=n, offset=off)
    tris[:, :, :] = data["v"]
    return tris


def load_obj(path):
    vs, faces = [], []
    for line in open(path, "r", errors="ignore"):
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "v":
            vs.append([float(parts[1]), float(parts[2]), float(parts[3])])
        elif parts[0] == "f":
            idx = []
            for p in parts[1:]:
                idx.append(int(p.split("/")[0]) - 1)
            for k in range(1, len(idx) - 1):     # fan triangulation
                faces.append([idx[0], idx[k], idx[k + 1]])
    v = np.array(vs, float)
    f = np.array(faces, int)
    return v[f]


def load_ply(path):
    raw = open(path, "rb").read()
    head_end = raw.index(b"end_header") + len(b"end_header\n")
    head = raw[:head_end].decode("ascii", "ignore").splitlines()
    fmt = "ascii" if "format ascii" in " ".join(head) else "binary"
    nv = next(int(l.split()[-1]) for l in head if l.startswith("element vertex"))
    nf = next(int(l.split()[-1]) for l in head if l.startswith("element face"))
    if fmt == "ascii":
        body = raw[head_end:].decode("ascii", "ignore").split()
        v = np.array(body[:nv * 3], float).reshape(-1, 3)
        rest = body[nv * 3:]
        faces, p = [], 0
        for _ in range(nf):
            k = int(rest[p]); p += 1
            faces.append([int(rest[p + m]) for m in range(k)]); p += k
        v = v[np.array(faces)]
        return np.stack([v[:, 0], v[:, 1], v[:, 2]], 1) if v.ndim == 3 else v
    raise SystemExit("binary PLY is not handled here; export ASCII PLY or STL")


def load_mesh(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".stl":
        return load_stl(path)
    if ext == ".obj":
        return load_obj(path)
    if ext == ".ply":
        return load_ply(path)
    raise SystemExit("unsupported %s - export STL, OBJ or ASCII PLY" % ext)


def _dist_to_tri(P, a, b, c):
    """Distance from points P (N,3) to triangle abc, plus inside-or-not.

    Barycentric test first: if the projection lands inside, the distance is
    the plane distance. Otherwise it is the nearest of the three edges. A
    sampled surface can miss a wide floor because it samples along the longest
    edge; this cannot, because it measures the whole box the triangle fills.
    """
    ab, ac = b - a, c - a
    n = np.cross(ab, ac)
    n = n / (np.linalg.norm(n) + 1e-12)
    d = np.abs((P - a) @ n)
    # project and solve for barycentric weights
    ap = P - a
    # drop the component along the normal before solving for barycentrics:
    # the weights are in-plane, and a point half a cell off the plane otherwise
    # reports as outside its own triangle
    ap = ap - (ap @ n)[:, None] * n
    d00 = ab @ ab; d01 = ab @ ac; d11 = ac @ ac
    d20 = ap @ ab; d21 = ap @ ac
    den = d00 * d11 - d01 * d01
    if abs(den) < 1e-12:
        return d, np.zeros(len(P), bool)
    v = (d11 * d20 - d01 * d21) / den
    w = (d00 * d21 - d01 * d20) / den
    inside = (v >= -0.02) & (w >= -0.02) & (v + w <= 1.02)
    # edge distance for the outside points
    def seg(p0, p1):
        e = p1 - p0
        t = np.clip(((P - p0) @ e) / (e @ e + 1e-12), 0, 1)
        q = p0 + t[:, None] * e
        return np.linalg.norm(P - q, axis=1)
    de = np.minimum(np.minimum(seg(a, b), seg(b, c)), seg(c, a))
    return np.where(inside, d, de), inside


def rasterise(tris, lo, shape, vox):
    """Mark every cell the surface passes through, at half a cell tolerance.

    Cell centres within 0.6 vox of a triangle are occupied. That is thick
    enough that a wall or floor has no pinholes a route could leak through, and
    thin enough that a door gap stays a gap.
    """
    occ = np.zeros(shape, bool)
    nx, ny, nz = shape
    tol = 0.6 * vox
    for t in tris:
        a, b, c = t
        mn = (np.minimum(np.minimum(a, b), c) - lo) / vox - 1
        mx = (np.maximum(np.maximum(a, b), c) - lo) / vox + 1
        i0 = max(0, int(np.floor(mn[0]))); i1 = min(nx - 1, int(np.ceil(mx[0])))
        j0 = max(0, int(np.floor(mn[1]))); j1 = min(ny - 1, int(np.ceil(mx[1])))
        k0 = max(0, int(np.floor(mn[2]))); k1 = min(nz - 1, int(np.ceil(mx[2])))
        if i1 < i0 or j1 < j0 or k1 < k0:
            continue
        I, J, K = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1),
                              np.arange(k0, k1 + 1), indexing="ij")
        P = np.stack([lo[0] + (I.ravel() + 0.5) * vox,
                      lo[1] + (J.ravel() + 0.5) * vox,
                      lo[2] + (K.ravel() + 0.5) * vox], 1)
        d, _ = _dist_to_tri(P, a, b, c)
        hit = d <= tol
        occ[I.ravel()[hit], J.ravel()[hit], K.ravel()[hit]] = True
    return occ


def standable(occ):
    """Cells a body can stand in: empty, with something solid directly below.

    Headroom is deliberately not tested. Inside a flight the clear height is
    one riser - 0.19 m - so a volume-clearance test leaves the stair entirely
    impassable while a real robot walks it every day. Contact is the right
    model: feet on a surface, next cell's surface at most one riser away.
    """
    up = np.zeros_like(occ)
    up[:, :, 1:] = occ[:, :, :-1]      # solid cell below this one
    return (~occ) & up


def astar(free, start, goal, vox):
    """26-neighbour A* whose only stair rule is a bounded rise per step."""
    import heapq
    shape = free.shape
    if not free[tuple(start)] or not free[tuple(goal)]:
        print("  start/goal not standable: start %s goal %s"
              % (bool(free[tuple(start)]), bool(free[tuple(goal)])))
        return None
    def h(p):
        d = np.array(goal) - np.array(p)
        return float(np.linalg.norm(d)) * 1.02
    start, goal = tuple(start), tuple(goal)
    g = {start: 0.0}
    came = {}
    q = [(h(start), 0.0, start)]
    seen = set()
    # horizontal neighbours only: a stair is climbed by stepping onto its
    # next tread, and allowing a horizontal step plus a rise of up to two
    # cells is exactly that. Pure vertical moves would let a route float.
    # A tread is 0.30 m deep and one riser is 0.19 m, so reaching the next tread
    # is a horizontal step of two to three cells together with a rise of one or
    # two. Restricting the horizontal to a single cell made every flight a dead
    # end - the route could stand on a tread but never reach the next one.
    nbr = [(dx, dy, dz) for dx in range(-MAX_STEP, MAX_STEP + 1)
           for dy in range(-MAX_STEP, MAX_STEP + 1)
           for dz in range(-MAX_GRADE, MAX_GRADE + 1)
           if dx * dx + dy * dy > 0]
    while q:
        f, gc, p = heapq.heappop(q)
        if p in seen:
            continue
        seen.add(p)
        if p == goal:
            path = [p]
            while p in came:
                p = came[p]
                path.append(p)
            return path[::-1]
        for d in nbr:
            n = (p[0] + d[0], p[1] + d[1], p[2] + d[2])
            if not (0 <= n[0] < shape[0] and 0 <= n[1] < shape[1] and 0 <= n[2] < shape[2]):
                continue
            if not free[n]:
                continue
            # a step may rise, but not by more than one riser
            if d[2] > 0 and d[2] * vox > STEP_RISE:
                continue
            # a stride costs its real length, so a long flat step is not free
            cost = float(np.linalg.norm([d[0], d[1], d[2]])) * 1.0
            ng = gc + cost
            if ng < g.get(n, 1e18):
                g[n] = ng
                came[n] = p
                heapq.heappush(q, (ng + h(n), ng, n))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mesh")
    ap.add_argument("--name", default=None)
    ap.add_argument("--start", default=None, help="x,y,z in model metres")
    ap.add_argument("--goal", default=None)
    ap.add_argument("--vox", type=float, default=VOX)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__),
                                                  "custom_scene.json"))
    a = ap.parse_args()

    tris = load_mesh(a.mesh)
    pt = tris.reshape(-1, 3)
    lo = pt.min(0) - 3 * a.vox
    hi = pt.max(0) + 3 * a.vox
    shape = tuple(np.ceil((hi - lo) / a.vox).astype(int) + 2)
    print("mesh   %s  %d triangles  bbox %s .. %s"
          % (os.path.basename(a.mesh), len(tris), np.round(pt.min(0), 2), np.round(pt.max(0), 2)))
    occ = rasterise(tris, lo, shape, a.vox)
    print("voxels %d  grid %s  occupied %d (%.1f%%)"
          % (np.prod(shape), shape, occ.sum(), 100 * occ.mean()))

    free = standable(occ)
    # start/goal default to the lowest and highest free cell near the bbox centre
    def pick(z_side):
        x = int((shape[0] - 1) / 2); y = int((shape[1] - 1) / 2)
        rng = range(shape[2]) if z_side > 0 else range(shape[2] - 1, -1, -1)
        for k in rng:
            if free[x, y, k]:
                return (x, y, k)
        for k in rng:
            hit = snap((x, y, k))
            if hit:
                return hit
        for k in rng:
            for dx in range(-8, 9):
                for dy in range(-8, 9):
                    i, j = x + dx, y + dy
                    if 0 <= i < shape[0] and 0 <= j < shape[1] and free[i, j, k]:
                        return (i, j, k)
        return None
    def snap(cell):
        """Nearest cell that can actually be stood in. A user picks a point on
        a floor, which lands in the half-cell under the surface."""
        i, j, k = cell
        best = None
        reach = int(np.ceil(1.5 / a.vox))   # a point can sit up to 1.5 m above the surface
        for dz in range(0, reach + 1):
            for kk in ((k + dz, k - dz) if dz else (k,)):
                if not (0 <= kk < shape[2]):
                    continue
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        ii, jj = i + dx, j + dy
                        if 0 <= ii < shape[0] and 0 <= jj < shape[1] and free[ii, jj, kk]:
                            d = dx * dx + dy * dy + dz * dz
                            if best is None or d < best[0]:
                                best = (d, (ii, jj, kk))
        return best[1] if best else None

    def to_cell(s):
        c = tuple(np.floor((np.array([float(q) for q in s.split(",")]) - lo) / a.vox).astype(int))
        return snap(c) or c
    start = to_cell(a.start) if a.start else pick(1)
    goal = to_cell(a.goal) if a.goal else pick(-1)
    if start is None or goal is None:
        raise SystemExit("no free cell found for start or goal")
    print("start %s  goal %s (cells)" % (start, goal))
    path = astar(free, start, goal, a.vox)
    if path is None:
        raise SystemExit("no route: the model is not connected at this resolution")
    world = [(lo + (np.array(p) + 0.5) * a.vox) for p in path]
    length = float(np.sum(np.linalg.norm(np.diff(np.array(world), axis=0), axis=1)))
    rise = float(world[-1][2] - world[0][2])
    print("route  %d cells  %.1f m  rise %.2f m" % (len(path), length, rise))

    # thin the polyline for display: keep every point that changes direction
    disp = [world[0]]
    for i in range(1, len(world) - 1):
        d0 = np.array(world[i]) - np.array(world[i - 1])
        d1 = np.array(world[i + 1]) - np.array(world[i])
        if np.linalg.norm(d0) > 1e-9 and np.dot(d0, d1) < 0.75 * np.linalg.norm(d0) * np.linalg.norm(d1):
            disp.append(world[i])
    disp.append(world[-1])

    sc = {"name": a.name or os.path.splitext(os.path.basename(a.mesh))[0],
          "desc": "imported from %s" % os.path.basename(a.mesh),
          "org": lo.tolist(), "vox": a.vox,
          "world": [float(pt.min(0)[0]), float(pt.min(0)[1]),
                    float(pt.max(0)[0]), float(pt.max(0)[1])],
          "z_range": [float(pt.min(0)[2]), float(pt.max(0)[2])],
          "stair": [],
          "start": [float(v) for v in world[0]], "goal": [float(v) for v in world[-1]],
          "start_z": float(world[0][2]), "goal_z": float(world[-1][2]),
          "path": [[float(v) for v in p] for p in disp],
          "trace": [[float(v) for v in p] + [0.0] for p in disp],
          "length": length, "rise": rise, "cells": len(path),
          "occ": occ}
    np.save(a.out.replace(".json", "_occ.npy"), occ)
    with open(a.out, "w") as f:
        json.dump({k: v for k, v in sc.items() if k != "occ"}, f)
    print("wrote  %s  (occupancy sidecar %s)" % (a.out, a.out.replace(".json", "_occ.npy")))


if __name__ == "__main__":
    main()
