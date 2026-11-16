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
SRC = "../../../../docs/multifloor/sim/nav_sim_data.json"
DST = "../../../../docs/multifloor/sim/mesh_scene_data.json"
STEPS = (0.25, 0.5, 0.75, 1.0)

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

def build(points, lo, hi):
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
        occ = binary_closing(occ, np.ones((3,3,3), bool))
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
            v, t_ = build(np.vstack(pts[:k]) if k else np.zeros((0,3)), lo, hi)
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
