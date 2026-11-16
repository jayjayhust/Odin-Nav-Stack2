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

def build(points, world, z0, z1):
    nx = int(np.ceil((world[2] - world[0]) / VOX)) + 2
    ny = int(np.ceil((world[3] - world[1]) / VOX)) + 2
    nz = int(np.ceil((z1 - z0) / VOX)) + 3
    occ = np.zeros((nx, ny, nz), bool)
    if len(points):
        idx = np.floor((points - np.array([world[0], world[1], z0])) / VOX).astype(int) + 1
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
        z0 = float(sc["z_range"][0]) - 0.1
        z1 = float(sc["z_range"][1]) + 0.1
        # cumulative cloud at each step of the run
        frames, cum_pts = [], 0
        for frac in STEPS:
            k = max(1, int(round(frac * n)))
            v, t_ = build(np.vstack(pts[:k]) if k else np.zeros((0,3)), sc["world"], z0, z1)
            cum_pts = int(sum(len(p) for p in pts[:k]))
            va = np.array(v, dtype=np.int16).reshape(-1)
            i16 = len(v) < 65535
            ia = np.array(t_, dtype=np.uint16 if i16 else np.uint32)
            frames.append({"frac": frac, "t": round(ts[min(k-1, n-1)], 2),
                           "nv": len(v), "ni": len(t_), "pts": cum_pts,
                           "v": base64.b64encode(va.tobytes()).decode(),
                           "vd": "i16", "i": base64.b64encode(ia.tobytes()).decode(),
                           "id": "u16" if i16 else "u32"})
        rec = {"name": name, "desc": sc["desc"], "world": sc["world"],
               "z_range": [z0, z1], "start": sc["start"], "goal": sc["goal"],
               "start_z": sc["start_z"], "goal_z": sc["goal_z"],
               "planners": [sc["name"].split(" · ")[-1] if " · " in sc["name"] else "single"],
               "trace": sc["trace"], "path": sc["global_path"], "frames": frames}
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
