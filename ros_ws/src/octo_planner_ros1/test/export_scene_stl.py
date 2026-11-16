#!/usr/bin/env python3
"""Export one sim scene as an STL, so the importer has something real to chew.

    python3 export_scene_stl.py --scene four_floor --out /tmp/four_floor.stl

Every static box the scenario declares becomes a solid, and the four-storey
stair model is added on top. The result is the exact geometry the simulator
plans against, which makes it the honest test case for mesh_import.py: a route
through an imported file has to climb the same three flights, or the import is
wrong.
"""
import argparse
import json
import os

import numpy as np

import build_mesh_scene as bms


def box_tris(x0, y0, x1, y1, z0, z1):
    """Corners in the order the scene stores them: (x0,y0,x1,y1,z0,z1).

    The first version took (x0,y0,z0,x1,y1,z1) while callers passed the scene
    tuple, so every exported box had its axes permuted - the floor of the two
    storey scene came out as a thin strip along one wall.
    """
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
         (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
         (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return v[np.array(f)]


def write_stl(tris, path):
    tris = np.asarray(tris, float).reshape(-1, 3, 3)
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(np.uint32(len(tris)).tobytes())
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            ln = np.linalg.norm(n)
            n = n / ln if ln > 0 else np.zeros(3)
            f.write(np.array(n, "<f4").tobytes())
            f.write(np.array(t, "<f4").tobytes())
            f.write(b"\0\0")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="four_floor")
    ap.add_argument("--out", default="/tmp/scene.stl")
    a = ap.parse_args()
    src = json.load(open(os.path.join(os.path.dirname(__file__), bms.SRC)))
    sc = [s for s in src["scenarios"] if bms.base_name(s["name"]) == a.scene][0]
    boxes = [(o["a"][0], o["a"][1], o["a"][2], o["a"][3], o["z"][0], o["z"][1])
             for o in (sc.get("obstacles") or [])
             if o.get("k") == 0 and (o.get("appear") or -1) <= 0]
    tr = bms.stair_model(sc.get("obstacles") or [], sc.get("stair") or [])
    tris = []
    for b in boxes + list(tr):
        tris.append(box_tris(*b))
    allt = np.vstack(tris)
    write_stl(allt, a.out)
    print("%-14s %d boxes + %d stair rects = %d triangles -> %s"
          % (a.scene, len(boxes), len(tr), len(allt), a.out))
    print("start %s  goal %s  stair %s"
          % (np.round(sc["start"], 2), np.round(sc["goal"], 2), sc["stair"]))


if __name__ == "__main__":
    main()
