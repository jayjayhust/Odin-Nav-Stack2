#!/usr/bin/env python3
"""Static scan check: single-shot Odin1 scan + pc_to_scan + fresh grid from
route poses (floor is level, so pitch 0). Viewpoint-limited by design: one
scan sees one arc, and returns closer than pc_to_scan range_min (0.2 m) are
dropped like on the real stack, so a lone scan can under-fit or see nothing
at very close range. The live layer accumulates arcs in the sliding_map grid
across viewpoints and tracks the fits over time - see verify_sense_trace.py
for the accumulated cluster ([4.57,3.01] r~0.30). On stairs the scan door
also takes a 5th pitch argument (rigid mount tilts with the route grade)."""
import json
from verify_sense_trace import session

EXPR = ('JSON.stringify([0,0.5,1.0,1.5,2.0].map(s=>{const a=__MESH.pathAt(s),'
        ' b=__MESH.pathAt(s+0.3); const yaw=Math.atan2(b[1]-a[1],b[0]-a[0]);'
        ' const r=__MESH.scan(a[0],a[1],a[2],yaw);'
        ' return {s, pts:r.pts.length, dyn:r.pts.filter(p=>p.dyn).length,'
        ' obs:r.obs.map(o=>[+o.x.toFixed(2),+o.y.toFixed(2),+o.r.toFixed(2)])};}))')

s = session('single,floor,sensor,autoobs', 2, EXPR)
# sample once is enough: static scans do not depend on the walking demo
for q in s[:1]:
    for row in q:
        print(json.dumps(row))
print('truth: post [4.57, 3.01] r=0.30')