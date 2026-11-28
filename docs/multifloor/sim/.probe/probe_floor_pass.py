#!/usr/bin/env python3
"""Frame-by-frame around the floor autoobs post: interval-rule window vs the
dog's actual offset, to find where the lateral answer lets the body graze."""
import json
from repro_penetration import session

TRACE = ('(function(){var d=__MESH.dog; if(!d) return "{}";'
         'var fw=__MESH.flatWin(__MESH.prog);'
         'return JSON.stringify({dog:d, prog:+__MESH.prog.toFixed(2),'
         ' blocked:__MESH.blocked, lat:+__MESH.state.latOff.toFixed(3),'
         ' tgt:+__MESH.state.latTgt.toFixed(3),'
         ' win:+(__MESH.state.latWin||0).toFixed(3),'
         ' fblk:fw.f.blocked, ftgt:+fw.f.tgt.toFixed(3),'
         ' cl:__MESH.state.sensorObs.map(o=>[+o.x.toFixed(2),+o.y.toFixed(2),+o.r.toFixed(2)])});})()')

_, s, _ = session('single,floor,sensor,autoobs', 26, 'JSON.stringify({})', TRACE)
post = [4.57, 3.01]
n = 0
for q in s:
    if not q.get('dog'):
        continue
    d = ((q['dog'][0]-post[0])**2 + (q['dog'][1]-post[1])**2)**0.5 - 0.3
    if d < 0.5:
        print('prog %.2f gap %.3f lat %+.2f tgt %+.2f win %.2f blk %s fblk %s ftgt %+.2f cl %s'
              % (q['prog'], d, q['lat'], q['tgt'], q['win'], q['blocked'],
                 q['fblk'], q['ftgt'], q['cl']))
        n += 1
print('frames near post:', n, 'of', len(s))