#!/usr/bin/env python3
"""probe_dog_model: the dog must show the vendor shell, not the stand-in.

The models are baked into the page by make_robot_models.py + build.py, so the
body has to come up as 'urdf' with no network at all. Asserts the baked model
builds (triangle count, trot joints, grounded bbox) for both 机型 and that the
live dog reports from === 'urdf'.
"""
import json
from repro_penetration import session

CHECK = ('JSON.stringify({dog:__MESH.dogBody, go2:__MESH.dbgRobot("go2"),'
         ' l1:__MESH.dbgRobot("l1"), say:document.getElementById("say").textContent})')

SWITCH = ('(function(){var s=document.getElementById("robotModel");'
          's.value="l1";s.onchange({target:s});'
          'return JSON.stringify({dog:__MESH.dogBody});})()')

setup, _, _ = session('single,nosave', 1, CHECK, CHECK)
d = json.loads(setup)
print('live dog  :', d['dog'])
print('baked go2 :', d['go2'])
print('baked l1  :', d['l1'])

ok = 0
tot = 0


def check(name, cond, extra=''):
    global ok, tot
    tot += 1
    ok += 1 if cond else 0
    print('  %-46s %s %s' % (name, 'PASS' if cond else 'FAIL', extra))


g, l, dog = d['go2'], d['l1'], d['dog']
check('go2 baked, nothing left to fetch', g.get('pending') == 0 and g.get('meshes', 0) > 0, g)
check('l1 baked, nothing left to fetch', l.get('pending') == 0 and l.get('meshes', 0) > 0, l)
check('go2 rig animates all four legs', g.get('rigLegs') == 8, g.get('rigLegs'))
check('l1 rig animates all four legs', l.get('rigLegs') == 8, l.get('rigLegs'))
check('go2 grounded (feet at z=0)', abs(g.get('z', [9])[0]) < 0.002, g.get('z'))
check('l1 grounded (feet at z=0)', abs(l.get('z', [9])[0]) < 0.002, l.get('z'))
check('go2 size is a dog, not the raw bbox',
      g.get('size') and 0.5 < g['size'][0] < 0.95 and 0.25 < g['size'][2] < 0.6, g.get('size'))
check('l1 size is a dog, not the raw bbox',
      l.get('size') and 0.45 < l['size'][0] < 0.95 and 0.3 < l['size'][2] < 0.7, l.get('size'))
check('live dog is the urdf body', dog and dog['from'] == 'urdf', dog)
check('live dog has a trot rig', dog and dog['legs'] == 8, dog)

setup2, _, _ = session('single,nosave', 1, SWITCH, SWITCH)
d2 = json.loads(setup2)
print('after switch:', d2)
check('switching to l1 installs the l1 body',
      d2['dog']['from'] == 'urdf' and d2['dog']['kind'] == 'l1', d2['dog'])

print('\n%d/%d passed' % (ok, tot))
raise SystemExit(0 if ok == tot else 1)
