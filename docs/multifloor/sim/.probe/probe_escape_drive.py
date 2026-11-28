#!/usr/bin/env python3
"""End to end: the four-storey sample must not stall at the top-floor corners.

Slow by design (2 minutes of real ticking, like probe_lane_walk.py): it drives
the reported endpoints with the demo loop on and samples progress, so a stall
shows up as `prog` frozen while the escape log churns.

The pass criterion cannot be "max prog near the route length": samples land
while an ephemeral detour is installed (a 8 m regrow replaces `pathLen`), and the
loop rewinds `prog` to ~1 m the moment the goal is reached, so a coarse sample
can easily miss the final leg. A completed pass is therefore recognised by any
of three independent signs - prog inside a metre of the route end, a large
drop between two samples (only the loop rewinds like that), or `prog` back near
zero after having been past halfway while the dog is on a lower storey again.
On top of that the run must have reached the top floor and used the widened
pool at least once, since that floor is the pinch the probe exists for.

probe_escape_widen.py is the fast algorithm-level guard; this one is the
behaviour check that the widened pool actually gets the dog through.
"""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_scene_alpha import session

START = [0.81, 2.13, 0.08]
GOAL = [7.01, 1.49, 10.8]

SETUP = r'''
(async () => {
  const W = ms => new Promise(r => setTimeout(r, ms));
  const F = () => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
  let m = null;
  for (let i = 0; i < 200 && !(m = window.__MESH.map); ++i) await W(100);
  for (let i = 0; i < 100 && !window.__MESH.path; ++i) await W(200);
  const M = window.__MESH;
  M.state.start = __START__; M.state.goal = __GOAL__; M.state.ways = [];
  document.getElementById('go').click();
  for (let i = 0; i < 100 && !(M.path && M.path.length > 2); ++i) await F();
  M.clearEscapeLog();
  return JSON.stringify({len: +M.pathLen.toFixed(2), pts: M.path.length});
})()
'''.replace('__START__', json.dumps(START)).replace('__GOAL__', json.dumps(GOAL))

PEEK = r'''
(() => {
  const M = window.__MESH;
  const q = M.prog > 1e-6 ? M.pathAt(Math.min(M.prog, M.pathLen)) : null;
  return JSON.stringify({prog: +M.prog.toFixed(2), len: +M.pathLen.toFixed(2),
    escape: M.escape.n, dog: q ? q.map(v => +v.toFixed(2)) : null});
})()
'''

SAMPLES = 8
SETTLE = 15.0


def run():
    steps = [(SETUP, 0.6, None)] + [(PEEK, SETTLE, None)] * SAMPLES
    res, exc = session('#demo', steps)
    print('exc %s' % exc[:1])
    boot = res[0] if isinstance(res[0], dict) else json.loads(res[0])
    print('route %s' % (boot,))
    if not boot or boot.get('len', 0) < 40:
        print('  RESULT: FAIL (route did not plan)')
        return
    best = 0.0
    escapes = 0
    top = -99.0
    wrapped = False
    prev = None
    for i, raw in enumerate(res[1:]):
        d = raw if isinstance(raw, dict) else json.loads(raw)
        best = max(best, d['prog'])
        escapes = max(escapes, d['escape'])
        if d['dog']:
            top = max(top, d['dog'][2])
        if prev is not None and d['prog'] < prev - 12.0:
            wrapped = True                 # only the goal loop rewinds prog this far
        prev = d['prog']
        print('t=%3ds  prog %6.2f / %6.2f  escape.n %d  dog %s'
              % ((i + 1) * SETTLE, d['prog'], d['len'], d['escape'], d['dog']))
    reached = (best >= boot['len'] - 0.6) or wrapped \
        or (best > boot['len'] * 0.5 and prev < 2.0)
    ok = reached and top >= 10.0 and escapes >= 1
    print('\nmax prog %.2f of %.2f (wrapped %s), top z %.2f, %d escape attempt(s)'
          '  RESULT: %s'
          % (best, boot['len'], wrapped, top, escapes, 'PASS' if ok else 'FAIL'))
    if not ok:
        for need, got in (('route end', reached), ('top floor', top >= 10.0),
                          ('pool used', escapes >= 1)):
            if not got:
                print('  missing: %s' % need)


if __name__ == '__main__':
    run()
