#!/usr/bin/env python3
"""The escape must be able to widen its attempt range on a multi-corner floor.

Four-storey sample, the endpoints of the reported stall (start 0.81,2.13,0.08 ->
goal 7.01,1.49,10.8). The top floor's route hugs the opening's rail band three
times in a row - leaving the stair head, the west corner, the goal corner - at
0.10..0.15 m, while the install gate wants `curD + 0.05` on the first round and
`min_candidate_clearance_m` (0.175) after that. A single-berth candidate can only
move one of the three, so before the multi-via producers existed every round
ended in `无可安装候选` and the dog sat there.

The guard is therefore behavioural, not numeric: at each stall station the pool
must install SOMETHING that beats the route the dog is on, and the installed
route must have a strictly larger clearance. Station 42.8 is the robot standing
inside the west corner - the first 0.35 m of any candidate is that corner, so
round 0 legitimately cannot win there; a retry (round 1) must.
"""
import json, os, re, sys
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
  return JSON.stringify({len: +M.pathLen.toFixed(2), pts: M.path.length});
})()
'''.replace('__START__', json.dumps(START)).replace('__GOAL__', json.dumps(GOAL))

# One escape burst parked at arc s in attempt-round r, with the pool log.
BURST = r'''
(async () => {
  const F = () => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
  const M = window.__MESH;
  M.letgo(); M.replanRoute();
  M.state.escape.try = __ROUND__;
  M.hold(__S__);
  await F(); await F(); await F();
  M.clearEscapeLog();
  const before = M.pathLen, prog0 = M.prog;
  const ok = M.replanGlobal();
  await F();
  return JSON.stringify({s: __S__, round: __ROUND__, ok, prog: +prog0.toFixed(2),
                         before: +before.toFixed(2), after: +M.pathLen.toFixed(2),
                         esc: M.escape, log: M.escapeLog()});
})()
'''

# (station, round the escape would reach there, must install?)
CASES = [(40.0, 0, True), (42.8, 1, True), (45.6, 0, True), (47.2, 0, True)]


def _gain(log):
    for ln in log:
        if not isinstance(ln, dict) or ln.get('cls') != 'ok':
            continue
        m = re.search(r'净空 ([\d.]+) → ([\d.]+)', ln.get('text') or '')
        if m:
            return float(m.group(1)), float(m.group(2))
    return None, None


def run():
    steps = [(SETUP, 0.5, None)]
    for s, r, _ in CASES:
        steps.append((BURST.replace('__S__', '%s' % s).replace('__ROUND__', '%d' % r), 0.4, None))
    res, exc = session('#demo', steps)
    boot = res[0] if isinstance(res[0], dict) else json.loads(res[0])
    print('route %s | exc %s' % (boot, exc[:1]))
    ok = 0
    if not boot or boot.get('len', 0) < 40:
        print('  RESULT: FAIL (route did not plan)')
        return
    ok += 1
    for (s, r, must), raw in zip(CASES, res[1:]):
        d = raw if isinstance(raw, dict) else json.loads(raw)
        lo, hi = _gain(d['log'])
        installed = bool(d['ok'])
        good = installed if must else True
        cleared = (hi is None) or (lo is None) or hi > lo
        print('\n[s=%.1f round %d] ok=%s  route %.2f -> %.2f  gains %s -> %s'
              % (s, r, installed, d['before'], d['after'], lo, hi))
        for ln in d['log']:
            txt = ln.get('text') if isinstance(ln, dict) else str(ln)
            if txt and ('安装' in str(txt) or '无可安装' in str(txt)):
                print('    ' + str(txt))
        if good and cleared:
            ok += 1
        else:
            print('    RESULT: FAIL')
    print('\n%d/%d passed' % (ok, len(CASES) + 1))


if __name__ == '__main__':
    run()
