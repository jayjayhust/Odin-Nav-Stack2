#!/usr/bin/env python3
"""Do the feet stay ON the ground the dog is ridden over?

The dog's origin sits where its feet touch the surface, and its feet reach
0.077 m below that. "The dog walks below the terrain" therefore means the drawn
body ends up below the stamped surface under it - or, worse, inside solid
voxels, which is also what the local collision band reads as an obstacle.

The route's own z is not the answer: a flight spine point is stamped at the tread
TOP, and a graded chord used to cut into the ramp. What matters is where the page
actually draws the dog, so this probe drives `prog` from outside (__MESH.hold),
lets a frame run, and measures the drawn body.

Reported per station: `sink` = stamped surface under the body - (z - FOOT), i.e.
how far the feet are below that surface, plus the occupancy of the body cell.
PASS = no station sinks more than SINK_MAX (one raster quantisation step of slack
for a slope top). The occupancy of the body cell is printed for context only: in a
stacked stair tower the cell above a tread is often the soffit of the flight
overhead, so a solid body cell there says nothing about the feet.

Usage: python3 .probe/probe_ground_clearance.py [-v]
"""
import os, sys, json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_scene_alpha import session   # same CDP harness

FOOT = 0.077          # foot offset below the body origin (DOG_SPEC go2)
SINK_MAX = 0.05       # slack: a slope's raster top sits up to a step above the face

SWEEP = r'''
(async () => {
  const W = ms => new Promise(r => setTimeout(r, ms));
  const raf = () => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
  %s
  let m = null;
  for (let i = 0; i < 120 && !(m = window.__MESH.map); ++i) await W(100);
  if (!m) return JSON.stringify({err: 'no map'});
  const s = window.__MESH.state;
  %s
  const go = document.getElementById('go');
  go.disabled = false;
  go.click();
  await W(600);
  if (!s.path || !s.path.length)
    return JSON.stringify({err: 'no plan', say: document.getElementById('say').textContent});
  const M = window.__MESH, res = m.res;
  const F = (i, j, k) => (k*m.ny + j)*m.nx + i;
  const L = M.pathLen;
  // the stamped surface at or below the drawn body: cell k is stamped => the
  // surface is the top of that cell, lo + (k+1)*res. Scan from the body's own
  // layer downwards only - a stamp above the feet is a ceiling or the next
  // riser, and reading it as "the ground" invented a failure on every flight.
  const under = q => {
    const i = Math.floor((q[0]-m.lo[0])/res), j = Math.floor((q[1]-m.lo[1])/res);
    if (i < 0 || j < 0 || i >= m.nx || j >= m.ny) return null;
    const k0 = Math.floor((q[2]-m.lo[2])/res);
    for (let k = k0; k >= 1 && k >= k0-3; --k){
      if (m.topFace[F(i, j, k)] !== 1) continue;
      return {k, top: +(m.lo[2] + (k+1)*res).toFixed(3),
              occ: m.data[F(i, j, Math.max(0, Math.min(m.nz-1, k0)))]};
    }
    return null;
  };
  const rows = [];
  const step = Math.max(0.10, res*0.7);
  for (let v = 0; v <= L + 1e-9; v += step){
    M.hold(Math.min(L, v));
    await raf();
    const dg = M.dogExact, p = M.pathAt(Math.min(L, v));
    const u = dg ? under(dg) : null;
    const c = M.terrainAt(p[0], p[1], p[2], v);
    rows.push({v: +v.toFixed(2), x: +p[0].toFixed(2), rz: +p[2].toFixed(3),
               dz: dg ? +dg[2].toFixed(3) : null,
               top: u ? u.top : null, occ: u ? u.occ : null,
               sink: u && dg ? +(u.top - (dg[2]-FOOTV)).toFixed(3) : null,
               cls: c ? c.text : ''});
  }
  M.hold(0); window.recHold = false;
  return JSON.stringify({len: +L.toFixed(2), pts: s.path.length, rows: rows});
})()

'''


def sweep(click, boot_note, label, hash_):
    """Run one scene; returns (label, worst_sink, solid_frames, rows)."""
    expr = (SWEEP % (click, boot_note)).replace('FOOTV', repr(FOOT))
    res, exc = session(hash_, [(expr, 1.0, None)])
    if exc:
        print('  exceptions:', json.dumps(exc, ensure_ascii=False)[:800])
    d = json.loads(res[0]) if res and res[0] else {}
    if 'err' in d:
        print('%s: %s %s' % (label, d['err'], d.get('say', '')))
        return label, 9.9, 99, []
    rows = d['rows']
    rows = [r for r in rows if r['sink'] is not None]
    worst = max(rows, key=lambda r: r['sink']) if rows else None
    solid = [r for r in rows if r['occ'] == 2]
    print('--- %s ---' % label)
    print('  path %.2f m / %d pts | stations %d | sink > %.2f: %d | body in solid: %d'
          % (d['len'], d['pts'], len(rows),
             SINK_MAX, len([r for r in rows if r['sink'] > SINK_MAX]), len(solid)))
    if worst:
        print('  worst: v=%.2f x=%.2f route_z=%.3f drawn_z=%.3f surface=%.3f '
              'sink=%+.3f occ=%s %s'
              % (worst['v'], worst['x'], worst['rz'], worst['dz'], worst['top'],
                 worst['sink'], worst['occ'], worst['cls']))
    for r in rows:
        if r['sink'] > SINK_MAX or r['occ'] == 2:
            print('    v=%5.2f x=%5.2f route_z=%6.3f drawn_z=%6.3f surface=%6.3f '
                  'sink=%+.3f occ=%s %s'
                  % (r['v'], r['x'], r['rz'], r['dz'], r['top'], r['sink'],
                     r['occ'], r['cls']))
    return label, (worst['sink'] if worst else 9.9), len(solid), rows


def main():
    lane = sweep(
        "const b = [...document.querySelectorAll('button')].find(x => "
        "/单层多地形/.test(x.textContent)); (b || document.getElementById('demo2')).click();",
        "s.start = [2.00, 2.20, 0.20]; s.goal = [15.50, 2.20, 0.65];",
        'lane A · 单层多地形', '#none')
    # control: #demo plans the four-storey route on boot, so no button click
    ctrl = sweep('/* booted with the demo plan */', '', 'four-storey · control',
                 '#demo,close')

    bad = [(lbl, sink, solid) for lbl, sink, solid, _ in (lane, ctrl)
           if sink > SINK_MAX]
    if bad:
        print('RESULT: FAIL %s' % (bad,))
        return 1
    print('RESULT: ok [worst sink %+.3f (%s), %+.3f (%s)]'
          % (lane[1], lane[0], ctrl[1], ctrl[0]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
