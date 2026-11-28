#!/usr/bin/env python3
"""Does the dog actually WALK lane A of the single-storey multi-terrain sample?

probe_ground_clearance.py measures geometry (is a route point below the surface);
this one measures behaviour: with the page ticking, does `prog` advance the whole
route, or does the dog park on a grade because the local layer reads the slope
itself as an obstacle?

The symptom this guards against: the ground-run chord shortcut used to cut the
12 deg ramp, which both buried the body in the wedge and flattened the route's
apparent grade below isStair()'s 8% hand-off - so the local layer ran on the ramp
as if it were flat floor, saw the ramp's own cells inside the body band, and set
blocked for the rest of the run.

Reports: reach (max prog / pathLen), the longest unbroken blocked spell, and the
stations walked through while blocked. PASS = the route is completed.

Usage: python3 .probe/probe_lane_walk.py [-v]
"""
import os, sys, json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_scene_alpha import session   # same CDP harness

WALK = r'''
(async () => {
  const W = ms => new Promise(r => setTimeout(r, ms));
  const b = [...document.querySelectorAll('button')].find(x => /单层多地形/.test(x.textContent));
  (b || document.getElementById('demo2')).click();
  let m = null;
  for (let i = 0; i < 120 && !(m = window.__MESH.map); ++i) await W(100);
  if (!m) return JSON.stringify({err: 'no map'});
  const s = window.__MESH.state, M = window.__MESH;
  s.start = [2.00, 2.20, 0.20]; s.goal = [15.50, 2.20, 0.65];
  const go = document.getElementById('go'); go.disabled = false; go.click();
  await W(700);
  if (!s.path || !s.path.length)
    return JSON.stringify({err: 'no plan', say: document.getElementById('say').textContent});
  const L = M.pathLen;
  const F = (i,j,k) => (k*m.ny + j)*m.nx + i;
  const cellOcc = q => {
    const i = Math.floor((q[0]-m.lo[0])/m.res), j = Math.floor((q[1]-m.lo[1])/m.res),
          k = Math.floor((q[2]-m.lo[2])/m.res);
    if (i < 0 || j < 0 || k < 0 || i >= m.nx || j >= m.ny || k >= m.nz) return -1;
    return m.data[F(i,j,k)];
  };
  const rows = [];
  for (let t = 0; t < 90; ++t){          // 90 x 0.25 s = 22.5 s
    await W(250);
    const pz = M.pathAt(M.prog);
    const dg = M.dogExact;
    const c = M.terrainAt(pz[0], pz[1], pz[2], M.prog);
    rows.push({t: +(t*0.25).toFixed(2), prog: +M.prog.toFixed(2),
               x: +pz[0].toFixed(2), z: +pz[2].toFixed(2),
               dz: dg ? +dg[2].toFixed(3) : null, dogOcc: dg ? cellOcc(dg) : null,
               blocked: !!s.blocked, cls: c ? c.text : ''});
  }
  let maxProg = 0, spell = 0, worstSpell = 0, drawOcc = 0, solidSpell = 0, worstSolid = 0;
  const blockedAt = [], sunkAt = [];
  for (const r of rows){
    if (r.prog > maxProg) maxProg = r.prog;
    if (r.dogOcc === 2){
      ++drawOcc; ++solidSpell; if (solidSpell > worstSolid) worstSolid = solidSpell;
      sunkAt.push([r.x, r.dz]);
    } else solidSpell = 0;
    if (r.blocked){ ++spell; if (spell > worstSpell) worstSpell = spell;
                    blockedAt.push(r.x); }
    else spell = 0;
  }
  return JSON.stringify({len: +L.toFixed(2), maxProg: +maxProg.toFixed(2),
                         reach: +(maxProg/L).toFixed(3),
                         worstBlockedS: +(worstSpell*0.25).toFixed(2),
                         blockedStations: blockedAt.length,
                         drawnInSolid: drawOcc,
                         worstSolidS: +(worstSolid*0.25).toFixed(2),
                         sunk: sunkAt.slice(0, 8),
                         blockedX: blockedAt.length ? [+blockedAt[0].toFixed(2),
                                                      +blockedAt[blockedAt.length-1].toFixed(2)] : null,
                         rows: rows});
})()
'''

THRESH_REACH = 0.98
THRESH_BLOCK_S = 1.5
THRESH_SOLID_S = 1.0    # a riser toe can hold the body cell for a couple of samples


def main():
    verbose = '-v' in sys.argv
    res, exc = session('#none', [(WALK, 1.0, None)])
    if exc:
        print('EXCEPTIONS:', json.dumps(exc, ensure_ascii=False)[:1200])
    d = json.loads(res[0]) if res and res[0] else {}
    if 'err' in d:
        print('RESULT: FAIL  %s %s' % (d['err'], d.get('say', '')))
        return 2
    print('path %.2f m | reach %.1f%% (max prog %.2f) | longest blocked spell %.2f s | '
          'blocked stations %d at x %s | body in a solid cell %d frame(s), longest %.2f s'
          % (d['len'], d['reach']*100, d['maxProg'], d['worstBlockedS'],
             d['blockedStations'], d['blockedX'], d['drawnInSolid'], d['worstSolidS']))
    if verbose:
        for r in d['rows']:
            print('  t=%5.2f prog=%5.2f x=%5.2f z=%5.2f dz=%s dogocc=%s %-22s %s'
                  % (r['t'], r['prog'], r['x'], r['z'], r['dz'], r['dogOcc'], r['cls'],
                     'BLOCKED' if r['blocked'] else ''))
    # A body cell can be solid for a single sample on a flight - the next riser's
    # toe, or the soffit of the stair above - so only a sustained spell counts;
    # probe_ground_clearance.py is the exact test of the feet against the surface.
    ok = (d['reach'] >= THRESH_REACH and d['worstBlockedS'] < THRESH_BLOCK_S
          and d['worstSolidS'] < THRESH_SOLID_S)
    print('RESULT: %s [reach %.3f (>= %.2f), worst blocked %.2f s (< %.1f), '
          'longest solid spell %.2f s (< %.1f)]'
          % ('PASS' if ok else 'FAIL', d['reach'], THRESH_REACH,
             d['worstBlockedS'], THRESH_BLOCK_S, d['worstSolidS'], THRESH_SOLID_S))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
