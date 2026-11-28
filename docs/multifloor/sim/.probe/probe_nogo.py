#!/usr/bin/env python3
"""Does the LOCAL layer respect a hand-painted 禁行区 (增禁行 / INFL cell)?

The synthetic scan raycasts physical geometry, and the dense occPoints set is
OCC-only, so neither feeds an INFL keep-out patch to the local rollout. The fix
adds state.added cells to the local collision test in BOTH 合成传感 modes, gated
by the hard/soft band (mirrors global's traversable). This probe paints one FREE
cell sitting on the already-planned route, then runs the dog WITHOUT re-planning
globally (so the route still crosses the patch and the local layer alone has to
handle it) and measures the closest body-centre approach to the patch centre.

  hard + sensor ON   => never enters (min gap >= 0.20)  - the reported bug
  hard + sensor OFF  => never enters (min gap >= 0.20)  - parity across modes
  soft + sensor ON   => enters (min gap < 0.15)         - gating control

Global-planner parity (added later): the cases above let the LOCAL layer dodge
the patch while the global route still runs straight over it (NOREPLAN masks
that). REPLAN_CASES paints the cell, forces a fresh global replan, and measures
the closest the route CENTRE LINE itself comes to the patch. A painted keep-out
now behaves like a painted wall (footprint band), so the replanned route must
stay clear without any local help.
"""
import json, os, subprocess, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9393


def session(hash_, seconds, setup_expr, sample_expr):
    url = 'file://%s/mesh_nav.html#%s' % (BASE, hash_)
    proc = subprocess.Popen(
        ['google-chrome', '--headless=new', '--disable-gpu', '--no-sandbox',
         '--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--hide-scrollbars',
         '--disable-dev-shm-usage', '--remote-debugging-port=%d' % PORT,
         '--remote-allow-origins=*', '--window-size=1600,900', url],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(60):
            try:
                tgt = json.load(urllib.request.urlopen('http://127.0.0.1:%d/json' % PORT))
                pages = [t for t in tgt if t['type'] == 'page']
                if pages:
                    break
            except Exception:
                pass
            time.sleep(0.5)
        ws = websocket.create_connection(pages[0]['webSocketDebuggerUrl'], timeout=30)
        mid = [0]
        exc = []
        def call(method, **params):
            mid[0] += 1
            ws.send(json.dumps({'id': mid[0], 'method': method, 'params': params}))
            while True:
                m = json.loads(ws.recv())
                if m.get('method') == 'Runtime.exceptionThrown':
                    d = m['params']['exceptionDetails']
                    exc.append(d.get('text') or d.get('exception', {}).get('description', ''))
                if m.get('id') == mid[0]:
                    return m.get('result', {})
        call('Page.enable'); call('Runtime.enable')
        def ev(expr):
            return call('Runtime.evaluate', expression=expr, returnByValue=True,
                        awaitPromise=True).get('result', {}).get('value')
        time.sleep(5.0)
        ev('window.NOREPLAN=1')            # stop periodic/autoobs global replans
        setup = ev(setup_expr)
        samples = []
        t0 = time.time()
        while time.time() - t0 < seconds:
            s = ev(sample_expr)
            if s:
                samples.append(json.loads(s))
            time.sleep(0.2)
        ws.close()
        return setup, samples, exc
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        time.sleep(0.5)


# Paint one FREE cell that the current global route passes straight over.
# SENSE=1/0 selects 合成传感; SOFT>0 relaxes the band (control).
def setup_expr(sense, soft):
    return ('(function(){var s=__MESH.state, m=__MESH.map;'
            's.start=[2.53,7.40,0.13]; s.goal=[7.82,4.13,0.13];'
            'document.getElementById("go").click();'
            'var best=null;'
            'for(var fr=0.12; fr<=0.7 && !best; fr+=0.04){'
            '  var a=__MESH.pathAt(__MESH.pathLen*fr);'
            '  var i=Math.floor((a[0]-m.lo[0])/m.res), j=Math.floor((a[1]-m.lo[1])/m.res);'
            '  if(i<1||j<1||i>=m.nx-1||j>=m.ny-1) continue;'
            '  var kk=Math.max(1,Math.round((a[2]+0.12-m.lo[2])/m.res));'
            '  for(var k=kk;k<Math.min(m.nz,kk+5);k++){'
            '    if(m.data[(k*m.ny+j)*m.nx+i]===1){best={i:i,j:j,k:k,a:a};break;}}'
            '}'
            'if(!best) return JSON.stringify({err:"no FREE cell on route"});'
            'var ret=__MESH.blockCell(best.i,best.j,best.k);'
            'var cx=m.lo[0]+(best.i+0.5)*m.res, cy=m.lo[1]+(best.j+0.5)*m.res;'
            's.senseMode=%d; s.bandSoft=%d;'
            'if(typeof senseReset==="function") senseReset();'
            'window.PATCH=[cx,cy];'
            'return JSON.stringify({blockRet:ret, added:s.added.size, sense:s.senseMode, soft:s.bandSoft,'
            '  patchXY:[+cx.toFixed(3),+cy.toFixed(3)], routeXY:[+best.a[0].toFixed(3),+best.a[1].toFixed(3)],'
            '  patchNearRoute:+Math.hypot(cx-best.a[0],cy-best.a[1]).toFixed(3), '
            '  onRoute0:+Math.hypot(cx-window.PATCH[0],cy-window.PATCH[1]).toFixed(3)});})()'
            % (sense, soft))


# Paint one FREE cell on the route, then FORCE a fresh global replan and measure
# how close the new route CENTRE LINE gets to the patch. Proves the global
# planner clears a hand-painted keep-out on its own (footprint band), so the
# local layer is no longer the only thing standing betwen the route and the zone.
def setup_expr_replan(sense):
    return ('(function(){var s=__MESH.state, m=__MESH.map;'
            's.start=[2.53,7.40,0.13]; s.goal=[7.82,4.13,0.13];'
            'document.getElementById("go").click();'
            'var best=null;'
            'for(var fr=0.12; fr<=0.7 && !best; fr+=0.04){'
            '  var a=__MESH.pathAt(__MESH.pathLen*fr);'
            '  var i=Math.floor((a[0]-m.lo[0])/m.res), j=Math.floor((a[1]-m.lo[1])/m.res);'
            '  if(i<1||j<1||i>=m.nx-1||j>=m.ny-1) continue;'
            '  var kk=Math.max(1,Math.round((a[2]+0.12-m.lo[2])/m.res));'
            '  for(var k=kk;k<Math.min(m.nz,kk+5);k++){'
            '    if(m.data[(k*m.ny+j)*m.nx+i]===1){best={i:i,j:j,k:k,a:a};break;}}'
            '}'
            'if(!best) return JSON.stringify({err:"no FREE cell on route"});'
            'var cx=m.lo[0]+(best.i+0.5)*m.res, cy=m.lo[1]+(best.j+0.5)*m.res;'
            'var gapBefore=1e9, L0=__MESH.pathLen;'
            'for(var d=0; d<=L0; d+=0.05){var q0=__MESH.pathAt(d);'
            '  gapBefore=Math.min(gapBefore, Math.hypot(q0[0]-cx,q0[1]-cy));}'
            's.senseMode=%d; s.bandSoft=0;'
            'var ret=__MESH.blockCell(best.i,best.j,best.k);'
            'if(!__MESH.replanRoute()) return JSON.stringify({err:"replan failed"});'
            'var gap=1e9, L=__MESH.pathLen;'
            'for(var d2=0; d2<=L; d2+=0.05){var q2=__MESH.pathAt(d2);'
            '  gap=Math.min(gap, Math.hypot(q2[0]-cx,q2[1]-cy));}'
            'return JSON.stringify({blockRet:ret, added:s.added.size, sense:s.senseMode,'
            '  infR:+(s.inf||0).toFixed(2), onRoute0:+gapBefore.toFixed(3),'
            '  patchXY:[+cx.toFixed(3),+cy.toFixed(3)], routeGap:+gap.toFixed(3),'
            '  dbg:__MESH.dbgRoute(best.i,best.j,best.k), ko:__MESH.koInfo()});})()'
            % sense)


# Stair / ramp / landing placement parity: a keep-out used to be brushable only
# on gridded floor cells (FREE+support); a stair tread, ramp slope or the
# landing between two flights is voxelized as OCC carrying the surface flag, so
# blockCell rejected it and nothing appeared. This scans a stair scene for such
# a surface stamp, paints it, and checks the keep-out registered WITHOUT
# destroying the surface (data stays OCC) and that the footprint band built.
def setup_expr_surface():
    return ('(function(){var s=__MESH.state,m=__MESH.map;'
            'if(!m) return JSON.stringify({err:"no map"});'
            'var F=function(i,j,k){return ((k*m.ny)+j)*m.nx+i;};'
            'var found=null;'
            'for(var k=1;k<m.nz-1 && !found;k++)'
            ' for(var j=1;j<m.ny-1 && !found;j++)'
            '  for(var i=1;i<m.nx-1 && !found;i++){'
            '   if(m.data[F(i,j,k)]===2 && m.surfaceCell && m.surfaceCell(i,j,k)'
            '      && m.data[F(i,j,k+1)]===1){found={i:i,j:j,k:k};}}'
            'if(!found) return JSON.stringify({err:"no stair surface cell"});'
            'var before=s.added.size;'
            'var ret=__MESH.blockCell(found.i,found.j,found.k);'
            'var f=F(found.i,found.j,found.k);'
            'return JSON.stringify({blockRet:ret,addedBefore:before,addedAfter:s.added.size,'
            '  inAdded:s.added.has(f),surfaceKeptOCC:(m.data[f]===2),'
            '  cell:[found.i,found.j,found.k],ko:__MESH.koInfo()});})()')


def sample_expr():
    return ('(function(){var p=__MESH.dog; if(!p) return null;'
            'var d=Math.hypot(p[0]-window.PATCH[0],p[1]-window.PATCH[1]);'
            'return JSON.stringify({d:+d.toFixed(3),prog:+__MESH.prog.toFixed(2),'
            '  blocked:!!__MESH.blocked,dog:[+p[0].toFixed(2),+p[1].toFixed(2),+p[2].toFixed(2)]});})()')


def report(tag, setup, samples, exc):
    print('--- %s ---' % tag)
    print('setup:', setup)
    if exc:
        print('PAGE EXCEPTIONS:', exc[:3])
    if not samples:
        print('no samples'); return
    mn = min(s['d'] for s in samples)
    closest = min(samples, key=lambda s: s['d'])
    entered = any(s['d'] < 0.15 for s in samples)
    blk = any(s['blocked'] for s in samples)
    print('min body->patch-centre gap = %.3f m  | entered(<0.15)=%s  | ever blocked=%s  | frames=%d'
          % (mn, entered, blk, len(samples)))
    print('closest:', json.dumps(closest))


CASES = [
    ('hard + sensor ON  (must AVOID)', 1, 0, 'single,floor', 16, 0.20),
    ('hard + sensor OFF (must AVOID)', 0, 0, 'single,floor', 16, 0.20),
    ('soft  + sensor ON  (control: cross)', 1, 2, 'single,floor', 16, None),
]

# tag, sense, hash, sample-window, min route centre-line gap to the patch
REPLAN_CASES = [
    ('replan hard (route itself must clear)', 1, 'single,floor', 2, 0.25),
]


def main():
    ok = 0
    total = 0
    for tag, sense, soft, hash_, secs, thresh in CASES:
        total += 1
        setup, samples, exc = session(hash_, secs, setup_expr(sense, soft), sample_expr())
        report(tag, setup, samples, exc)
        if not samples or exc or (setup and 'err' in setup):
            print('  RESULT: FAIL (setup/exception)'); continue
        mn = min(s['d'] for s in samples)
        if thresh is not None:            # must avoid
            passed = mn >= thresh and not exc
            print('  RESULT: %s (min gap %.3f >= %.2f)'
                  % ('PASS' if passed else 'FAIL', mn, thresh))
        else:                             # control: must enter
            passed = mn < 0.15
            print('  RESULT: %s (entered, min gap %.3f < 0.15)'
                  % ('PASS' if passed else 'FAIL', mn))
        ok += 1 if passed else 0

    for tag, sense, hash_, secs, thresh in REPLAN_CASES:
        total += 1
        setup, samples, exc = session(hash_, secs, setup_expr_replan(sense), sample_expr())
        print('--- %s ---' % tag)
        print('setup:', setup)
        if exc:
            print('PAGE EXCEPTIONS:', exc[:3])
        if exc or not setup or 'err' in setup:
            print('  RESULT: FAIL (setup/exception)'); continue
        st = setup if isinstance(setup, dict) else json.loads(setup)
        gap, onr = st.get('routeGap'), st.get('onRoute0')
        passed = (gap is not None and onr is not None
                  and gap >= thresh and onr < 0.15)
        print('  keep-out band:', st.get('ko'))
        print('  replanned route centre-line min gap to patch = %.3f m '
              '(was %.3f m on the old route)' % (gap, onr))
        print('  RESULT: %s (routeGap %.3f >= %.2f; patch was on the route)'
              % ('PASS' if passed else 'FAIL', gap, thresh))
        ok += 1 if passed else 0

    # Stair / ramp / landing surface placement (no dog movement needed).
    total += 1
    setup, _samples, exc = session('single,stair', 1, setup_expr_surface(), sample_expr())
    print('--- stair-surface keep-out placement ---')
    print('setup:', setup)
    if exc:
        print('PAGE EXCEPTIONS:', exc[:3])
    st = None
    if setup and not exc:
        st = setup if isinstance(setup, dict) else json.loads(setup)
    if not st or 'err' in st:
        print('  RESULT: FAIL (setup/exception: %s)' % (st.get('err') if st else 'none'))
    else:
        ko = st.get('ko') or {}
        passed = (st.get('blockRet') == 1 and
                  st.get('addedAfter') == st.get('addedBefore') + 1 and
                  st.get('inAdded') and st.get('surfaceKeptOCC') and
                  (ko.get('bandCells') or 0) > 0)
        print('  painted cell %s kept OCC surface=%s, in state.added=%s, band cells=%s'
              % (st.get('cell'), st.get('surfaceKeptOCC'), st.get('inAdded'), ko.get('bandCells')))
        print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
        ok += 1 if passed else 0

    print('\n%d/%d passed' % (ok, total))


if __name__ == '__main__':
    main()
