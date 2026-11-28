#!/usr/bin/env python3
"""Two keep-out controls added to the mesh-nav simulator.

  autoNoGo (自动计算禁行区, default ON) - toggles whether buildMap stamps the
    wall-clearance INFL band at load. Off keeps walls solid OCC but paints no
    auto band; only hand-painted keep-outs remain. Verified by counting INFL
    cells that are NOT in state.added (the pure auto band) with the box on vs
    off. m.infR and the painted footprint (keepOutBand) must be unaffected.

  clearNoGo (清禁行区) - wipes every manual edit (state.added AND state.erased)
    and rebuilds from raw geometry. Verified by painting one cell then clicking
    the button and asserting both sets are empty and the cell is no longer INFL.

The harness loads mesh_nav.html headless, waits for the model, then evaluates an
expression and returns its JSON value plus any page exceptions.
"""
import json, os, subprocess, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9396


def run(hash_, expr, wait=6.0):
    url = 'file://%s/mesh_nav.html#%s' % (BASE, hash_)
    proc = subprocess.Popen(
        ['google-chrome', '--headless=new', '--disable-gpu', '--no-sandbox',
         '--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--hide-scrollbars',
         '--disable-dev-shm-usage', '--remote-debugging-port=%d' % PORT,
         '--remote-allow-origins=*', '--window-size=1600,900', url],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        pages = []
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
        def ev(e):
            return call('Runtime.evaluate', expression=e, returnByValue=True,
                        awaitPromise=True).get('result', {}).get('value')
        time.sleep(wait)
        # A thrown/syntactically-broken expression comes back as exceptionDetails
        # on THIS response (not as a Runtime.exceptionThrown event), so without
        # this the probe would just print `result: None` with an empty exc list.
        r = call('Runtime.evaluate', expression=expr, returnByValue=True,
                 awaitPromise=True)
        d = r.get('exceptionDetails')
        if d:
            exc.append('EVAL ' + (d.get('text') or '') + ': ' +
                       str((d.get('exception') or {}).get('description', ''))[:300])
        out = (r.get('result') or {}).get('value')
        ws.close()
        return out, exc
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        time.sleep(0.5)


# Count INFL cells that are not operator-painted (the auto wall band), then flip
# the 自动禁行带 checkbox off (its onchange reloads the map) and count again.
EXPR_AUTONOGO = (
    '(function(){var m=__MESH.map,s=__MESH.state;if(!m)return JSON.stringify({err:"no map"});'
    'var cnt=function(){var n=0;for(var f=0;f<m.data.length;f++)if(m.data[f]===3&&!s.added.has(f))n++;return n;};'
    'var on=cnt();var infR=m.infR;var box=document.getElementById("autoNoGo");'
    'var wasOn=box.checked;box.checked=false;box.onchange({target:box});'
    'var m2=__MESH.map;var n2=0;for(var f=0;f<m2.data.length;f++)if(m2.data[f]===3&&!__MESH.state.added.has(f))n2++;'
    'return JSON.stringify({autoOn:on,autoOff:n2,infR_after:+m2.infR.toFixed(3),'
    '  stateInf:__MESH.state.inf,checked:box.checked});})()')

# Paint one floor cell, then click 清禁行区 and assert both manual-edit sets are
# empty and the painted cell is no longer INFL.
EXPR_CLEAR = (
    '(function(){var m=__MESH.map,s=__MESH.state;if(!m)return JSON.stringify({err:"no map"});'
    'var F=function(i,j,k){return ((k*m.ny)+j)*m.nx+i;};var found=null;'
    'for(var k=1;k<m.nz-1&&!found;k++)for(var j=1;j<m.ny-1&&!found;j++)'
    'for(var i=1;i<m.nx-1&&!found;i++)if(m.data[F(i,j,k)]===1&&m.support(i,j,k))found={i:i,j:j,k:k};'
    'if(!found)return JSON.stringify({err:"no FREE cell"});'
    'var painted=F(found.i,found.j,found.k);'
    'var ret=__MESH.blockCell(found.i,found.j,found.k);var before=s.added.size;'
    'document.getElementById("clearNoGo").onclick();'
    'var s2=__MESH.state;var m2=__MESH.map;'
    'return JSON.stringify({blockRet:ret,addedBefore:before,addedAfter:s2.added.size,'
    '  erasedAfter:s2.erased.size,stillInfl:(m2.data[painted]===3)});})()')


# Static-hug escape: with 自动禁行带 off the live map has no wall clearance, so
# replanGlobal can re-plan against the banded sibling built at load. dbgEscape
# plans start->goal over both maps and reports each route's min clearance to
# physical OCC. Assert the sibling exists, carries the auto band the live map
# lacks, plans successfully, and its route is never tighter than the band-off
# route (the roomier alternate the escape installs).
EXPR_ESCAPE = (
    '(function(){var box=document.getElementById("autoNoGo");'
    'if(box.checked){box.checked=false;box.onchange({target:box});}'
    'var s=__MESH.state,mb=__MESH.bandMap;'
    'if(!__MESH.map)return JSON.stringify({err:"no map"});if(!mb)return JSON.stringify({err:"no bandMap"});'
    's.start=[2.53,7.40,0.13];s.goal=[7.82,4.13,0.13];s.ways=[];'
    'return JSON.stringify(__MESH.dbgEscape());})()')


# Escape retries must DIVERSIFY, not re-run the same argmax: routeSafety is a
# proxy, and the route it ranks first is not guaranteed to be the one that
# works. Plan the band-off route, then fire the escape twice back to back in one
# stuck episode and require each call to install a route that differs from the
# one before it (and the attempt counter/tried-set to advance). The 2nd call
# only succeeds thanks to the relaxed retry gate + widened pool, since the
# route installed first already scores well and would otherwise block every
# candidate under "+0.05 over current".
def _sig():
    return ('(s.path||[]).map(function(q){return q.map(function(v)'
            '{return (+v).toFixed(2)}).join(",")}).join(";")')


EXPR_ROTATE = (
    '(function(){var box=document.getElementById("autoNoGo");'
    'if(box.checked){box.checked=false;box.onchange({target:box});}'
    'var s=__MESH.state;'
    'if(!s.map||!__MESH.bandMap)return JSON.stringify({err:"no map/bandMap"});'
    's.start=[2.53,7.40,0.13];s.goal=[7.82,4.13,0.13];s.ways=[];'
    'var g=document.getElementById("go");if(g){g.disabled=false;g.click();}'
    'if(__MESH.clearEscapeLog)__MESH.clearEscapeLog();'
    'var sg=function(){return ' + _sig() + '};if(!s.path)return JSON.stringify({err:"no plan"});'
    'var p0=sg();'
    'var r1=__MESH.replanGlobal(),n1=__MESH.escape,n1s=sg();'
    'var r2=__MESH.replanGlobal(),n2=__MESH.escape,n2s=sg();'
    'var L=(__MESH.escapeLog&&__MESH.escapeLog())||[];'
    'var cnt=function(c){return L.filter(function(x){return x.cls===c;}).length;};'
    'var cand=L.filter(function(x){return x.cls&&x.cls.indexOf("cand")===0;}).length;'
    'return JSON.stringify({bandOff:s.autoNoGo===false,r1:r1,r2:r2,'
    'diff1:n1s!==p0,diff2:n2s!==n1s,try1:n1.n,try2:n2.n,tried2:n2.tried.length,'
    'logHdr:cnt("hdr"),logOk:cnt("ok"),logCand:cand});})()')


def main():
    ok = 0
    total = 0

    total += 1
    out, exc = run('single,floor', EXPR_AUTONOGO)
    print('--- autoNoGo toggle (wall-inflation band) ---')
    print('result:', out, '| exc:', exc[:2])
    st = None
    if out and not exc:
        st = out if isinstance(out, dict) else json.loads(out)
    if not st or 'err' in st:
        print('  RESULT: FAIL')
    else:
        # band on paints a real auto band; off clears it; m.infR / state.inf kept
        passed = (st['autoOn'] > 0 and st['autoOff'] == 0 and
                  st['infR_after'] > 0 and st['stateInf'] > 0)
        print('  auto-INFL cells: on=%d off=%d | m.infR kept=%s state.inf kept=%s'
              % (st['autoOn'], st['autoOff'], st['infR_after'], st['stateInf']))
        print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
        ok += 1 if passed else 0

    total += 1
    out, exc = run('single,floor', EXPR_CLEAR)
    print('--- clearNoGo button (all manual edits) ---')
    print('result:', out, '| exc:', exc[:2])
    st = None
    if out and not exc:
        st = out if isinstance(out, dict) else json.loads(out)
    if not st or 'err' in st:
        print('  RESULT: FAIL')
    else:
        passed = (st['blockRet'] == 1 and st['addedBefore'] >= 1 and
                  st['addedAfter'] == 0 and st['erasedAfter'] == 0 and
                  not st['stillInfl'])
        print('  painted 1 cell -> after clear: added=%d erased=%d stillInfl=%s'
              % (st['addedAfter'], st['erasedAfter'], st['stillInfl']))
        print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
        ok += 1 if passed else 0

    total += 1
    out, exc = run('single,floor', EXPR_ESCAPE)
    print('--- static-hug escape (band off -> banded sibling re-plan) ---')
    print('result:', out, '| exc:', exc[:2])
    st = None
    if out and not exc:
        st = out if isinstance(out, dict) else json.loads(out)
    if not st or 'err' in st:
        print('  RESULT: FAIL')
    else:
        # sibling exists, carries the auto band the live map lacks, plans ok,
        # and its route keeps at least as much clearance as the band-off route
        passed = (st['autoNoGo'] is False and st['hasBandMap'] and
                  st['bandMapInfl'] > st['bandOffInfl'] and
                  st['offFail'] < 0 and st['onFail'] < 0 and
                  st['offClear'] is not None and st['onClear'] is not None and
                  st['onClear'] >= st['offClear'] - 1e-3)
        print('  band-off INFL=%d vs sibling INFL=%d | offClear=%s onClear=%s'
              % (st['bandOffInfl'], st['bandMapInfl'], st['offClear'], st['onClear']))
        print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
        ok += 1 if passed else 0

    total += 1
    out, exc = run('single,floor', EXPR_ROTATE)
    print('--- escape retry diversity (2nd attempt installs a DIFFERENT route) ---')
    print('result:', out, '| exc:', exc[:2])
    st = None
    if out and not exc:
        st = out if isinstance(out, dict) else json.loads(out)
    if not st or 'err' in st:
        print('  RESULT: FAIL')
    else:
        passed = (st['bandOff'] and st['r1'] and st['r2'] and
                  st['diff1'] and st['diff2'] and
                  st['try1'] == 1 and st['try2'] == 2 and st['tried2'] == 2 and
                  st['logOk'] >= 2 and st['logCand'] >= 2 and st['logHdr'] >= 2)
        print('  escape#1 installed new route=%s (try=%s) | '
              'escape#2 installed a different route=%s (try=%s, tried=%s)'
              % (st['diff1'], st['try1'], st['diff2'], st['try2'], st['tried2']))
        print('  escape log: %d attempt/pool headers, %d installs, %d candidate lines (rotation observable)'
              % (st['logHdr'], st['logOk'], st['logCand']))
        print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
        ok += 1 if passed else 0

    print('\n%d/%d passed' % (ok, total))


if __name__ == '__main__':
    main()
