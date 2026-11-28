#!/usr/bin/env python3
"""Terrain-aware stuck escape (P2 of docs/multifloor/escape-exploration.md).

The escape has two layers: a local stop-or-go that can reverse the dog, and a
global best-of-N route swap. On a staircase the second one used to be terrain
blind - it re-planned from whichever tread the dog happened to be on, and the
route it installed could send the dog back down the flight it was climbing.
These four cases pin the four rules that fix that, against the live page rather
than against a model of it:

  1  mid-flight, no route swap. Same dog, same post, same candidate pool; the
     only thing that changes is stair_commit_enable. With it on the escape holds
     and the drawn route is untouched; with it off the very same call installs.
     That pairing is what makes the case proof rather than a passing glance - an
     empty candidate pool would "hold" for reasons of its own.
  2  a landing is standable floor, so rotation runs there on the multi-floor
     route (a flight's portal cell plus one tread's worth of snap radius).
  3  a reversal demanded mid-flight retreats to the nearest portal, not back
     along the nose - the dog runs onto a platform the recogniser knows about.
  4  across a floor transition the stall threshold the tick times against is
     transition_dwell_ms, twice the ordinary one, and it is the profile's number
     rather than a second copy of it: a slow gait handover is not a block.

The harness is the same CDP driver as probe_autonogo.py, with a longer socket
timeout because cases 3 and 4 watch the dog move in real time.

  python3 .probe/probe_escape_terrain.py          # all four
  python3 .probe/probe_escape_terrain.py 2 4      # a subset
"""
import json, os, subprocess, sys, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9397

CHROME = ['google-chrome', '--headless=new', '--disable-gpu', '--no-sandbox',
          '--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--hide-scrollbars',
          '--disable-dev-shm-usage', '--remote-debugging-port=%d' % PORT,
          '--remote-allow-origins=*', '--window-size=1600,900']


def run(hash_, expr, wait=6.0, timeout=180):
    """Load mesh_nav.html#<hash>, evaluate <expr> (may be an async IIFE) and
    return its value plus any page exceptions."""
    url = 'file://%s/mesh_nav.html#%s' % (BASE, hash_)
    proc = subprocess.Popen(CHROME + [url], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
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
        ws = websocket.create_connection(pages[0]['webSocketDebuggerUrl'], timeout=timeout)
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
        time.sleep(wait)
        r = call('Runtime.evaluate', expression=expr, returnByValue=True, awaitPromise=True)
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


def value(out):
    if out is None:
        return None
    return out if isinstance(out, dict) else json.loads(out)


# Shared JS preamble: park the dog at an arc position and let the tick catch up,
# and a route fingerprint so "installed nothing" means the polyline is untouched
# rather than "the escape returned false for some reason we did not look at".
JS_HELPERS = (
    'function sig(){return JSON.stringify(__MESH.state.path.map(function(v){return v.map'
    '(function(z){return (+z).toFixed(2)}).join(",")}));}\n'
    'async function park(a){__MESH.hold(a);for(var t=0;t<4;t++)'
    'await new Promise(function(r){setTimeout(r,70);});}\n'
    # Scan the route for a spot whose terrain verdict is (onFlight, atLanding)
    # and that still has `ahead` metres of route in front of the dog, so a post
    # can be placed where the escape would otherwise want to move. onF < 0 means
    # the flight flag does not matter, only the landing one. 0.25 m steps because
    # the snap window around a portal is barely one tread wide.
    'async function spot(onF,atL,ahead){var L=__MESH.pathLen;'\
    'for(var a=0.2*L;a<0.92*L;a+=0.25){if(L-a<(ahead||6))continue;await park(a);'\
    'var d=__MESH.dbgLanding();'\
    'if((onF<0||d.onFlight===!!onF)&&d.atLanding===!!atL)return{prog:a,d:d};}return null;}\n')


# ---- case 1: mid-flight the escape holds, and only the gate makes it hold ----
# single,ridgedn starts on the crossing's 1.2 m landing and runs down eight
# risers, so one scene offers both a mid-flight spot and a portal spot with the
# same tight geometry - the candidate pool the escape would install from exists
# at both, which is what makes the second half of this case meaningful.
EXPR1 = ('(async function(){' + JS_HELPERS +
         'var s=__MESH.state,out={};'
         'if(!s.path||!__MESH.dog)return JSON.stringify({err:"no path/dog"});'
         'var f=await spot(1,0,1.2);if(!f)return JSON.stringify(Object.assign(out,{err:"no mid-flight spot"}));'
         'out.prog=+f.prog.toFixed(2);out.nd=f.d.nearDist;out.act=f.d.action;out.flight=f.d.flight;'
         'out.range=f.d.range;'
         'var pp=__MESH.pathAt(Math.min(__MESH.pathLen,f.prog+0.8));'
         '__MESH.drop(pp[0],pp[1],pp[2],false);'
         's.escape.try=0;s.escape.sigs.length=0;s.escape.terrain="";'
         'var p0=sig();out.installedOn=__MESH.replanGlobal();'
         'out.routeUnchanged=sig()===p0;'
         'out.terrain=__MESH.escape.terrain;out.callsOn=__MESH.escape.calls;'
         '__MESH.escapeCfg.stair_commit_enable=false;'
         's.escape.try=0;s.escape.sigs.length=0;await park(f.prog);'
         'var p1=sig();out.installedOff=__MESH.replanGlobal();'
         'out.routeChangedOff=sig()!==p1;'
         '__MESH.escapeCfg.stair_commit_enable=true;'
         'return JSON.stringify(out);})()')


# ---- case 2: the same escape runs normally beside a landing ----
EXPR2 = ('(async function(){' + JS_HELPERS +
         'var s=__MESH.state,out={};'
         'if(!s.path||!__MESH.dog)return JSON.stringify({err:"no path/dog"});'
         'var f=await spot(-1,1,1.2);if(!f)return JSON.stringify(Object.assign(out,{err:"no landing spot"}));'
         'out.prog=+f.prog.toFixed(2);out.nd=f.d.nearDist;out.snap=__MESH.escapeCfg.landing_snap_radius_m;'
         'out.onFlight=f.d.onFlight;out.flight=f.d.flight;'
         'var pp=__MESH.pathAt(Math.min(__MESH.pathLen,f.prog+0.8));'
         '__MESH.drop(pp[0],pp[1],pp[2],false);'
         's.escape.try=0;s.escape.sigs.length=0;s.escape.terrain="";'
         'var p0=sig();out.installed=__MESH.replanGlobal();'
         'out.routeChanged=sig()!==p0;out.terrain=__MESH.escape.terrain;'
         'out.dbgTerrainAction=__MESH.dbgEscape().terrain_action;'
         'var tr=(window.__ESCAPE_TRACE||[]);out.trace=tr.length?tr[tr.length-1]:null;'
         'return JSON.stringify(out);})()')


# ---- case 3: a reversal demanded mid-flight aims at the portal ----
# The dog is NOT parked here: the tick only re-solves the local layer when the
# route moves or the dog is already blocked, so a pinned dog never notices a
# post. It is driven up the flight, and the post is dropped on it mid-climb -
# the one situation that genuinely asks for a reversal, since dNow < HALF_W is a
# real overlap rather than a tight squeeze. That is the case rule 2 exists for.
EXPR3 = ('(async function(){' + JS_HELPERS +
         'var s=__MESH.state,out={rows:[]};'
         'var L=__MESH.pathLen;'
         # the flight's own stretch of the route, from the recogniser
         '__MESH.hold(0.5);await new Promise(function(r){setTimeout(r,300);});'
         'var d0=__MESH.dbgLanding();if(!d0||!d0.range)return JSON.stringify({err:"no flight on route"});'
         'var lo=d0.range[0]+0.5,hi=d0.range[1]-0.8;'
         'var g=document.getElementById("go");if(g){g.disabled=false;g.click();}'
         '__MESH.letgo();'
         'var t0=performance.now(),seen=null;'
         'for(var i=0;i<150&&!seen;i++){await new Promise(function(r){setTimeout(r,200);});'
         'var dg=__MESH.dog,p=__MESH.prog;'
         'if(!dg||p<lo||p>hi||s.obs.length)continue;'
         '__MESH.drop(dg[0],dg[1],__MESH.pathAt(p)[2],false);out.droppedAt=+p.toFixed(2);'
         'for(var j=0;j<20;j++){await new Promise(function(r){setTimeout(r,250);});'
         'var e=__MESH.escape,d=__MESH.dbgLanding();'
         'out.rows.push([Math.round((performance.now()-t0)/50)/10,+__MESH.prog.toFixed(2),'
         '__MESH.blocked?1:0,s.backOut?1:0,e.retreat,d.onFlight?1:0,d.atLanding?1:0,d.nearDist]);'
         'if(s.backOut&&e.retreat>=0&&d.onFlight&&!d.atLanding){'
         'seen={retreat:e.retreat,near:d.near,prog:__MESH.prog};break;}}}'
         'if(!seen)return JSON.stringify(Object.assign(out,{err:"no mid-flight retreat seen"}));'
         'var q=__MESH.pathAt(seen.retreat);'
         'out.retreatArc=+seen.retreat.toFixed(3);out.retreatProgGap=+(seen.prog-seen.retreat).toFixed(3);'
         'out.retreatPos=[+q[0].toFixed(3),+q[1].toFixed(3),+q[2].toFixed(3)];'
         'out.nearPos=[+seen.near[0].toFixed(3),+seen.near[1].toFixed(3),+seen.near[2].toFixed(3)];'
         'out.portalGap=+Math.hypot(q[0]-seen.near[0],q[1]-seen.near[1],q[2]-seen.near[2]).toFixed(3);'
         # The route only passes the portal cell so close, so "it retreated to the
         # landing" has to be judged against the route: no point on the route sits
         # nearer the portal than the retreat target does. Before rule 2 there was
         # no target at all - the reversal ran half a metre back along the nose.
         'var best=1e9,L2=__MESH.pathLen;'
         'for(var a=0;a<=L2;a+=0.2){var w=__MESH.pathAt(a);'
         'var h=Math.hypot(w[0]-seen.near[0],w[1]-seen.near[1]);if(h<best)best=h;}'
         'out.routeBestGap=+best.toFixed(3);'
         # and the rewind has to stop there rather than run back down the flight
         'var k=0;while(s.backOut&&k<20){await new Promise(function(r){setTimeout(r,250);});k++;}'
         'out.progAfterRetreat=+__MESH.prog.toFixed(3);out.backOutCleared=!s.backOut;'
         'out.snap=__MESH.escapeCfg.landing_snap_radius_m;'
         'return JSON.stringify(out);})()')


# ---- case 4: a floor transition is not a block, so the dwell is longer ----
# What rule 4 changes is the threshold the tick compares its stall timer
# against, and the page reports exactly that number through the escape door
# (escDwellMs over the tick's own floorPass verdict). The timing consequence -
# "no escape fired within the ordinary dwell of a handover block" - is asserted
# by escape_contract.fixture.json (the dwell_ms: 6000 case) and by its C++ twin,
# not here: a stopwatch version would need a test-only hook to reset the tick's
# private blockedSince, and a code path that only the test walks is precisely
# what this branch set out to stop shipping.
EXPR4 = ('(async function(){' + JS_HELPERS +
         'var s=__MESH.state,out={mismatch:0,fp:0,nonFp:0};'
         'if(!s.path)return JSON.stringify({err:"no path"});'
         'out.triggerMs=__MESH.escapeCfg.trigger_dwell_ms;'
         'out.transitionMs=__MESH.escapeCfg.transition_dwell_ms;'
         # Park the dog along the route instead of racing the wall clock: a demo
         # run spends its 30 s window wherever its own timing lands, which is not
         # proof. #demo interleaves flights (onStairNow -> floorPass false) and
         # open landing floor (floorPass true), so a coarse scan reaches both
         # branches deterministically and every stop checks the tick's threshold.
         'var L=__MESH.pathLen;'
         'for(var a=0.08*L;a<0.92*L;a+=0.4){await park(a);'
         'var e=__MESH.escape;'
         'if(e.dwellMs!==(e.floorPass?out.transitionMs:out.triggerMs)){out.mismatch++;}'
         'if(e.floorPass){out.fp++;}else{out.nonFp++;}'
         'if(out.fp&&out.nonFp)break;}'
         # prove the number is the live parameter and not a second literal: move
         # it and the tick's own floorPass threshold moves with it
         'var fp=-1;for(var b=0.08*L;b<0.92*L;b+=0.4){await park(b);'
         'if(__MESH.escape.floorPass){fp=b;break;}}'
         'if(fp<0)return JSON.stringify(Object.assign(out,{err:"no floorPass sample"}));'
         '__MESH.escapeCfg.transition_dwell_ms=1234;await park(fp);'
         'out.sentinelDwell=__MESH.escape.dwellMs;'
         '__MESH.escapeCfg.transition_dwell_ms=out.transitionMs;await park(fp);'
         'out.restored=__MESH.escape.dwellMs;'
         'return JSON.stringify(out);})()')

CASES = [
    ('rule 1: mid-flight holds the route (and the gate is why)', 'single,ridgedn', EXPR1,
     lambda r: (not r.get('err') and r.get('act') == 'hold' and r.get('installedOn') is False
                and r.get('routeUnchanged') and r.get('terrain') == 'hold'
                and r.get('installedOff') is True and r.get('routeChangedOff')),
     lambda r: 'commit on: installed=%s route untouched=%s terrain=%s | commit off: installed=%s '
               'route swapped=%s (prog %s m on flight %s, portal %s m away)'
               % (r.get('installedOn'), r.get('routeUnchanged'), r.get('terrain'),
                  r.get('installedOff'), r.get('routeChangedOff'), r.get('prog'),
                  r.get('flight'), r.get('nd'))),
    ('rule 1/2 exception: a landing may still swap routes', 'single,ridgedn', EXPR2,
     lambda r: (not r.get('err') and r.get('installed') is True and r.get('routeChanged')
                and r.get('terrain') == 'allow' and r.get('nd') is not None
                and r['nd'] <= r['snap'] and (r.get('trace') or {}).get('action') == 'installed'),
     lambda r: 'flight %s, %s m from the portal (snap %s m): installed=%s terrain=%s kind=%s'
               % (r.get('flight'), r.get('nd'), r.get('snap'), r.get('installed'),
                  r.get('terrain'), (r.get('trace') or {}).get('kind'))),
    ('rule 2: a mid-flight reversal retreats to the portal', 'single,stair', EXPR3,
     lambda r: (not r.get('err') and r.get('retreatArc') is not None
                and r.get('retreatProgGap', 0) > 0
                and r['portalGap'] <= r['routeBestGap'] + 0.25
                and r.get('backOutCleared')
                and r['progAfterRetreat'] >= r['retreatArc'] - 0.02),
     lambda r: 'retreat arc %s m (from %s m back), route point %s vs portal %s: gap %s m, '
               'closest the route gets is %s m; rewind stopped at prog %s (backOut cleared=%s)'
               % (r.get('retreatArc'), r.get('retreatProgGap'), r.get('retreatPos'),
                  r.get('nearPos'), r.get('portalGap'), r.get('routeBestGap'),
                  r.get('progAfterRetreat'), r.get('backOutCleared'))),
    ('rule 4: a floor transition raises the stall threshold', 'demo', EXPR4,
     lambda r: (not r.get('err') and r.get('mismatch') == 0
                and r.get('fp', 0) > 0 and r.get('nonFp', 0) > 0
                and r.get('transitionMs') == 2 * r.get('triggerMs')
                and r.get('sentinelDwell') == 1234
                and r.get('restored') == r.get('transitionMs')),
     lambda r: 'trigger %s ms / transition %s ms over %s handover + %s ordinary samples, '
               'mismatches %s; retuning transition to 1234 moved the tick\'s threshold to %s '
               '(restored %s)'
               % (r.get('triggerMs'), r.get('transitionMs'), r.get('fp'), r.get('nonFp'),
                  r.get('mismatch'), r.get('sentinelDwell'), r.get('restored'))),
]


def main():
    wanted = [int(a) for a in sys.argv[1:] if a.isdigit()] or [1, 2, 3, 4]
    ok = total = 0
    for n, (title, hash_, expr, check, report) in enumerate(CASES, 1):
        if n not in wanted:
            continue
        total += 1
        print('--- %d. %s  [#%s] ---' % (n, title, hash_))
        out, exc = run(hash_, expr, wait=16.0 if hash_ == 'demo' else 12.0)
        r = value(out)
        if exc:
            print('  page exceptions:', exc[:3])
        if not isinstance(r, dict):
            print('  RESULT: FAIL (no result)')
            ok += 0
            continue
        if 'err' in r:
            print('  setup failure:', r)
            print('  RESULT: FAIL')
            continue
        print('  ' + report(r))
        if r.get('rows'):
            for x in r['rows'][-6:]:
                print('    ', x)
        passed = bool(check(r)) and not exc
        print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
        ok += 1 if passed else 0
    print('\n%d/%d passed' % (ok, total))
    return 0 if ok == total else 1


if __name__ == '__main__':
    sys.exit(main())
