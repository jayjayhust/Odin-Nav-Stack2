# Head-vs-MOTION check: the dog's nose (dogMesh.rotation.z) must track the
# direction it actually travels (its per-frame XY displacement). We paint a cell
# on the route so the local layer forces a visible veer, then verify the angle
# between velocity and heading stays small while the dog moves.
import json
import math
import os
import subprocess
import sys
import time
import urllib.request

import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 9410


def run():
    url = 'file://%s/mesh_nav.html#single,floor' % BASE
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
        ws = websocket.create_connection(pages[0]['webSocketDebuggerUrl'], timeout=60)
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
        ev('window.NOREPLAN=1')
        # setup: plan, paint a route cell to force a veer, then let the page's
        # own rAF loop drive it (real dt keeps the blocked/veer timers honest)
        setup = ev(
            "(function(){try{var m=state.map;"
            "state.start=[2.53,7.40,0.13]; state.goal=[7.82,4.13,0.13];"
            "document.getElementById('go').click();"
            "var best=null;"
            "for(var fr=0.12; fr<=0.7 && !best; fr+=0.04){"
            "  var a=pathAt(state.path,pathLen*fr);"
            "  var i=Math.floor((a[0]-m.lo[0])/m.res), j=Math.floor((a[1]-m.lo[1])/m.res);"
            "  if(i<1||j<1||i>=m.nx-1||j>=m.ny-1) continue;"
            "  var kk=Math.max(1,Math.round((a[2]+0.12-m.lo[2])/m.res));"
            "  for(var k=kk;k<Math.min(m.nz,kk+5);k++){"
            "    if(m.data[(k*m.ny+j)*m.nx+i]===1){best={i:i,j:j,k:k};break;}}"
            "}"
            "if(!best) return JSON.stringify({err:'no FREE cell'});"
            "blockCell(best.i,best.j,best.k); state.senseMode=1;"
            "if(typeof senseReset==='function')senseReset();"
            "state.running=true;"
            "return JSON.stringify({ok:1,pathLen:pathLen});"
            "}catch(e){return JSON.stringify({err:String(e && e.stack || e)});}})()")
        if not setup or 'err' in json.loads(setup):
            ws.close(); return setup, exc
        # poll the live dog heading + the route-tangent it should match
        rows = []
        t0 = time.time()
        while time.time() - t0 < 22.0:
            r = ev("(function(){if(!dogMesh||!state.path)return null;"
                   "var b=pathAt(state.path,Math.min(pathLen,prog+0.35));"
                   "return JSON.stringify([dogMesh.rotation.z,"
                   "Math.atan2(b[1]-dogMesh.position.y,b[0]-dogMesh.position.x),prog]);})()")
            if r:
                rows.append(json.loads(r))
            time.sleep(0.05)
        ws.close()
        return json.dumps({'rows': rows}), exc
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


data, exc = run()
if not data:
    print("no data", json.dumps(exc)[:400]); sys.exit(2)
obj = json.loads(data)
if 'err' in obj:
    print("setup err:", obj); sys.exit(2)
rows = obj['rows']
if len(rows) < 3:
    print("too few samples", rows[:3]); sys.exit(2)
worst = 0.0
nadv = 0
for i in range(1, len(rows)):
    yaw, tangent = rows[i][0], rows[i][1]
    if abs(rows[i][2] - rows[i - 1][2]) < 0.01:   # not advancing: heading held
        continue
    nadv += 1
    worst = max(worst, abs(wrap(tangent - yaw)))
print("prog_end=%.2f  samples=%d advancing=%d  worst |nose - route-tangent| = %.2f rad (%.0f deg)"
      % (rows[-1][2], len(rows), nadv, worst, math.degrees(worst)))
ok = worst < 0.35
print("VERDICT:", "PASS (nose tracks motion)" if ok else "FAIL")
sys.exit(0 if ok else 1)
