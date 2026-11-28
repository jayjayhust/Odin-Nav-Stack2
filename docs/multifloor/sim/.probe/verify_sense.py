#!/usr/bin/env python3
"""Headless CDP verification of the sensor-data view (showSense / senseMode).

Same harness pattern as record_mesh_nav.py: chromium headless + swiftshader,
drive nothing, let the page's own demo timers and tick run, then read state
through window.__MESH and grab one screenshot per scenario.

    python3 .probe/verify_sense.py
"""
import base64, json, os, subprocess, sys, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9377
CASES = [
    # hash, screenshot, expect_sensor_on, min_pts, stair
    ('single,floor,senseviz,autoobs,follow',        'verify_sense_consumption.png', False, 0,  False),
    ('single,floor,sensor,autoobs,follow',          'verify_sense_sensor.png',      True,  50, False),
    ('single,floor,senseviz,sensor,autoobs,follow', 'verify_sense_both.png',        True,  50, False),
    # plain climb: the synthetic sensor must not break stair traversal
    ('single,stair,sensor',                         'verify_sense_stair.png',       True,  0,  True),
    # treadblock: a post centred on a narrow tread is an honest STOP verdict
    # (the default layer refuses too), so assert scan + verdict, not a climb
    ('single,stair,sensor,autoobs,treadblock',      'verify_sense_stair_refuse.png', True, 0,  False),
    ('single,floor,autoobs',                        'verify_sense_default.png',     False, 0,  False),
]

def wait_page():
    for _ in range(60):
        try:
            tgt = json.load(urllib.request.urlopen('http://127.0.0.1:%d/json' % PORT))
            pages = [t for t in tgt if t['type'] == 'page']
            if pages:
                return pages[0]
        except Exception:
            pass
        time.sleep(0.5)
    raise RuntimeError('no debuggable page')

def run_case(hash_, shot, expect_on, min_pts, stair):
    url = 'file://%s/mesh_nav.html#%s' % (BASE, hash_)
    log = open('/tmp/verify_sense_%s.log' % shot, 'w')
    proc = subprocess.Popen(
        ['google-chrome', '--headless=new', '--disable-gpu', '--no-sandbox',
         '--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--hide-scrollbars',
         '--disable-dev-shm-usage', '--remote-debugging-port=%d' % PORT,
         '--remote-allow-origins=*', '--window-size=1600,900', url],
        stdout=subprocess.DEVNULL, stderr=log)
    try:
        page = wait_page()
        ws = websocket.create_connection(page['webSocketDebuggerUrl'], timeout=30)
        mid = [0]
        errors = []
        def call(method, **params):
            mid[0] += 1
            ws.send(json.dumps({'id': mid[0], 'method': method, 'params': params}))
            while True:
                m = json.loads(ws.recv())
                if m.get('method') == 'Runtime.exceptionThrown':
                    d = m['params']['exceptionDetails']
                    errors.append(d.get('exception', {}).get('description', d.get('text', '?')))
                if m.get('method') == 'Log.entryAdded' and m['params']['entry']['level'] == 'error':
                    errors.append(m['params']['entry']['text'])
                if m.get('id') == mid[0]:
                    return m.get('result', {})
        call('Page.enable'); call('Runtime.enable'); call('Log.enable')
        def ev(expr):
            r = call('Runtime.evaluate', expression=expr, returnByValue=True, awaitPromise=True)
            return r.get('result', {}).get('value')
        # demo timers: 600 ms load + 1200 ms plan; give the walk some time too
        time.sleep(6.0 if not stair else 10.0)
        snap1 = ev('JSON.stringify({sensor:__MESH.sensor, dog:__MESH.dog, prog:__MESH.prog,'
                   ' blocked:__MESH.blocked, show:__MESH.state.showSense, mode:__MESH.state.senseMode,'
                   ' obs:__MESH.state.obs.length, hud:document.getElementById("senseHud").style.display})')
        time.sleep(2.5)
        snap2 = ev('JSON.stringify({sensor:__MESH.sensor, dog:__MESH.dog, prog:__MESH.prog,'
                   ' blocked:__MESH.blocked, maxz:window.__maxz||0})')
        if stair:
            # let the climb run, track the highest base z reached
            ev('window.__maxz=0; setInterval(()=>{const d=__MESH.dog; if(d) window.__maxz=Math.max(window.__maxz,d[2]);},200)')
            time.sleep(8.0)
            snap2 = ev('JSON.stringify({sensor:__MESH.sensor, dog:__MESH.dog, prog:__MESH.prog,'
                       ' blocked:__MESH.blocked, maxz:window.__maxz||0})')
        img = call('Page.captureScreenshot', format='png')
        open(os.path.join(BASE, shot), 'wb').write(base64.b64decode(img['data']))
        ws.close()
        s1, s2 = json.loads(snap1), json.loads(snap2)
        moved = (s1['dog'] and s2['dog'] and
                 (abs(s1['dog'][0]-s2['dog'][0]) + abs(s1['dog'][1]-s2['dog'][1]) > 0.02
                  or abs(s1['prog']-s2['prog']) > 0.02))
        ok = True
        reasons = []
        if s1['sensor']['on'] != expect_on: ok = False; reasons.append('sensor.on=%s want %s' % (s1['sensor']['on'], expect_on))
        if expect_on and s2['sensor']['pts'] < min_pts and s1['sensor']['pts'] < min_pts:
            ok = False; reasons.append('pts=%d/%d < %d' % (s1['sensor']['pts'], s2['sensor']['pts'], min_pts))
        if not expect_on and (s1['sensor']['pts'] or s2['sensor']['pts']):
            ok = False; reasons.append('pts nonzero while off')
        if not moved and not s2['blocked']: ok = False; reasons.append('dog did not move')
        if stair and s2.get('maxz', 0) < 0.30: ok = False; reasons.append('never climbed (maxz=%.2f)' % s2.get('maxz', 0))
        real_errors = [e for e in errors if 'favicon' not in e.lower()]
        if real_errors: ok = False; reasons.append('console errors: %s' % real_errors[:2])
        print('%-34s %s  s1=%s s2=%s%s' % (hash_, 'PASS' if ok else 'FAIL',
              json.dumps(s1), json.dumps(s2), ('  ' + '; '.join(reasons)) if reasons else ''))
        return ok
    finally:
        proc.terminate()
        try: proc.wait(timeout=5)
        except Exception: proc.kill()
        log.close()
        time.sleep(0.5)

if __name__ == '__main__':
    results = [run_case(*c) for c in CASES]
    print('=== %d/%d passed' % (sum(results), len(results)))
    sys.exit(0 if all(results) else 1)