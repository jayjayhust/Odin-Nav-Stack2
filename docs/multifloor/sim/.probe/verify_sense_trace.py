#!/usr/bin/env python3
"""Focused traces: (1) stair treadblock with sensor OFF vs ON - is the stop
the same honest verdict the default layer gives? (2) floor sensor-mode pass:
did the dog actually keep hard clearance off the post while it was visible?"""
import base64, json, os, subprocess, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9378

def session(hash_, seconds, sample_expr, shot=None):
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
                if pages: break
            except Exception: pass
            time.sleep(0.5)
        ws = websocket.create_connection(pages[0]['webSocketDebuggerUrl'], timeout=30)
        mid = [0]
        def call(method, **params):
            mid[0] += 1
            ws.send(json.dumps({'id': mid[0], 'method': method, 'params': params}))
            while True:
                m = json.loads(ws.recv())
                if m.get('id') == mid[0]: return m.get('result', {})
        call('Page.enable'); call('Runtime.enable')
        def ev(expr):
            return call('Runtime.evaluate', expression=expr, returnByValue=True,
                        awaitPromise=True).get('result', {}).get('value')
        time.sleep(5.0)
        samples = []
        t0 = time.time()
        while time.time() - t0 < seconds:
            s = ev(sample_expr)
            if s: samples.append(json.loads(s))
            time.sleep(0.25)
        if shot:
            img = call('Page.captureScreenshot', format='png')
            open(os.path.join(BASE, shot), 'wb').write(base64.b64decode(img['data']))
        ws.close()
        return samples
    finally:
        proc.terminate()
        try: proc.wait(timeout=5)
        except Exception: proc.kill()
        time.sleep(0.5)

TRACE = ('JSON.stringify({dog:__MESH.dog, prog:+__MESH.prog.toFixed(2),'
         ' blocked:__MESH.blocked, lat:+__MESH.state.latOff.toFixed(3),'
         ' clusters:__MESH.sensor.clusters,'
         ' obs:__MESH.state.obs.map(o=>[+o.x.toFixed(2),+o.y.toFixed(2),+o.r.toFixed(2)])})')

def main():
    print('--- stair treadblock, sensor OFF (default) ---')
    s = session('single,stair,autoobs,treadblock', 14, TRACE)
    zs = [q['dog'][2] for q in s if q['dog']]
    print('maxz=%.2f  blocked frames=%d/%d  last=%s' % (max(zs), sum(q['blocked'] for q in s), len(s), json.dumps(s[-1])))

    print('--- stair treadblock, sensor ON ---')
    s = session('single,stair,sensor,autoobs,treadblock', 14, TRACE, 'verify_sense_stair_refuse.png')
    zs = [q['dog'][2] for q in s if q['dog']]
    print('maxz=%.2f  blocked frames=%d/%d  clusters max=%d  last=%s' % (
        max(zs), sum(q['blocked'] for q in s), len(s), max(q['clusters'] for q in s), json.dumps(s[-1])))

    print('--- floor sensor ON: clearance trace around the post ---')
    s = session('single,floor,sensor,autoobs', 16, TRACE, 'verify_sense_sensor.png')
    best = 9.9; bestq = None; seen = 0
    for q in s:
        if not q['dog'] or not q['obs']: continue
        seen = max(seen, q['clusters'])
        for o in q['obs']:
            d = ((q['dog'][0]-o[0])**2 + (q['dog'][1]-o[1])**2)**0.5 - o[2]
            if d < best: best, bestq = d, q
    print('min surface clearance=%.3f m (hard limit 0.225)  clusters seen max=%d' % (best, seen))
    print('closest frame:', json.dumps(bestq))

if __name__ == '__main__':
    main()