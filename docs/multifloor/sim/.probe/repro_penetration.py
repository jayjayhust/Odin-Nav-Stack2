#!/usr/bin/env python3
"""Repro of the user report: on demo1 (示例 单层阶梯) a post dropped by hand on
the route is penetrated by the global line AND by the dog. Unlike autoobs,
which only plants posts where a detour window exists, this drops the post at
a fixed route station in both local modes and watches the body/post gap."""
import base64, json, os, subprocess, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9380

def session(hash_, seconds, setup_expr, sample_expr, shot=None, mid_expr=None):
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
                tgt = json.load(urllib.request.urlopen('http://127.0.0.1:%d' % PORT + '/json'))
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
        if os.environ.get('NOREPLAN'): ev('window.NOREPLAN=1')
        setup = ev(setup_expr)
        samples = []
        midres = None
        t0 = time.time()
        while time.time() - t0 < seconds:
            if mid_expr and midres is None and time.time() - t0 > seconds/2:
                midres = ev(mid_expr)
            s = ev(sample_expr)
            if s: samples.append(json.loads(s))
            time.sleep(0.2)
        if shot:
            img = call('Page.captureScreenshot', format='png')
            open(os.path.join(BASE, shot), 'wb').write(base64.b64decode(img['data']))
        ws.close()
        return setup, samples, midres
    finally:
        proc.terminate()
        try: proc.wait(timeout=5)
        except Exception: proc.kill()
        time.sleep(0.5)

# drop the post at a fixed station, then report the global line's closest
# approach to the post axis (horizontal) so complaint #1 is measured too
def drop_expr(station):
    return ('(function(){var a=__MESH.pathAt(%f);'
            'var o=__MESH.drop(a[0],a[1],a[2],false);'
            'document.getElementById("go").disabled=false;'
            'if(!window.NOREPLAN)document.getElementById("go").click();'
            'var g=1e9;for(var s=0;s<=__MESH.pathLen;s+=0.1){'
            'var q=__MESH.pathAt(s);var d=Math.hypot(q[0]-o.x,q[1]-o.y);'
            'if(d<g)g=d;}'
            'return JSON.stringify({post:o,globalMinDist:+g.toFixed(3),'
            'pathLen:+__MESH.pathLen.toFixed(2),'
            'msg:document.getElementById("say").textContent});})()' % station)

TRACE = ('JSON.stringify({dog:__MESH.dog, prog:+__MESH.prog.toFixed(2),'
         ' blocked:__MESH.blocked, lat:+__MESH.state.latOff.toFixed(3),'
         ' win:+(__MESH.state.latWin||0).toFixed(3),'
         ' stair:!!__MESH.state.onStairNow,'
         ' clusters:__MESH.sensor.clusters,'
         ' post:__MESH.state.obs.map(function(o){return [o.x,o.y,o.z,o.r];})})')

def report(tag, setup, samples, midres=None):
    print('--- %s ---' % tag)
    print('setup:', setup)
    if midres: print('mid replan:', midres)
    if not setup: return
    post = json.loads(setup)['post']
    best = 9.9; bestq = None; pen = 0
    for q in samples:
        if not q['dog'] or not q['post']: continue
        o = q['post'][0]
        d = ((q['dog'][0]-o[0])**2 + (q['dog'][1]-o[1])**2)**0.5 - o[3]
        if d < best: best, bestq = d, q
        if d < 0.175: pen += 1
    print('min body-surface gap=%.3f m (half width 0.175)  overlap frames=%d/%d'
          % (best, pen, len(samples)))
    print('closest:', json.dumps(bestq))
    print('last:', json.dumps(samples[-1]) if samples else None)

# in sensor mode the global merge can only know what the memory has seen, so
# replan again mid-run once the dog has looked at the post
MID = ('(function(){document.getElementById("go").disabled=false;'
       'document.getElementById("go").click();'
       'var o=__MESH.state.obs[0];var g=1e9;'
       'for(var s=0;s<=__MESH.pathLen;s+=0.1){'
       'var q=__MESH.pathAt(s);var d=Math.hypot(q[0]-o.x,q[1]-o.y);'
       'if(d<g)g=d;}'
       'return JSON.stringify({globalMinDist:+g.toFixed(3),'
       'clusters:__MESH.sensor.clusters});})()')

def main():
    import sys
    only = sys.argv[1] if len(sys.argv) > 1 else None
    for tag, hash_, st in [
        ('floor, sensor OFF, post mid-route', 'single,floor', 3.0),
        ('floor, sensor ON,  post mid-route', 'single,floor,sensor', 3.0),
        ('stair, sensor OFF, post on tread', 'single,stair', 2.0),
        ('stair, sensor ON,  post on tread', 'single,stair,sensor', 2.0),
    ]:
        if only and only not in hash_: continue
        mid = MID if 'sensor' in hash_ else None
        setup, s, mr = session(hash_, 18, drop_expr(st), TRACE, mid_expr=mid)
        report(tag, setup, s, mr)
        if os.environ.get('DUMP'):
            fn = os.path.join(os.path.dirname(__file__),
                              'trace_%s.json' % hash_.replace(',', '_'))
            json.dump(s, open(fn, 'w'))
            print('dumped', fn)

if __name__ == '__main__':
    main()