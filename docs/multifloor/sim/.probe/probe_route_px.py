"""Report the real on-screen pixel width of the two route ribbons.

WebGL ignores LineBasicMaterial.linewidth, so both routes are ribbon meshes now.
This asks the page to project the ribbon vertices through the live camera and
measure the drawn thickness directly - no colour masks, no guessing which amber
pixel belongs to the leg and which to a tread band.
"""
import json, os, subprocess, time, urllib.request
import websocket

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 9392

p = subprocess.Popen(['google-chrome', '--headless=new', '--no-sandbox',
                      '--disable-gpu', '--use-gl=swiftshader',
                      '--enable-unsafe-swiftshader', '--hide-scrollbars',
                      '--remote-debugging-port=%d' % PORT, '--window-size=1600,1000',
                      'about:blank'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    tabs = None
    for _ in range(60):
        try:
            tabs = json.load(urllib.request.urlopen('http://127.0.0.1:%d/json/list' % PORT))
            if tabs: break
        except Exception: time.sleep(0.3)
    ws = websocket.create_connection(tabs[0]['webSocketDebuggerUrl'], timeout=180)
    mid = [0]
    exc = []

    def send(method, params=None):
        mid[0] += 1
        ws.send(json.dumps({'id': mid[0], 'method': method, 'params': params or {}}))
        while True:
            r = json.loads(ws.recv())
            m = r.get('method')
            if m == 'Runtime.exceptionThrown':
                exc.append(json.dumps(r['params'].get('exceptionDetails', {}))[:400])
            if r.get('id') == mid[0]:
                if 'error' in r: raise Exception(r['error'])
                return r.get('result', {})

    def ev(expr):
        return send('Runtime.evaluate', {'expression': expr,
                                         'returnByValue': True}).get('result', {}).get('value')

    def wheel(n, dy):
        # the page's own zoom: host.addEventListener('wheel') -> cam.dist
        for _ in range(n):
            send('Input.dispatchMouseEvent', {'type': 'mouseWheel', 'x': 800, 'y': 680,
                                              'deltaX': 0, 'deltaY': dy})
            time.sleep(0.05)

    send('Runtime.enable'); send('Page.enable')
    send('Page.navigate', {'url': 'file://' + SIM + '/mesh_nav.html'})
    time.sleep(8)
    ev("document.getElementById('demo1').click()")
    for _ in range(90):
        time.sleep(1)
        if ev("!!window.__MESH && !!__MESH.map"): break
    ev("""(function(){const s=__MESH.state;s.start=[1.38,7.39,0.13];s.goal=[14.08,3.35,0.13];
 s.ways=[[3.92,4.78,0.13],[6.41,2.87,0.00],[10.16,1.61,0.13]];})()""")
    ev("document.getElementById('go').disabled=false; document.getElementById('go').click()")
    time.sleep(6)

    print('default view :', json.dumps(ev('__MESH.dbgRoutePx()')))
    wheel(12, -120)                      # zoom in, the way a recording frames the dog
    time.sleep(1.5)
    print('zoomed in    :', json.dumps(ev('__MESH.dbgRoutePx()')))
    wheel(24, 120)                       # and back out again
    time.sleep(1.5)
    print('zoomed out   :', json.dumps(ev('__MESH.dbgRoutePx()')))
    print('PAGE EXC:', len(exc), exc[:2])
    ws.close()
finally:
    p.terminate()
