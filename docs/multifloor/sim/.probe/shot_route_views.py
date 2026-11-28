"""Save the two route lines as a recording would see them: overview and close.

WebGL ignores linewidth, so both routes are camera-aware ribbon meshes now. The
pixel numbers come from the page itself (dbgRoutePx); these frames are for
eyeballing that bold does not mean blobby - the route must still read as a
centre line, not as a painted corridor.
"""
import base64, json, os, subprocess, time, urllib.request
import websocket

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 9393
VIEWS = [('route_overview.png', 0), ('route_close.png', -120)]

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
            if r.get('method') == 'Runtime.exceptionThrown':
                exc.append(json.dumps(r['params'].get('exceptionDetails', {}))[:400])
            if r.get('id') == mid[0]:
                if 'error' in r: raise Exception(r['error'])
                return r.get('result', {})

    def ev(expr):
        return send('Runtime.evaluate', {'expression': expr,
                                         'returnByValue': True}).get('result', {}).get('value')

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

    for name, dy in VIEWS:
        if dy:
            for _ in range(14):
                send('Input.dispatchMouseEvent', {'type': 'mouseWheel', 'x': 800, 'y': 680,
                                                  'deltaX': 0, 'deltaY': dy})
                time.sleep(0.05)
            time.sleep(1.5)
        print(name, json.dumps(ev('__MESH.dbgRoutePx()')))
        png = base64.b64decode(send('Page.captureScreenshot', {'format': 'png'})['data'])
        open(os.path.join(SIM, name), 'wb').write(png)

    # the view a recording actually uses: #follow rides the camera behind the
    # dog, which is exactly where an opaque band would wipe the body out
    ev("location.hash = 'follow'")
    for _ in range(8):
        send('Input.dispatchMouseEvent', {'type': 'mouseWheel', 'x': 800, 'y': 680,
                                          'deltaX': 0, 'deltaY': -120})
        time.sleep(0.05)
    time.sleep(2.5)
    print('route_follow.png', json.dumps(ev('__MESH.dbgRoutePx()')))
    png = base64.b64decode(send('Page.captureScreenshot', {'format': 'png'})['data'])
    open(os.path.join(SIM, 'route_follow.png'), 'wb').write(png)
    print('PAGE EXC:', len(exc), exc[:2])
    ws.close()
finally:
    p.terminate()
