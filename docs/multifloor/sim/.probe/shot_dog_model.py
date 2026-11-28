"""shot_dog_model: close-ups of the two vendor bodies, for eyeballing only.

Rides the demo route, turns on #follow and zooms in to the 2 m floor so the
0.7 m dog fills a few hundred pixels, then shoots the same frame for 宇树 Go2
and 智身 钢镚 L1. The numeric half of the check is probe_dog_model.py.
"""
import base64, json, os, subprocess, time, urllib.request
import websocket

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 9394

p = subprocess.Popen(['google-chrome', '--headless=new', '--no-sandbox',
                      '--disable-gpu', '--use-gl=swiftshader',
                      '--enable-unsafe-swiftshader', '--hide-scrollbars',
                      '--remote-debugging-port=%d' % PORT, '--window-size=1400,900',
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
    send('Page.navigate', {'url': 'file://' + SIM + '/mesh_nav.html#single,nosave'})
    time.sleep(8)
    for _ in range(90):
        time.sleep(1)
        if ev("!!window.__MESH && !!__MESH.map && !!__MESH.dog"): break
    ev("document.getElementById('go').disabled=false; document.getElementById('go').click()")
    time.sleep(2.0)

    def shoot(name):
        # re-aim at the dog right before the capture: it keeps walking, and the
        # camera is only re-applied by a drag or by this call
        ev("__MESH.dbgCam(null, 1.4, -2.2, 0.22)")
        time.sleep(0.1)
        print(name, json.dumps(ev('({dog:__MESH.dog, body:__MESH.dogBody})')))
        png = base64.b64decode(send('Page.captureScreenshot', {'format': 'png'})['data'])
        open(os.path.join(SIM, name), 'wb').write(png)

    shoot('verify_dog_go2.png')
    ev("(function(){var s=document.getElementById('robotModel');s.value='l1';"
       "s.onchange({target:s});})()")
    time.sleep(1.5)
    shoot('verify_dog_l1.png')
    print('PAGE EXC:', len(exc), exc[:2])
    ws.close()
finally:
    p.terminate()
