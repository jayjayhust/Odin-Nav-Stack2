"""shot_dog_side: side and front close-ups on opaque geometry.

The resting dog, no route, no follow cam: the point is to see the baked shell
sitting on the tread with its legs bent (a wrong up-axis shows up here as a
body with its limbs thrown forward).
"""
import base64, json, os, subprocess, time, urllib.request
import websocket

SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 9397
p = subprocess.Popen(['google-chrome', '--headless=new', '--no-sandbox', '--disable-gpu',
                      '--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--hide-scrollbars',
                      '--remote-debugging-port=%d' % PORT, '--window-size=1200,800', 'about:blank'],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    tabs = None
    for _ in range(60):
        try:
            tabs = json.load(urllib.request.urlopen('http://127.0.0.1:%d/json/list' % PORT))
            if tabs: break
        except Exception: time.sleep(0.3)
    ws = websocket.create_connection(tabs[0]['webSocketDebuggerUrl'], timeout=120)
    mid = [0]

    def send(m, par=None):
        mid[0] += 1
        ws.send(json.dumps({'id': mid[0], 'method': m, 'params': par or {}}))
        while True:
            r = json.loads(ws.recv())
            if r.get('id') == mid[0]: return r.get('result', {})

    def ev(e):
        return send('Runtime.evaluate', {'expression': e, 'returnByValue': True}).get('result', {}).get('value')

    send('Runtime.enable'); send('Page.enable')
    send('Page.navigate', {'url': 'file://' + SIM + '/mesh_nav.html#single,solid,nosave'})
    time.sleep(9)
    print('dog', ev('JSON.stringify(__MESH.dogBody)'))
    for name, yaw, pitch, dist in (('verify_dog_go2_side.png', -1.57, 0.06, 0.95),
                                   ('verify_dog_go2_front.png', -2.9, 0.12, 0.95)):
        print(name, ev('JSON.stringify(__MESH.dbgCam(null, %f, %f, %f))' % (dist, yaw, pitch)))
        time.sleep(0.35)
        open(os.path.join(SIM, name), 'wb').write(
            base64.b64decode(send('Page.captureScreenshot', {'format': 'png'})['data']))
    print('ok')
finally:
    p.terminate()
