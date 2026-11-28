"""Hero shot of the fixed stair case: post on the tread, global detour
installed, dog climbing past it with the synthetic sensor active."""
import base64, json, subprocess, time, urllib.request, os
import websocket
SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 9389
p = subprocess.Popen(['google-chrome', '--headless=new', '--no-sandbox',
                      '--disable-gpu', '--use-gl=swiftshader',
                      '--enable-unsafe-swiftshader', '--hide-scrollbars',
                      '--remote-debugging-port=%d' % PORT, '--window-size=1600,1000',
                      'about:blank'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    for _ in range(60):
        try:
            tabs = json.load(urllib.request.urlopen('http://127.0.0.1:%d/json/list' % PORT))
            if tabs: break
        except Exception: time.sleep(0.3)
    ws = websocket.create_connection(tabs[0]['webSocketDebuggerUrl'], timeout=120)
    mid = [0]
    def ev(expr):
        mid[0] += 1
        ws.send(json.dumps({'id': mid[0], 'method': 'Runtime.evaluate',
                            'params': {'expression': expr, 'returnByValue': True}}))
        while True:
            r = json.loads(ws.recv())
            if r.get('id') == mid[0]:
                return r['result']['result'].get('value')
    ev('Page.enable'); ev('Runtime.enable')
    mid[0] += 1
    ws.send(json.dumps({'id': mid[0], 'method': 'Page.navigate',
                        'params': {'url': 'file://' + SIM + '/mesh_nav.html#single,stair,sensor,senseviz'}}))
    while True:
        r = json.loads(ws.recv())
        if r.get('id') == mid[0]: break
    time.sleep(5)
    print('setup:', ev('''(()=>{const p=__MESH.pathAt(2.0);
        window.__POST=[p[0],p[1]];
        __MESH.drop(p[0],p[1],p[2],false);
        const g=document.getElementById("go"); if(g){g.disabled=false;g.click();}
        return "ok";})()'''))
    # wait until the dog is beside the post on the flight
    for _ in range(90):
        time.sleep(0.4)
        st = ev('''(()=>({prog:__MESH.prog,len:__MESH.pathLen,
            dog:__MESH.dog,tracks:__MESH.state.sensorObs.length,
            near:Math.hypot(__MESH.dog[0]-window.__POST[0],__MESH.dog[1]-window.__POST[1])}))()''')
        if st and st['near'] < 0.62 and st['dog'][2] > 0.45:
            break
    print('state:', st)
    print('freeze:', ev('''(()=>{__MESH.hold(__MESH.prog);
        const d=__MESH.dog; cam.target.set(d[0],d[1],d[2]+0.25);
        cam.dist=2.6; cam.pitch=0.30; cam.yaw=-0.78; applyCam();
        return {prog:__MESH.prog,len:__MESH.pathLen};})()'''))
    time.sleep(1.0)
    mid[0] += 1
    ws.send(json.dumps({'id': mid[0], 'method': 'Page.captureScreenshot',
                        'params': {'format': 'png'}}))
    while True:
        r = json.loads(ws.recv())
        if r.get('id') == mid[0]:
            open(os.path.join(SIM, 'verify_stair_detour.png'), 'wb').write(
                base64.b64decode(r['result']['data']))
            break
    print('saved verify_stair_detour.png')
finally:
    p.terminate()