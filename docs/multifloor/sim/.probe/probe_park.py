"""Near-goal park diagnosis: drop a post mid-flight, let the dog detour and
climb, then inspect stairLegInfo at the parked progress with coordinates."""
import json, subprocess, time, urllib.request, os
import websocket
SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 9388
p = subprocess.Popen(['google-chrome', '--headless=new', '--no-sandbox',
                      '--disable-gpu', '--use-gl=swiftshader',
                      '--enable-unsafe-swiftshader',
                      '--remote-debugging-port=%d' % PORT, '--window-size=1600,1000',
                      'about:blank'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    for _ in range(60):
        try:
            tabs = json.load(urllib.request.urlopen('http://127.0.0.1:%d/json/list' % PORT))
            if tabs: break
        except Exception: time.sleep(0.3)
    ws = websocket.create_connection(tabs[0]['webSocketDebuggerUrl'], timeout=180)
    mid = [0]
    errs = []
    def ev(expr):
        mid[0] += 1
        ws.send(json.dumps({'id': mid[0], 'method': 'Runtime.evaluate',
                            'params': {'expression': expr, 'returnByValue': True}}))
        while True:
            r = json.loads(ws.recv())
            if r.get('method') == 'Runtime.exceptionThrown':
                errs.append(r['params']['exceptionDetails'].get('exception', {}).get('description', '?'))
            if r.get('id') == mid[0]:
                if 'exceptionDetails' in r.get('result', {}):
                    return {'EVAL_ERR': r['result']['exceptionDetails'].get('text')}
                return r['result']['result'].get('value')
    ev('Page.enable'); ev('Runtime.enable')
    mid[0] += 1
    ws.send(json.dumps({'id': mid[0], 'method': 'Page.navigate',
                        'params': {'url': 'file://' + SIM + '/mesh_nav.html#single,stair,sensor'}}))
    while True:
        r = json.loads(ws.recv())
        if r.get('id') == mid[0]: break
    time.sleep(5)
    print('setup:', ev('''(()=>{const p=__MESH.pathAt(2.0);
        __MESH.drop(p[0],p[1],p[2],false);
        const g=document.getElementById("go"); if(g){g.disabled=false;g.click();}
        return "ok";})()'''))
    for i in range(14):
        time.sleep(3)
        print('t%02d:' % (i*3), ev('''(()=>({prog:+__MESH.prog.toFixed(2),
            len:+__MESH.pathLen.toFixed(2),blocked:!!__MESH.blocked,
            arrived:!!__MESH.state.arrived,dog:__MESH.dog.map(v=>+v.toFixed(2)),
            tracks:__MESH.state.sensorObs.map(o=>[+o.x.toFixed(2),+o.y.toFixed(2),+o.r.toFixed(2)]),
            toast:(document.getElementById("toast")||{}).textContent||""}))()'''))
    prog = ev('__MESH.prog')
    print('info@prog:', ev('__MESH.stairLegInfo(%f)' % prog))
    print('info@end :', ev('__MESH.stairLegInfo(__MESH.pathLen)'))
    print('info@mid :', ev('__MESH.stairLegInfo(1.5)'))
    print('PAGE ERRORS:', errs[:4])
finally:
    p.terminate()