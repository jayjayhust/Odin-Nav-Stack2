"""User report: after dropping a post on the stair flight and replanning, the
drawn global line still crosses the post and the dog parks (STOP).  Trace the
global route's min distance to the post, the blocked/backOut flags and the
perception tracks frame by frame to see which layer keeps the penetration."""
import json, subprocess, time, urllib.request, os
import websocket
SIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 9381
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
        while True:                      # drain events until OUR result arrives
            r = json.loads(ws.recv())
            if r.get('method') == 'Runtime.exceptionThrown':
                errs.append(r['params']['exceptionDetails'].get('exception', {}).get('description', '?'))
            if r.get('id') == mid[0]:
                if 'exceptionDetails' in r.get('result', {}):
                    return {'EVAL_ERR': r['result']['exceptionDetails'].get('text')}
                return r['result']['result'].get('value')
    ev('Page.enable')
    ev('Runtime.enable')
    mid[0] += 1
    ws.send(json.dumps({'id': mid[0], 'method': 'Page.navigate',
                        'params': {'url': 'file://' + SIM + '/mesh_nav.html#single,stair,sensor'}}))
    while True:
        r = json.loads(ws.recv())
        if r.get('id') == mid[0]: break
    time.sleep(5)
    print('setup:', ev('''(()=>{const p=__MESH.pathAt(2.0);
        __MESH.drop(p[0],p[1],p[2],false);
        const b=document.getElementById("replan"); if(b) b.click();
        const g=document.getElementById("go"); if(g){g.disabled=false;g.click();}
        return "ok";})()'''))
    time.sleep(1)
    TRACE = '''(()=>{const d=__MESH.dog, st=__MESH.state; const o=st.obs[0];
        let gmin=1e9; if(st.path) for(let s=0;s<__MESH.pathLen;s+=0.1){const q=__MESH.pathAt(s);
        gmin=Math.min(gmin,Math.hypot(q[0]-o.x,q[1]-o.y));}
        return {t:+(performance.now()/1000).toFixed(1),prog:+__MESH.prog.toFixed(2),
        blocked:!!__MESH.blocked,backOut:!!st.backOut,arrived:!!st.arrived,
        gap:+(Math.hypot(d[0]-o.x,d[1]-o.y)-o.r).toFixed(3),
        gmin:+gmin.toFixed(3),leg:(st.leg||[]).length,
        lat:+st.latOff.toFixed(2),tgt:+st.latTgt.toFixed(2),win:+st.latWin.toFixed(2),
        half:+__MESH.stairWin(__MESH.prog).half.toFixed(2),stair:!!st.onStairNow,
        tracks:st.sensorTracks.map(t=>[+t.x.toFixed(2),+t.y.toFixed(2),+t.r.toFixed(2)]),
        toast:(document.getElementById("toast")||{}).textContent||""};})()'''
    rows = []
    for _ in range(100):
        time.sleep(0.3)
        rows.append(ev(TRACE))
    for r in rows:
        if not isinstance(r, dict):
            print('RAW', r); continue
        if 'EVAL_ERR' in r:
            print(r); continue
        if r['blocked'] or r['gmin'] < 0.5 or r['t'] % 3 < 0.35:
            print(r)
    print('last:', rows[-1])
    print('PAGE ERRORS:', errs[:4])
finally:
    p.terminate()