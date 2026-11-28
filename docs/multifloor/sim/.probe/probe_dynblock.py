#!/usr/bin/env python3
"""One-shot: does the dynBlock gate actually flip walkable, and does a plan
with a dropped post differ from one without?"""
import json, os, subprocess, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9381
url = 'file://%s/mesh_nav.html#single,floor' % BASE
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
        r = call('Runtime.evaluate', expression=expr, returnByValue=True, awaitPromise=True)
        return r.get('result', {}).get('value', r)
    time.sleep(5.0)
    print(ev('''(function(){
      var m=__MESH.map, a=__MESH.pathAt(3.0);
      var o=__MESH.drop(a[0],a[1],a[2],false);
      var i=Math.floor((o.x-m.lo[0])/m.res), j=Math.floor((o.y-m.lo[1])/m.res);
      var ks=[]; for(var k=1;k<m.nz;k++) if(m.walkable(i,j,k)) ks.push(k);
      var set=new Set(); for(var k2 of ks) set.add((k2*m.ny+j)*m.nx+i);
      m.setDynBlock(set);
      var after=[]; for(var k3=1;k3<m.nz;k3++) if(m.walkable(i,j,k3)) after.push(k3);
      m.setDynBlock(null);
      var back=[]; for(var k4=1;k4<m.nz;k4++) if(m.walkable(i,j,k4)) back.push(k4);
      return JSON.stringify({walkBefore:ks, walkDuring:after, walkAfter:back,
                             hasSetter:typeof m.setDynBlock});
    })()'''))
    print(ev('''(function(){
      var m=__MESH.map, o=__MESH.state.obs[0];
      var rr=o.r+m.infR, set=new Set();
      var i0=Math.max(0,Math.floor((o.x-rr-m.lo[0])/m.res)),
          i1=Math.min(m.nx-1,Math.ceil((o.x+rr-m.lo[0])/m.res)),
          j0=Math.max(0,Math.floor((o.y-rr-m.lo[1])/m.res)),
          j1=Math.min(m.ny-1,Math.ceil((o.y+rr-m.lo[1])/m.res));
      for(var j=j0;j<=j1;j++)for(var i=i0;i<=i1;i++){
        var dx=m.lo[0]+(i+0.5)*m.res-o.x, dy=m.lo[1]+(j+0.5)*m.res-o.y;
        if(dx*dx+dy*dy>rr*rr) continue;
        for(var k=1;k<m.nz;k++) set.add((k*m.ny+j)*m.nx+i); }
      m.setDynBlock(set);
      var p0=__MESH.path, dis=document.getElementById('go').disabled, err=null;
      try{ document.getElementById('go').click(); }catch(e){ err=e.message; }
      var g=1e9;
      for(var s=0;s<=__MESH.pathLen;s+=0.1){
        var q=__MESH.pathAt(s); var d=Math.hypot(q[0]-o.x,q[1]-o.y);
        if(d<g)g=d; }
      m.setDynBlock(null);
      return JSON.stringify({manualStamp:true, setSize:set.size,
        disabled:dis, err:err, samePath:__MESH.path===p0,
        globalMinDist:+g.toFixed(3),
        msg:document.getElementById('say').textContent});
    })()'''))
    ws.close()
finally:
    proc.terminate()
    try: proc.wait(timeout=5)
    except Exception: proc.kill()