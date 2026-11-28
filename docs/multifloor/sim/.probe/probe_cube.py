#!/usr/bin/env python3
"""Repro the user report: on demo1 (示例 单层阶梯) a route from
start (2.53,7.40,0.13) to goal (7.82,4.13,0.13) is planned and audited against
the map-original static boxes (desk/barrel/partition/column/platform). Prints
where the global line enters each box footprint and at what height, so we can
see whether it cuts THROUGH the body or climbs ON TOP of it."""
import json, os, subprocess, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9390

# static furniture/obstacle boxes from example_single_floor.obj (name, xmin,xmax,ymin,ymax,zmin,zmax)
BOXES = [
    ('platform',   0.60, 2.00, 0.60, 2.60, 0.00, 0.60),
    ('column',     3.80, 4.20, 3.30, 3.70, 0.00, 3.40),
    ('column',     7.40, 7.80, 1.60, 2.00, 0.00, 3.40),
    ('desk',       5.20, 6.40, 5.20, 6.00, 0.00, 0.75),
    ('partition',  6.90, 8.50, 6.40, 6.55, 0.00, 1.20),
    ('barrel',     2.60, 3.00, 5.80, 6.20, 0.00, 1.00),
]

def main():
    url = 'file://%s/mesh_nav.html#single,stair' % BASE
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
        setup = ev(
            '(function(){window.NOREPLAN=1;'
            'state.start=[2.53,7.40,0.13];state.goal=[7.82,4.13,0.13];'
            'document.getElementById("go").disabled=false;'
            'document.getElementById("go").click();'
            'return JSON.stringify({len:__MESH.path.length,'
            'plen:+__MESH.pathLen.toFixed(3),msg:document.getElementById("say").textContent});})()')
        print('setup:', setup)
        path = json.loads(ev('JSON.stringify(__MESH.path)'))
        print('points:', len(path))
        # report each path point's footprint hit
        for bi, (nm, x0, x1, y0, y1, z0, z1) in enumerate(BOXES):
            hits = []
            for idx, p in enumerate(path):
                if x0 <= p[0] <= x1 and y0 <= p[1] <= y1:
                    hits.append((idx, round(p[0], 2), round(p[1], 2), round(p[2], 2)))
            if hits:
                print('BOX %-10s x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] -> %d path point(s) inside footprint'
                      % (nm, x0, x1, y0, y1, z0, z1, len(hits)))
                for h in hits[:12]:
                    print('    pt#%d (%.2f,%.2f,%.2f)' % h)
        walk = ev(
            '(function(){var m=state.map;var a=state.start,b=state.goal;var out=[];'
            'var L=Math.hypot(b[0]-a[0],b[1]-a[1]);var n=Math.ceil(L/0.1);'
            'for(var t=0;t<=n;t++){var x=a[0]+(b[0]-a[0])*t/n,y=a[1]+(b[1]-a[1])*t/n,z=a[2]+(b[2]-a[2])*t/n;'
            'var i=Math.floor((x-m.lo[0])/m.res),j=Math.floor((y-m.lo[1])/m.res),k=Math.floor((z-m.lo[2])/m.res);'
            'var v=m.data[(k*m.ny+j)*m.nx+i],vb=m.data[((k-1)*m.ny+j)*m.nx+i];'
            'out.push([+x.toFixed(2),+y.toFixed(2),k,v,vb,m.walkable(i,j,k)?1:0]);}return JSON.stringify(out);})()')
        print('occupancy walk along start->goal straight line:')
        print('  [x,y,k,data(data),dataBelow,walkable]  UNKNOWN=0 FREE=1 OCC=2 INFL=3')
        for r in json.loads(walk):
            print('  %s' % r)
        print('full path:')
        for idx, p in enumerate(path):
            print('  #%2d  [%.2f, %.2f, %.2f]' % (idx, p[0], p[1], p[2]))
        # dense clearance of the ACTUAL polyline against every static box
        # (signed: >0 keeps the body clear, <0 means the line is inside the box)
        dpath = ev(
            '(function(){var out=[];var L=__MESH.pathLen;'
            'for(var s=0;s<=L;s+=0.05){var q=__MESH.pathAt(s);out.push([q[0],q[1]]);}return JSON.stringify(out);})()')
        pts = json.loads(dpath)
        print('actual-polyline clearance vs static boxes (min gap to each footprint edge):')
        for nm, x0, x1, y0, y1, z0, z1 in BOXES:
            g = 1e9
            for (x, y) in pts:
                dx = max(x0 - x, x - x1, 0.0)
                dy = max(y0 - y, y - y1, 0.0)
                d = (dx*dx + dy*dy) ** 0.5
                inside = (x0 <= x <= x1 and y0 <= y <= y1)
                d = -d if inside else d
                if d < g: g = d
            print('  %-10s footprint x[%.2f,%.2f] y[%.2f,%.2f]: signed min gap = %+.3f m %s'
                  % (nm, x0, x1, y0, y1, g, 'PENETRATION' if g < -0.001 else ''))
        ws.close()
    finally:
        proc.terminate()
        try: proc.wait(timeout=5)
        except Exception: proc.kill()

if __name__ == '__main__':
    main()
