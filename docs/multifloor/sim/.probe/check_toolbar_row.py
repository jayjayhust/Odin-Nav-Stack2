#!/usr/bin/env python3
"""Headless check of the route toolbar row (#single,floor boot).

Guards the row rework: start/goal are one segmented pair whose halves light up
when armed and carry a "set" dot, group dividers are real rules instead of "|"
text, 清途经点 is gone from the toolbar and the only way to clear waypoints is
the ways panel's 清空全部 button.

Also guards the width budget: the row is two .cluster blocks (编辑 / 导出) and
must never scroll sideways - the macOS overlay scrollbar is invisible, so a
1366 px window used to put 导出 pgm / 导出梯段 out of reach. From 1440 px up the
two blocks share one line; below that the export block wraps as a whole.

    python3 .probe/check_toolbar_row.py
"""
import base64, json, os, re, subprocess, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9391
SHOT = '/tmp/check_toolbar_row.png'


def wait_page():
    for _ in range(60):
        try:
            tgt = json.load(urllib.request.urlopen('http://127.0.0.1:%d/json' % PORT))
            pages = [t for t in tgt if t['type'] == 'page']
            if pages:
                return pages[0]
        except Exception:
            pass
        time.sleep(0.5)
    raise RuntimeError('no page')


def run():
    proc = subprocess.Popen(
        ['google-chrome', '--headless=new', '--disable-gpu', '--no-sandbox',
         '--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--hide-scrollbars',
         '--disable-dev-shm-usage', '--remote-debugging-port=%d' % PORT,
         '--remote-allow-origins=*', '--window-size=1600,900',
         'file://%s/mesh_nav.html#single,floor' % BASE],
        stdout=subprocess.DEVNULL, stderr=open('/tmp/check_toolbar_row.log', 'w'))
    bad = []
    try:
        page = wait_page()
        ws = websocket.create_connection(page['webSocketDebuggerUrl'], timeout=40)
        mid, errors = [0], []

        def call(method, **params):
            mid[0] += 1
            ws.send(json.dumps({'id': mid[0], 'method': method, 'params': params}))
            while True:
                m = json.loads(ws.recv())
                if m.get('method') == 'Runtime.exceptionThrown':
                    d = m['params']['exceptionDetails']
                    errors.append(d.get('exception', {}).get('description', d.get('text', '?')))
                if m.get('method') == 'Log.entryAdded' and m['params']['entry']['level'] == 'error':
                    errors.append(m['params']['entry']['text'])
                if m.get('id') == mid[0]:
                    return m.get('result', {})

        call('Page.enable'); call('Runtime.enable'); call('Log.enable')

        def ev(expr):
            r = call('Runtime.evaluate', expression=expr, returnByValue=True, awaitPromise=True)
            res = r.get('result', {})
            if res.get('subtype') == 'error':
                bad.append('eval: %s' % res.get('description'))
            return res.get('value')

        # the #single,floor boot plans by itself, so both ends end up set
        t0 = time.time()
        while time.time() - t0 < 40:
            if ev('!!(window.__MESH && __MESH.map && __MESH.pathLen > 0.5)'):
                break
            time.sleep(0.5)
        time.sleep(0.6)

        # 1. structure, set-state dots, hit-testing, row width
        s = json.loads(ev(r'''JSON.stringify((() => {
          const ss = document.getElementById('setStart'), sg = document.getElementById('setGoal');
          const r = el => { const b = el.getBoundingClientRect();
                            return {l:Math.round(b.left), t:Math.round(b.top),
                                    w:Math.round(b.width), h:Math.round(b.height)}; };
          const hit = el => { const b = el.getBoundingClientRect();
                              const e = document.elementFromPoint(b.left + b.width/2, b.top + b.height/2);
                              return e ? (e.id || e.tagName + '.' + e.className) : null; };
          const bars = document.querySelectorAll('.bar');
          const b2 = bars[1], br = b2.getBoundingClientRect();
          return {
            ssCls: ss.className, sgCls: sg.className, ssTitle: ss.title,
            segCls: ss.parentElement.className, segId: ss.parentElement.id,
            ssHit: hit(ss), sgHit: hit(sg), ssBox: r(ss), segBox: r(ss.parentElement),
            clearWayGone: document.getElementById('clearWay') === null,
            seps: document.querySelectorAll('.bar .sep').length,
            pipes: Array.from(document.querySelectorAll('.bar .lbl'))
                     .filter(e => e.textContent.trim() === '|').length,
            routeRight: Math.round(document.getElementById('inspDemo').getBoundingClientRect().right),
            barRight: Math.round(br.right),
            row2: {sw: b2.scrollWidth, cw: b2.clientWidth},
            clearLabel: document.getElementById('clear').textContent
          };
        })())''') or '{}')

        if 'grp' not in s.get('segCls', ''):
            bad.append('start/goal not one .grp: %s' % s.get('segCls'))
        if 'has' not in s.get('ssCls', '') or 'has' not in s.get('sgCls', ''):
            bad.append('set dots missing: %s / %s' % (s.get('ssCls'), s.get('sgCls')))
        if '(' not in s.get('ssTitle', ''):
            bad.append('setStart title has no coords: %s' % s.get('ssTitle'))
        if s.get('ssHit') != 'setStart' or s.get('sgHit') != 'setGoal':
            bad.append('pair not hit-testable: %s / %s' % (s.get('ssHit'), s.get('sgHit')))
        if not s.get('clearWayGone'):
            bad.append('#clearWay still in the DOM')
        if s.get('pipes'):
            bad.append('%s "|" text separators left' % s.get('pipes'))
        if s.get('seps', 0) < 3:
            bad.append('only %s .sep dividers' % s.get('seps'))
        # the pair must keep its own two-button width: overflow:hidden gave it a
        # zero automatic min size and the flex row squashed it to 2 px once
        seg = s.get('segBox') or {}
        if not 120 <= seg.get('w', 0) <= 200:
            bad.append('segmented pair width=%s' % seg.get('w'))
        # the route cluster (设起点 … 巡检示例) is what the row is for: it must be
        # on screen without scrolling, whatever the export controls to its right do
        if s.get('routeRight', 1e9) > s.get('barRight', 0):
            bad.append('route cluster clipped: %s > %s' % (s.get('routeRight'), s.get('barRight')))
        if s.get('clearLabel') != '清空场景':
            bad.append('clear label=%s' % s.get('clearLabel'))
        print('row:', json.dumps({k: s.get(k) for k in
              ('ssCls', 'sgCls', 'ssHit', 'sgHit', 'seps', 'segBox', 'ssBox',
               'routeRight', 'barRight', 'row2')}, ensure_ascii=False))

        # 2. the armed half lights up, and the two are exclusive
        ev("document.getElementById('setStart').click()")
        a = json.loads(ev("JSON.stringify({s:setStart.className,g:setGoal.className,w:setWay.className})") or '{}')
        if 'on' not in a.get('s', ''):
            bad.append('armed start not highlighted: %s' % a.get('s'))
        if 'on' in a.get('g', '') or 'on' in a.get('w', ''):
            bad.append('other modes lit too: %s / %s' % (a.get('g'), a.get('w')))
        ev("document.getElementById('setGoal').click()")
        a = json.loads(ev("JSON.stringify({s:setStart.className,g:setGoal.className})") or '{}')
        if 'on' in a.get('s', '') or 'on' not in a.get('g', ''):
            bad.append('start/goal not exclusive: %s / %s' % (a.get('s'), a.get('g')))

        # 3. way mode arms its own button
        ev("document.getElementById('setWay').click()")
        a = json.loads(ev("JSON.stringify({s:setStart.className,g:setGoal.className,w:setWay.className})") or '{}')
        if 'on' not in a.get('w', '') or 'on' in a.get('s', '') + a.get('g', ''):
            bad.append('way highlight wrong: %s' % a)

        # 4. clearing waypoints now lives in the ways panel only
        ev(r'''(() => {
          const cv = document.getElementById('cv'), b = cv.getBoundingClientRect();
          for (const [fx, fy] of [[0.42, 0.55], [0.55, 0.5]])
            cv.dispatchEvent(new MouseEvent('click', {clientX: b.left + b.width*fx,
              clientY: b.top + b.height*fy, bubbles: true, cancelable: true}));
          return true;
        })()''')
        time.sleep(0.4)
        w = json.loads(ev(r'''JSON.stringify((() => {
          const clr = document.querySelector('#waysPanel .wclear');
          return {rows: document.querySelectorAll('#waysPanel .wrow').length,
                  has: !!clr, label: clr ? clr.textContent : null};
        })())''') or '{}')
        if w.get('rows', 0) < 1:
            bad.append('canvas click added no waypoint row: %s' % w)
        if not w.get('has') or w.get('label') != '清空全部':
            bad.append('ways panel clear: %s' % w)

        ev(r'''(() => { const b = document.querySelector('#waysPanel .wclear').getBoundingClientRect();
          document.elementFromPoint(b.left + b.width/2, b.top + b.height/2).click();
          return true; })()''')
        time.sleep(0.3)
        after = json.loads(ev(r'''JSON.stringify({
          rows: document.querySelectorAll('#waysPanel .wrow').length,
          clear: !!document.querySelector('#waysPanel .wclear'),
          say: document.getElementById('say').textContent})''') or '{}')
        if after.get('rows') != 0 or after.get('clear'):
            bad.append('panel clear left %s rows / button=%s' % (after.get('rows'), after.get('clear')))

        img = call('Page.captureScreenshot', format='png')
        open(SHOT, 'wb').write(base64.b64decode(img['data']))

        # 5. the floor list still drives the export, just in less space: 自动 first,
        #    then "第 n 层 z0–z1 m" (the voxel row range went to the option title)
        fl = json.loads(ev("JSON.stringify({v:document.getElementById('expFloor').value,"
                           "o:[].map.call(document.getElementById('expFloor').options,"
                           "function(x){return x.textContent})})") or '{}')
        print('  expFloor:', json.dumps(fl, ensure_ascii=False))
        opts = fl.get('o') or []
        if fl.get('v') != 'auto' or not opts or opts[0] != '自动' or len(opts) < 2:
            bad.append('floor list = %s' % fl)
        if len(opts) > 1 and not re.match(r'^第 \d+ 层 \d+\.\d\d–\d+\.\d\d m$', opts[1]):
            bad.append('floor option text = %s' % opts[1])

        # 6. width budget: never sideways, one line from 1440 up, and the export
        #    controls must be clickable (not just present) at every width
        WIDTH_SWEEP = r'''JSON.stringify((() => {
          const bar = document.querySelectorAll('.bar')[1];
          const watch = ['mapName', 'expFloor', 'expMode', 'expPgm', 'expStairs'];
          return {
            sw: bar.scrollWidth, cw: bar.clientWidth,
            h: Math.round(bar.getBoundingClientRect().height),
            docW: document.documentElement.scrollWidth, vw: innerWidth,
            dead: watch.filter(id => {
              const e = document.getElementById(id), r = e.getBoundingClientRect();
              if (r.width < 8 || r.right > innerWidth + 1) return true;
              const hit = document.elementFromPoint(r.left + r.width/2, r.top + r.height/2);
              return hit !== e && !e.contains(hit);
            })
          };
        })())'''
        for w in (1200, 1366, 1440, 1600, 1920):
            call('Emulation.setDeviceMetricsOverride', width=w, height=900,
                 deviceScaleFactor=1, mobile=False)
            time.sleep(0.5)
            m = json.loads(ev(WIDTH_SWEEP) or '{}')
            print('  %dpx: sw=%s cw=%s h=%s doc=%s/%s dead=%s' %
                  (w, m.get('sw'), m.get('cw'), m.get('h'), m.get('docW'),
                   m.get('vw'), m.get('dead')))
            if m.get('sw', 0) > m.get('cw', 0) + 1:
                bad.append('%dpx: row overflows sw=%s cw=%s' % (w, m.get('sw'), m.get('cw')))
            if m.get('dead'):
                bad.append('%dpx: unreachable %s' % (w, m.get('dead')))
            if m.get('docW', 0) > m.get('vw', 0) + 1:
                bad.append('%dpx: page scrolls sideways' % w)
            if w >= 1440 and m.get('h', 0) > 56:
                bad.append('%dpx: row spilled to %s px tall' % (w, m.get('h')))
            if w <= 1366 and m.get('h', 0) <= 56:
                bad.append('%dpx: export block did not wrap (%s px)' % (w, m.get('h')))
        call('Emulation.clearDeviceMetricsOverride')

        ws.close()
        real = [e for e in errors if 'favicon' not in e.lower()]
        if real:
            bad.append('console errors: %s' % real[:3])
        return bad
    finally:
        proc.terminate()


if __name__ == '__main__':
    bad = run()
    print(('FAIL ' + ' | '.join(bad)) if bad else 'PASS check_toolbar_row')
