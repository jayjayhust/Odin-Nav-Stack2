#!/usr/bin/env python3
"""Headless CDP verification of the inspection task example (mesh_nav #single,floor,insp).

Same harness pattern as verify_sense.py / record_mesh_nav.py: chromium headless +
swiftshader, let the page's own demo timers and tick run, drive nothing except the
closing panel button, then assert the task through window.__MESH.insp.

What it proves end to end:
  - the sample file stays in the backend's own vocabulary (name / sensors /
    actions[].type) and only `target` is the sim extension
  - the inlined example task starts on its own after the boot has planned
  - one station per step, arcs monotone, first at 0 and last at the route end
  - the dog parks at each station (prog is pinned, not driven past it)
  - scan reads returns/distance, capture shoots a 640x480 JPEG through the Odin1
    head camera (mount 0.30 m ahead / body height, FOV from the sensor constants),
    speak records its text, wait just dwells
  - the row thumbnail opens the full frame in the zoom overlay and Esc closes it
  - the head badge carries the same task (name / state / station / progress), stays
    above the projected head inside the canvas, and leaves with the task
  - the task ends SUCCEEDED and the dog stays at the goal (the loop that keeps
    the plain demo running must not restart a finished inspection)
  - the panel shows every row and hides itself on close

    python3 .probe/probe_inspection.py
"""
import base64, json, os, subprocess, sys, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9381
# #single,floor sets its own start/goal and plans on the one-room sample. The
# four-floor #demo boot is NOT used here: in this tree its auto-picked start cell
# sits inside the local layer's own margin, so the dog backs out of prog 0 and
# never leaves (a pre-existing demo defect, unrelated to inspection).
HASH = 'single,floor,insp'
SHOT = 'verify_inspection.png'
START_TIMEOUT = 40.0      # model load + auto start/goal + plan + task start
RUN_TIMEOUT = 120.0       # 4 stations over the one-room route, real speed


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
    raise RuntimeError('no debuggable page')


def run():
    url = 'file://%s/mesh_nav.html#%s' % (BASE, HASH)
    log = open('/tmp/probe_inspection.log', 'w')
    proc = subprocess.Popen(
        ['google-chrome', '--headless=new', '--disable-gpu', '--no-sandbox',
         '--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--hide-scrollbars',
         '--disable-dev-shm-usage', '--remote-debugging-port=%d' % PORT,
         '--remote-allow-origins=*', '--window-size=1600,900', url],
        stdout=subprocess.DEVNULL, stderr=log)
    reasons = []
    try:
        page = wait_page()
        ws = websocket.create_connection(page['webSocketDebuggerUrl'], timeout=40)
        mid = [0]
        errors = []

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
                reasons.append('eval error: %s' % res.get('description'))
            return res.get('value')

        # 1. the task must start by itself once the demo has planned
        t0, snap = time.time(), None
        while time.time() - t0 < START_TIMEOUT:
            snap = ev('JSON.stringify({insp:__MESH.insp, pathLen:__MESH.pathLen,'
                      ' prog:__MESH.prog, panel:getComputedStyle(document.getElementById("inspPanel")).display})')
            if snap and json.loads(snap)['insp']:
                break
            time.sleep(0.5)
        snap = json.loads(snap or '{}')
        insp = snap.get('insp')
        if not insp:
            reasons.append('inspection never started')
            return False, reasons, errors

        if insp['task_id'] != 'insp_mesh_demo_001':
            reasons.append('task_id=%s' % insp['task_id'])
        if insp['steps'] != 4:
            reasons.append('steps=%s want 4' % insp['steps'])
        arcs = insp['arcs']
        if len(arcs) != 4 or any(b < a - 1e-6 for a, b in zip(arcs, arcs[1:])):
            reasons.append('arcs not monotone: %s' % arcs)
        if arcs and abs(arcs[0]) > 1e-6:
            reasons.append('first arc %s want 0' % arcs[0])
        if arcs and abs(arcs[-1] - snap['pathLen']) > 0.05:
            reasons.append('last arc %s want pathLen %s' % (arcs[-1], snap['pathLen']))
        if snap['panel'] != 'flex':
            reasons.append('panel display=%s' % snap['panel'])

        # 1b. the sample file must stay in the backend's own vocabulary, or the
        #     "same file goes to the robot" claim in the README stops being true.
        task = json.loads(ev('JSON.stringify(__MESH.inspTask())') or '{}')
        allowed = {'measure_scan', 'capture_image', 'capture_burst', 'speak', 'wait'}
        sensors = {'camera', 'lidar_scan'}
        if not (task.get('metadata') or {}).get('note'):
            reasons.append('task has no metadata.note (backend rejects an unknown top-level note)')
        for st in task.get('steps') or []:
            sid = st.get('step_id')
            if 'name' not in st:
                reasons.append('%s has no name' % sid)
            if str(st.get('name') or '').strip() == '':
                reasons.append('%s has an empty name' % sid)
            types = [a.get('type') for a in st.get('actions') or []]
            if not types:
                reasons.append('%s has no actions' % sid)
            bad = [t for t in types if t not in allowed]
            if bad:
                reasons.append('%s action type(s) not in the backend set: %s' % (sid, bad))
            bad = [s for s in st.get('sensors') or [] if s not in sensors]
            if bad:
                reasons.append('%s sensor(s) not in the backend set: %s' % (sid, bad))
            if 'target' not in st:
                reasons.append('%s has no target' % sid)

        # 2. the dog must actually park at a station: prog stays at the arc while
        #    the step is held, and the row for that station is marked running.
        parked = False
        t0 = time.time()
        while time.time() - t0 < RUN_TIMEOUT:
            got = json.loads(ev('JSON.stringify({insp:__MESH.insp, prog:__MESH.prog})') or '{}')
            i = got.get('insp') or {}
            if i.get('hold'):
                a = ev('JSON.stringify({prog:__MESH.prog, arc:%s})' % json.dumps(i['arcs'][i['idx']]))
                p1 = json.loads(a or '{}')
                time.sleep(0.5)
                p2 = json.loads(ev('JSON.stringify({prog:__MESH.prog})') or '{}')
                parked = (p1.get('prog') is not None and abs(p1['prog'] - p1['arc']) < 0.02
                          and abs(p2.get('prog', -9) - p1['prog']) < 0.02)
                break
            if i.get('state') in ('SUCCEEDED', 'INTERRUPTED', 'FAILED'):
                break
            time.sleep(0.4)
        if not parked:
            reasons.append('never observed a parked station')

        # 3. let it finish (sampling the head badge while it runs)
        t0, done, lines, badge_mid = time.time(), None, set(), None
        while time.time() - t0 < RUN_TIMEOUT:
            done = json.loads(ev('JSON.stringify({insp:__MESH.insp, prog:__MESH.prog,'
                                 ' badge:__MESH.taskBadge,'
                                 ' panel:getComputedStyle(document.getElementById("inspPanel")).display,'
                                 ' rows:document.querySelectorAll("#inspPanel .irow").length,'
                                 ' thumbs:document.querySelectorAll("#inspPanel .ithumb").length})') or '{}')
            bd = done.get('badge')
            if bd and (bd.get('line') or '').strip():
                lines.add(bd['line'])
                if badge_mid is None:
                    badge_mid = bd
            state = (done.get('insp') or {}).get('state')
            if state in ('SUCCEEDED', 'INTERRUPTED', 'FAILED'):
                break
            time.sleep(0.5)
        i = done.get('insp') or {}
        if i.get('state') != 'SUCCEEDED':
            reasons.append('task state=%s msg=%s' % (i.get('state'), i.get('message')))

        report = i.get('report') or []
        acts = [r['action'] for r in report]
        if acts != ['scan', 'wait', 'capture', 'speak']:
            reasons.append('actions=%s' % acts)
        scan = next((r for r in report if r['action'] == 'scan'), None)
        if not scan or not scan['measured']:
            reasons.append('scan has no measurement')
        elif not scan['measured'].get('returns'):
            reasons.append('scan returns=0')
        cap = next((r for r in report if r['action'] == 'capture'), None)
        if not cap or not cap['artifact']:
            reasons.append('capture has no artifact')
        else:
            art = cap['artifact']
            if art.get('kind') != 'image/jpeg':
                reasons.append('artifact kind=%s' % art.get('kind'))
            # the photo must come from the dog's Odin1, not from the viewport
            if art.get('view') != 'odin1':
                reasons.append('artifact view=%s want odin1' % art.get('view'))
            if art.get('bytes', 0) < 2000:
                reasons.append('artifact bytes=%s' % art.get('bytes'))
            if (art.get('w'), art.get('h')) != (640, 480):
                reasons.append('artifact size=%sx%s' % (art.get('w'), art.get('h')))
            if not 90 <= (art.get('fov') or 0) <= 110:
                reasons.append('artifact fov=%s want ~100 (120x90 pattern, 4:3)' % art.get('fov'))
            # the mount geometry itself: 0.30 m ahead of the body, 0.30 m up
            sh = art.get('shot') or {}
            pos = sh.get('pos') or [0, 0, 0]
            ahead = ((pos[0] - cap['pose'][0])**2 + (pos[1] - cap['pose'][1])**2) ** 0.5
            dz = pos[2] - cap['pose'][2]
            if not 0.2 <= ahead <= 0.45:
                reasons.append('camera %.2f m ahead of the body, want ~0.30' % ahead)
            if not 0.15 <= dz <= 0.45:
                reasons.append('camera %.2f m above the body, want ~0.30' % dz)
        spk = next((r for r in report if r['action'] == 'speak'), None)
        if not spk or '巡检完成' not in (spk.get('text') or ''):
            reasons.append('speak text=%s' % (spk or {}).get('text'))
        bad = [r['step_id'] for r in report if r['state'] not in ('OK', 'WARN')]
        if bad:
            reasons.append('steps not OK/WARN: %s' % bad)

        if done.get('rows') != 4:
            reasons.append('panel rows=%s want 4' % done.get('rows'))
        if done.get('thumbs') != 1:
            reasons.append('panel thumbs=%s want 1' % done.get('thumbs'))
        # the panel must not be cut off by the canvas edge: its bottom stays
        # inside the window, and head/foot survive even when the list shrinks
        geo = json.loads(ev('(() => { const p = document.getElementById("inspPanel");\n'
                            '  const r = p.getBoundingClientRect();\n'
                            '  return JSON.stringify({bottom:r.bottom, vh:innerHeight,\n'
                            '    head:!!p.querySelector(".ihead"), foot:!!p.querySelector(".ifoot"),\n'
                            '    listClipped:p.querySelector(".ilist").scrollHeight >\n'
                            '      p.querySelector(".ilist").clientHeight}); })()') or '{}')
        if geo.get('bottom', 1e9) > geo.get('vh', 0) + 1:
            reasons.append('panel bottom %s below viewport %s' % (geo.get('bottom'), geo.get('vh')))
        if not geo.get('head') or not geo.get('foot'):
            reasons.append('panel lost head/foot: %s' % geo)

        # 3a. the head badge: same task as the panel, live text while running,
        #     pinned above the projected head and inside the canvas
        if badge_mid is None:
            reasons.append('no head badge during the run')
        else:
            if badge_mid.get('name') != i.get('name'):
                reasons.append('badge name=%s want %s' % (badge_mid.get('name'), i.get('name')))
            if not any(k in (badge_mid.get('line') or '')
                       for k in ('前往', '拍照', '扫描', '喊话', '驻留')):
                reasons.append('badge line is not live: %s' % badge_mid.get('line'))
            r = badge_mid.get('rect') or {}
            a = badge_mid.get('anchor') or [0, 0]
            cv = badge_mid.get('cview') or [0, 0, 0, 0]
            if not (r.get('l', -1) >= cv[0] - 1 and r.get('t', -1) >= cv[1] - 1
                    and r.get('l', 0) + r.get('w', 0) <= cv[2] + 1
                    and r.get('b', 1e9) <= cv[3] + 1):
                reasons.append('badge outside the canvas: %s in %s' % (r, cv))
            if r.get('b', 0) > a[1]:
                reasons.append('badge bottom %.1f is not above the head anchor %.1f'
                               % (r.get('b', 0), a[1]))
            if abs(r.get('l', 0) + r.get('w', 0) / 2 - a[0]) > 2:
                reasons.append('badge not centred on the head: rect=%s anchor=%s' % (r, a))
            if not lines:
                reasons.append('badge never showed a status line')
        fin = done.get('badge') or {}
        if not fin:
            reasons.append('head badge gone while the task is still on screen')
        else:
            if '已完成' not in (fin.get('state') or ''):
                reasons.append('badge state at the end: %s' % fin.get('state'))
            if '4/4' not in (fin.get('state') or ''):
                reasons.append('badge progress at the end: %s' % fin.get('state'))
            if (fin.get('bar') or '') != '100%':
                reasons.append('badge bar at the end: %s' % fin.get('bar'))

        # 3b. the row thumbnail must open the full frame at full size, and Esc
        #     must close that layer instead of leaving a stuck overlay
        zoom = json.loads(ev('(() => { const b = document.querySelector("#inspPanel .ithumb");\n'
                             '  if (!b) return JSON.stringify({thumb:false});\n'
                             '  b.click();\n'
                             '  const o = document.getElementById("inspZoom");\n'
                             '  const im = o.querySelector("img"), r = im.getBoundingClientRect();\n'
                             '  return JSON.stringify({thumb:true, on:o.classList.contains("on"),\n'
                             '    w:im.naturalWidth, h:im.naturalHeight,\n'
                             '    jpeg:(im.src||"").indexOf("data:image/jpeg")===0,\n'
                             '    shown:[Math.round(r.width), Math.round(r.height)],\n'
                             '    cap:o.querySelector(".zcap").textContent}); })()') or '{}')
        if not zoom.get('thumb'):
            reasons.append('no capture thumbnail in the panel')
        else:
            if not zoom.get('on'):
                reasons.append('thumbnail click did not open the zoom overlay')
            if (zoom.get('w'), zoom.get('h')) != (640, 480):
                reasons.append('zoomed image %sx%s want 640x480' % (zoom.get('w'), zoom.get('h')))
            if not zoom.get('jpeg'):
                reasons.append('zoomed image is not a JPEG data url')
            if (zoom.get('shown') or [0])[0] < 300:
                reasons.append('zoom drawn %spx wide, too small to read' % (zoom.get('shown') or [0])[0])
            if 'Odin1' not in (zoom.get('cap') or '') or '640×480' not in (zoom.get('cap') or ''):
                reasons.append('zoom caption=%s' % zoom.get('cap'))
            if zoom.get('on'):
                zimg = call('Page.captureScreenshot', format='png')
                open(os.path.join(BASE, 'verify_inspection_zoom.png'), 'wb').write(
                    base64.b64decode(zimg['data']))
            ev('window.dispatchEvent(new KeyboardEvent("keydown", {key:"Escape", bubbles:true}))')
            if ev('document.getElementById("inspZoom").classList.contains("on")'):
                reasons.append('Esc did not close the zoom overlay')

        # 4. a finished inspection must stay parked at the goal, not loop
        p1 = json.loads(ev('JSON.stringify({prog:__MESH.prog, len:__MESH.pathLen})') or '{}')
        time.sleep(2.0)
        p2 = json.loads(ev('JSON.stringify({prog:__MESH.prog})') or '{}')
        if p1.get('prog', 0) < p1.get('len', 1e9) - 0.2:
            reasons.append('stopped short: prog=%s len=%s' % (p1.get('prog'), p1.get('len')))
        if p2.get('prog') is not None and p2['prog'] < p1.get('prog', 0) - 0.5:
            reasons.append('looped after finishing: %s -> %s' % (p1.get('prog'), p2.get('prog')))

        img = call('Page.captureScreenshot', format='png')
        open(os.path.join(BASE, SHOT), 'wb').write(base64.b64decode(img['data']))

        # 5. close the panel; the block must fail closed (display:none, no task)
        ev('document.querySelector("#inspPanel button[data-act=close]").click()')
        after = json.loads(ev('JSON.stringify({insp:__MESH.insp,'
                              ' panel:getComputedStyle(document.getElementById("inspPanel")).display})') or '{}')
        if after.get('insp') is not None or after.get('panel') != 'none':
            reasons.append('close left insp=%s panel=%s' % (after.get('insp') is not None, after.get('panel')))
        # the head badge lives off the same state, so it must go with the task
        # (one frame later at most - the tick is what hides it)
        t0, gone = time.time(), False
        while time.time() - t0 < 2.0:
            g = json.loads(ev('JSON.stringify({badge:__MESH.taskBadge,'
                              ' on:document.getElementById("dogTask").classList.contains("on")})') or '{}')
            if g.get('badge') is None and not g.get('on'):
                gone = True
                break
            time.sleep(0.1)
        if not gone:
            reasons.append('head badge left on screen after closing the task')

        ws.close()
        real_errors = [e for e in errors if 'favicon' not in e.lower()]
        if real_errors:
            reasons.append('console errors: %s' % real_errors[:2])
        return not reasons, reasons, errors
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        log.close()
        time.sleep(0.5)


if __name__ == '__main__':
    ok, reasons, errors = run()
    print('%s probe_inspection  arcs/report/panel %s' % ('PASS' if ok else 'FAIL',
                                                         '; '.join(reasons) if reasons else 'all checks green'))
    if errors:
        print('console:', errors[:3])
    sys.exit(0 if ok else 1)
