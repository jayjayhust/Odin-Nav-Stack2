#!/usr/bin/env python3
"""The task badge on the dog's head: live text, really projected, pause styling.

probe_inspection.py samples the badge while the task runs and checks it against
the panel. This probe tests the three things that sampling cannot: the badge is
driven by the head's *projected* position (orbit the camera and the anchor has
to move on screen while the badge stays centred on it), a real mouse click on
暂停 flips it to the paused styling, and 继续 puts it back.

    python3 .probe/check_head_badge.py [http://127.0.0.1:8123]
"""
import base64, json, os, subprocess, sys, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9393
ROOT = (sys.argv[1] if len(sys.argv) > 1 else 'file://' + os.path.join(BASE, 'mesh_nav.html'))
URL = '%s#single,floor,insp' % ROOT.rstrip('#')


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


def main():
    log = open('/tmp/check_head_badge.log', 'w')
    proc = subprocess.Popen(
        ['google-chrome', '--headless=new', '--disable-gpu', '--no-sandbox',
         '--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--hide-scrollbars',
         '--disable-dev-shm-usage', '--remote-debugging-port=%d' % PORT,
         '--remote-allow-origins=*', '--window-size=1600,900', URL],
        stdout=subprocess.DEVNULL, stderr=log)
    reasons = []
    try:
        page = wait_page()
        ws = websocket.create_connection(page['webSocketDebuggerUrl'], timeout=40)
        mid = [0]

        def call(method, **params):
            mid[0] += 1
            ws.send(json.dumps({'id': mid[0], 'method': method, 'params': params}))
            while True:
                m = json.loads(ws.recv())
                if m.get('id') == mid[0]:
                    return m.get('result', {})

        call('Page.enable'); call('Runtime.enable')

        def ev(expr):
            r = call('Runtime.evaluate', expression=expr, returnByValue=True)
            v = (r.get('result') or {}).get('value')
            if v is None:
                return None
            try:
                return json.loads(v)
            except Exception:
                return v

        def shot(name):
            img = call('Page.captureScreenshot', format='png')
            open(os.path.join(BASE, name), 'wb').write(base64.b64decode(img['data']))

        # 1. wait for the task to actually be running with a badge on screen
        t0, st = time.time(), None
        while time.time() - t0 < 180:
            st = ev('(() => { const t = __MESH.insp, b = __MESH.taskBadge;'
                    ' return JSON.stringify({state:t && t.state, name:t && t.name,'
                    ' badge:b, panel:getComputedStyle(document.getElementById("inspPanel")).display}); })()')
            if st and st.get('state') in ('RUNNING', 'SUCCEEDED'):
                break
            time.sleep(0.5)
        if not st or st.get('state') is None:
            reasons.append('the demo task never started: %s' % st)
            return reasons
        badge = st.get('badge')
        if not badge:
            reasons.append('no badge while state=%s' % st.get('state'))
            return reasons
        print('badge', json.dumps(badge, ensure_ascii=False))
        if badge['name'] != st.get('name'):
            reasons.append('badge name=%r want %r' % (badge['name'], st.get('name')))
        if not (badge.get('line') or '').strip():
            reasons.append('badge has no status line')

        def geom(b):
            r, a, cv = b['rect'], b['anchor'], b['cview']
            return {'inside': r['l'] >= cv[0] - 1 and r['t'] >= cv[1] - 1
                    and r['l'] + r['w'] <= cv[2] + 1 and r['b'] <= cv[3] + 1,
                    'above': r['b'] <= a[1],
                    'off': abs((r['l'] + r['w'] / 2) - a[0])}

        g = geom(badge)
        if not g['inside']:
            reasons.append('badge outside the canvas: %s in %s' % (badge['rect'], badge['cview']))
        if not g['above']:
            reasons.append('badge bottom is not above the head anchor')
        if g['off'] > 2:
            reasons.append('badge not centred on the head: %.1f px off' % g['off'])
        shot('verify_inspection_head.png')

        # 2. orbit the camera with a real drag: the anchor must move, the badge
        #    must stay centred on the new anchor (it is projected, not pinned
        #    to a fixed spot on screen)
        cx, cy = int((badge['cview'][0] + badge['cview'][2]) / 2), 470
        a0 = badge['anchor']
        call('Input.dispatchMouseEvent', type='mousePressed', x=cx, y=cy,
             button='left', clickCount=1)
        for k in range(1, 9):
            call('Input.dispatchMouseEvent', type='mouseMoved', x=cx + 15 * k, y=cy,
                 button='left', buttons=1)
            time.sleep(0.05)
        call('Input.dispatchMouseEvent', type='mouseReleased', x=cx + 120, y=cy,
             button='left', clickCount=1)
        time.sleep(0.6)
        b2 = ev('JSON.stringify(__MESH.taskBadge)')
        if not b2:
            reasons.append('badge vanished after the orbit')
        else:
            a1 = b2['anchor']
            moved = abs(a1[0] - a0[0]) + abs(a1[1] - a0[1])
            print('anchor %s -> %s (%.1f px)' % (a0, a1, moved))
            if moved < 20:
                reasons.append('anchor barely moved (%.1f px) on a 120 px orbit' % moved)
            g = geom(b2)
            if g['off'] > 2 or not g['above']:
                reasons.append('after the orbit the badge lost the head: %s' % g)
            # the head must be where the dog is drawn: project the body box's
            # top centre through the same camera
            dog = ev('(() => { try { const b = __MESH.dogBox;'
                     ' if (!b) return JSON.stringify({err:"no dogBox"});'
                     ' const el = document.getElementById("cv");'
                     ' const v = new THREE.Vector3(b.ctr[0], b.ctr[1], b.hi[2] + DOG_TASK_HEAD_Z)'
                     '   .project(camera);'
                     ' const cv = el.getBoundingClientRect();'
                     ' return JSON.stringify({px:[cv.left + el.clientLeft + (v.x*0.5+0.5)*W(),'
                     '   cv.top + el.clientTop + (-v.y*0.5+0.5)*H()], box:b});'
                     ' } catch (e){ return JSON.stringify({err:String(e)}); } })()')
            if dog and not dog.get('err'):
                print('dog box', json.dumps(dog['box'], ensure_ascii=False))
            if not dog or dog.get('err'):
                reasons.append('cannot project the dog: %s' % (dog or {}).get('err'))
            elif None in dog['px']:
                reasons.append('projected head pixel is not a number: %s' % dog)
            elif (abs(dog['px'][0] - b2['anchor'][0]) > 2
                  or abs(dog['px'][1] - b2['anchor'][1]) > 2):
                reasons.append('anchor %s is not the head %s' % (b2['anchor'], dog['px']))

        # 3. pause with a real mouse click on the panel button: the badge has to
        #    switch to the paused styling, and 继续 has to switch it back
        for act, label, chip in (('pause', '暂停', '已暂停'), ('resume', '继续', '执行中')):
            btn = ev('(() => { const b = document.querySelector("#inspPanel [data-act=%s]");'
                     ' if (!b) return null; const r = b.getBoundingClientRect();'
                     ' const x = r.left + r.width/2, y = r.top + r.height/2;'
                     ' return JSON.stringify({x:x, y:y, hit:(document.elementFromPoint(x,y)||{})'
                     '.getAttribute && document.elementFromPoint(x,y).getAttribute("data-act")}); })()' % act)
            if not btn:
                reasons.append('no %s button in the panel' % act)
                break
            if btn.get('hit') != act:
                reasons.append('%s button is covered by %s' % (act, btn.get('hit')))
            for t in ('mousePressed', 'mouseReleased'):
                call('Input.dispatchMouseEvent', type=t, x=btn['x'], y=btn['y'],
                     button='left', clickCount=1)
            time.sleep(0.7)
            b = ev('JSON.stringify(__MESH.taskBadge)')
            if not b:
                reasons.append('badge gone after %s' % act)
                break
            print('%s -> %s' % (act, json.dumps({'state': b['state'], 'line': b['line'],
                                                 'pause': b['pause']}, ensure_ascii=False)))
            if b['pause'] != (act == 'pause'):
                reasons.append('badge pause class=%s after %s' % (b['pause'], act))
            if chip not in (b['state'] or ''):
                reasons.append('badge state %r has no %r after %s' % (b['state'], chip, act))
            if act == 'pause':
                if label not in (b['line'] or ''):
                    reasons.append('badge line %r does not say %r' % (b['line'], label))
                shot('verify_inspection_paused.png')

        # 4. close-up: park the camera on the dog so the badge can be eyeballed
        #    against the head it is supposed to sit on
        ev('__MESH.dbgCam(null, 2.4, 0.9, 0.3)')
        time.sleep(0.9)
        b3 = ev('JSON.stringify(__MESH.taskBadge)')
        if not b3:
            reasons.append('badge gone on the close-up')
        else:
            g = geom(b3)
            if g['off'] > 2 or not g['above']:
                reasons.append('close-up lost the head: %s' % g)
            print('close-up anchor %s rect %s' % (b3['anchor'], b3['rect']))
        shot('verify_inspection_head_closeup.png')
        return reasons
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        log.close()


if __name__ == '__main__':
    r = main()
    print('%s check_head_badge  %s' % ('PASS' if not r else 'FAIL',
                                       '; '.join(r) if r else
                                       'projected to the head, moves with the orbit, pause styling'))
    sys.exit(0 if not r else 1)
