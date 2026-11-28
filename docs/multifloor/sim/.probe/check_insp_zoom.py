#!/usr/bin/env python3
"""Click the inspection photo the way a mouse would, over the HTTP server.

probe_inspection.py calls `.click()` in JS, which skips hit-testing: a panel
that is visually there but covered by another layer would still pass. This
probe drives Input.dispatchMouseEvent at the thumbnail's centre, checks what
elementFromPoint actually returns there, and screenshots both states.

    python3 .probe/check_insp_zoom.py [http://127.0.0.1:8123]
"""
import base64, json, os, subprocess, sys, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9392
ROOT = sys.argv[1] if len(sys.argv) > 1 else 'http://127.0.0.1:8123'
URL = '%s/mesh_nav.html#single,floor,insp' % ROOT.rstrip('/')


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
    log = open('/tmp/check_insp_zoom.log', 'w')
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
            return (r.get('result') or {}).get('value')

        # wait for the capture row: the task runs the demo route on its own
        t0, box = time.time(), None
        while time.time() - t0 < 150:
            box = ev('(() => { const b = document.querySelector("#inspPanel .ithumb");'
                     ' if (!b) return null; const r = b.getBoundingClientRect();'
                     ' return JSON.stringify({x:r.left+r.width/2, y:r.top+r.height/2,'
                     ' w:r.width, h:r.height, vh:innerHeight,'
                     ' top:(document.elementFromPoint(r.left+r.width/2, r.top+r.height/2)||{})'
                     '.outerHTML.slice(0,70)}); })()')
            if box:
                break
            time.sleep(0.5)
        if not box:
            reasons.append('no capture thumbnail appeared on the page')
            return reasons
        box = json.loads(box)
        print('thumb box', box)

        # 1. a real mouse press/release at the thumbnail centre
        for t in ('mousePressed', 'mouseReleased'):
            call('Input.dispatchMouseEvent', type=t, x=box['x'], y=box['y'],
                 button='left', clickCount=1)

        after = json.loads(ev('(() => { const o = document.getElementById("inspZoom");'
                              ' const im = o.querySelector("img"); const r = im.getBoundingClientRect();'
                              ' return JSON.stringify({on:o.classList.contains("on"),'
                              ' d:getComputedStyle(o).display, z:getComputedStyle(o).zIndex,'
                              ' nat:[im.naturalWidth, im.naturalHeight],'
                              ' shown:[Math.round(r.width), Math.round(r.height)],'
                              ' complete:im.complete, src:(im.src||"").slice(0,30),'
                              ' cap:o.querySelector(".zcap").textContent}); })()') or '{}')
        print('after click', after)
        if not after.get('on') or after.get('d') != 'flex':
            reasons.append('mouse click did not open the overlay: %s' % after)
        if after.get('nat') != [640, 480]:
            reasons.append('image not decoded as 640x480: %s' % after.get('nat'))
        if min(after.get('shown') or [0, 0]) < 100:
            reasons.append('image drawn %s px — not visible' % after.get('shown'))

        img = call('Page.captureScreenshot', format='png')
        open(os.path.join(BASE, 'verify_inspection_zoom.png'), 'wb').write(
            base64.b64decode(img['data']))

        # 2. Esc closes it
        call('Input.dispatchKeyEvent', type='keyDown', key='Escape', code='Escape', windowsVirtualKeyCode=27)
        call('Input.dispatchKeyEvent', type='keyUp', key='Escape', code='Escape', windowsVirtualKeyCode=27)
        if ev('document.getElementById("inspZoom").classList.contains("on")'):
            reasons.append('Esc did not close the overlay')
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
    print('%s check_insp_zoom  %s' % ('PASS' if not r else 'FAIL',
                                      '; '.join(r) if r else 'real-mouse click opens the full frame'))
    sys.exit(0 if not r else 1)
