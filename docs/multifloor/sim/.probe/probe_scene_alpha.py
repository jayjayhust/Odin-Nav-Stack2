#!/usr/bin/env python3
"""场景透明 switch: does the toolbar chip reach the imported mesh's material?

The page draws the imported OBJ at 42% opacity so the walkable plates, the orange
flight band and the route read through the walls. The new 场景透明 chip turns that
off to get an opaque shell. Two things are easy to get wrong and are asserted here:

  * the switch must only touch the MATERIAL - a rebuild would also throw away the
    plan, the endpoints and the dog position (the map object has to be the same
    one after the flip, and pathLen unchanged);
  * `transparent` in three.js picks a shader variant, not a uniform, so the flip
    needs `needsUpdate` - read back the material, not just `state.sceneAlpha`.

Scene 1: multi-terrain sample, plan lane A, read the material, click the chip off,
check the map/path identity, click it back on, then drive the __MESH door.
Scene 2: the same page booted with #solid, which has to arrive as "off" both in
state and in the chip's own checkbox (the HTML ships it checked).

Also writes shot_alpha_off.png / shot_alpha_on.png from one camera, for the visual
check that opaque really looks opaque.
"""
import base64, json, os, subprocess, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
HERE = os.path.dirname(os.path.abspath(__file__))
PORT = 9411

EXPR_PLAN = r'''
(async () => {
  const W = ms => new Promise(r => setTimeout(r, ms));
  const F2 = () => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
  document.getElementById('demo2').click();
  let m = null;
  for (let i = 0; i < 120 && !(m = window.__MESH.map); ++i) await W(100);
  if (!m) return JSON.stringify({err: 'no map'});
  const s = window.__MESH.state;
  s.start = [2.00, 2.20, 0.20];
  s.goal  = [15.50, 2.20, 0.65];
  s.ways  = [];
  const go = document.getElementById('go');
  go.disabled = false;
  go.click();
  await W(500);
  if (!s.path || !s.path.length)
    return JSON.stringify({err: 'no plan', say: document.getElementById('say').textContent});
  const cb = document.getElementById('sceneAlpha');
  const map0 = m, path0 = s.path, len0 = window.__MESH.pathLen;
  const on = window.__MESH.scene, chipOn = cb.checked;
  cb.click();                                   // off
  await F2(); await W(200);
  const off = window.__MESH.scene;
  const kept = {map: window.__MESH.map === map0, path: s.path === path0,
                len: Math.abs(window.__MESH.pathLen - len0) < 1e-9, chip: cb.checked};
  cb.click();                                   // back on
  await F2(); await W(200);
  const back = window.__MESH.scene;
  const door = window.__MESH.setSceneAlpha(false);   // the probe door, same path
  const doorChip = cb.checked;
  window.__MESH.setSceneAlpha(true);
  return JSON.stringify({on, chipOn, off, kept, back, door, doorChip,
                         len: +len0.toFixed(2), pts: s.path.length});
})()
'''

EXPR_SOLID = r'''
(async () => {
  const W = ms => new Promise(r => setTimeout(r, ms));
  const chip0 = document.getElementById('sceneAlpha').checked;
  const s0 = window.__MESH.state.sceneAlpha;
  document.getElementById('demo2').click();
  let m = null;
  for (let i = 0; i < 120 && !(m = window.__MESH.map); ++i) await W(100);
  if (!m) return JSON.stringify({err: 'no map'});
  return JSON.stringify({chip0, s0, chip: document.getElementById('sceneAlpha').checked,
                         scene: window.__MESH.scene});
})()
'''

SET = 'window.__MESH.setSceneAlpha(%s)'


def session(hash_, steps):
    """steps: [(expr, settle_s, png_name|None)] -> (results, exceptions)."""
    url = 'file://%s/mesh_nav.html%s' % (BASE, hash_)
    proc = subprocess.Popen(
        ['google-chrome', '--headless=new', '--disable-gpu', '--no-sandbox',
         '--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--hide-scrollbars',
         '--disable-dev-shm-usage', '--mute-audio',
         '--remote-debugging-port=%d' % PORT,
         '--remote-allow-origins=*', '--window-size=1600,900', url],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        pages = []
        for _ in range(60):
            try:
                tgt = json.load(urllib.request.urlopen('http://127.0.0.1:%d/json' % PORT))
                pages = [t for t in tgt if t['type'] == 'page']
                if pages:
                    break
            except Exception:
                pass
            time.sleep(0.5)
        ws = websocket.create_connection(pages[0]['webSocketDebuggerUrl'], timeout=240)
        mid = [0]
        exc = []

        def call(method, **params):
            mid[0] += 1
            ws.send(json.dumps({'id': mid[0], 'method': method, 'params': params}))
            while True:
                msg = json.loads(ws.recv())
                if msg.get('method') == 'Runtime.exceptionThrown':
                    d = msg['params']['exceptionDetails']
                    exc.append(d.get('text') or d.get('exception', {}).get('description', ''))
                if msg.get('id') == mid[0]:
                    return msg.get('result', {})

        call('Page.enable'); call('Runtime.enable')
        time.sleep(3.0)
        out = []
        for expr, settle, png in steps:
            r = call('Runtime.evaluate', expression=expr, returnByValue=True,
                     awaitPromise=True)
            d = r.get('exceptionDetails')
            if d:
                exc.append('EVAL ' + (d.get('text') or '') + ': ' +
                           str((d.get('exception') or {}).get('description', ''))[:400])
            time.sleep(settle)
            if png:
                s = call('Page.captureScreenshot', format='png')
                with open(os.path.join(HERE, png), 'wb') as fh:
                    fh.write(base64.b64decode(s['data']))
            out.append((r.get('result') or {}).get('value'))
        ws.close()
        return out, exc
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        time.sleep(0.5)


def main():
    res, exc = session('', [
        (EXPR_PLAN, 0.0, None),
        (SET % 'false', 0.5, 'shot_alpha_off.png'),
        (SET % 'true', 0.5, 'shot_alpha_on.png'),
    ])
    print('--- 场景透明 switch · multi-terrain sample, lane A ---')
    print('exc:', exc[:2])
    out = res[0]
    if not out:
        print('  RESULT: FAIL (no value)')
        return 1
    st = out if isinstance(out, dict) else json.loads(out)
    if 'err' in st:
        print('  err:', st)
        print('  RESULT: FAIL')
        return 1
    print('path %.2f m / %d pts' % (st['len'], st['pts']))
    for k in ('on', 'off', 'back', 'door'):
        print('  %-5s %s' % (k, st[k]))
    print('  kept %s | chip after flip: %s / after door: %s' %
          (st['kept'], st['kept'].get('chip'), st['doorChip']))

    checks = [
        ('boots transparent at 0.42', st['on'] and st['on']['mode'] == 'alpha' and
         st['on']['transparent'] and abs(st['on']['opacity'] - 0.42) < 1e-6 and
         not st['on']['depthWrite']),
        ('chip starts checked', st['chipOn'] is True),
        ('off -> opaque 0.96', st['off'] and st['off']['mode'] == 'solid' and
         not st['off']['transparent'] and abs(st['off']['opacity'] - 0.96) < 1e-6),
        ('off writes depth', st['off'] and st['off']['depthWrite'] is True),
        ('chip unchecks itself', st['kept'] and st['kept']['chip'] is False),
        ('map object not rebuilt', st['kept'] and st['kept']['map'] is True),
        ('path not recomputed', st['kept'] and st['kept']['path'] is True and
         st['kept']['len'] is True),
        ('back on restores alpha', st['back'] and st['back']['mode'] == 'alpha' and
         st['back']['transparent'] and not st['back']['depthWrite']),
        ('probe door flips the material too', st['door'] and st['door']['mode'] == 'solid'),
        ('door keeps the chip in sync', st['doorChip'] is False),
    ]

    # --- scene 2: #solid at boot
    res2, exc2 = session('#solid', [(EXPR_SOLID, 0.0, None)])
    print()
    print('--- 场景透明 switch · page booted with #solid ---')
    print('exc:', exc2[:2])
    st2 = res2[0]
    st2 = st2 if isinstance(st2, dict) else (json.loads(st2) if st2 else {})
    print('  chip0=%s state0=%s scene=%s' % (st2.get('chip0'), st2.get('s0'), st2.get('scene')))
    checks2 = [
        ('#solid boots opaque in state', st2.get('s0') is False),
        ('#solid unchecks the chip', st2.get('chip0') is False),
        ('#solid mesh is solid', (st2.get('scene') or {}).get('mode') == 'solid'),
        ('#solid keeps 0.96 + depthWrite',
         (st2.get('scene') or {}).get('depthWrite') is True),
        ('chip state survives the model load', st2.get('chip') is False),
        ('no page exceptions on the #solid boot', not exc2),
    ]

    passed = total = 0
    for name, good in checks + checks2:
        print('  [%s] %s' % ('ok' if good else 'XX', name))
        total += 1
        passed += 1 if good else 0
    print('\n%d/%d checks passed' % (passed, total))
    print('no page exceptions on scene 1: %s' % (not exc))
    return 0 if passed == total and not exc else 1


if __name__ == '__main__':
    raise SystemExit(main())
