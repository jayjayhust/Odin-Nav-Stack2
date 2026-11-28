#!/usr/bin/env python3
"""The 地形提示 panel: does a route report the ground it is ridden on?

Two scenes, one browser each:

1. the multi-terrain sample (示例 单层多地形). Plans lane A end to end - west apron
   -> four treads -> 0.60 m plate -> 12 deg ramp up -> 1.24 m dock -> ramp down ->
   0.60 m raised floor - walks it under the frame grabber (window.recHold) in
   0.30 m steps, holding each step long enough for the 300 ms anti-flicker dwell to
   commit, and reads back __MESH.terrainLog().
2. the four-storey sample via the #demo button, which is the case where the
   ground rides up the first treads of a flight one layer at a time: the panel has
   to fold those 0.15 m refinements into the last line instead of appending, and
   the boot has to raise no page exception.

Asserts, in order:
  * the log is empty right after the model loads (a new map starts clean)
  * the first entry is 平地, and the sequence contains 台阶上行, a 台面, 坡道上行,
    another 台面, 坡道下行 and a final 台面 (the raised floor)
  * every line landed in the DOM and the header's live badge shows the last one
  * no line was written while the panel was hidden, and turning the panel back
    on re-renders the retained history
  * no page exceptions
"""
import json, os, subprocess, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9407

EXPR = r'''
(async () => {
  const W = ms => new Promise(r => setTimeout(r, ms));
  const F2 = () => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
  document.getElementById('demo2').click();
  let m = null;
  for (let i = 0; i < 120 && !(m = window.__MESH.map); ++i) await W(100);
  if (!m) return JSON.stringify({err: 'no map'});
  const afterLoad = window.__MESH.terrainLog().length;
  const s = window.__MESH.state;
  // lane A: west apron -> raised floor east of the down ramp (OBJ metres, Z up;
  // the floor is z = 0 and the raised floor is +0.60)
  s.start = [2.00, 2.20, 0.20];
  s.goal  = [15.50, 2.20, 0.65];
  s.ways  = [];
  const go = document.getElementById('go');
  go.disabled = false;
  go.click();
  await W(400);
  if (!s.path || !s.path.length)
    return JSON.stringify({err: 'no plan', say: document.getElementById('say').textContent});
  const L = window.__MESH.pathLen;
  // Walk the route under the grabber: 0.30 m steps, 150 ms apart, so every
  // class held longer than two steps clears the 300 ms dwell in terrainTick.
  window.__MESH.hold(0);
  await F2(); await W(160);
  for (let v = 0.30; v <= L + 0.001; v += 0.30){
    window.__MESH.hold(v);
    await F2(); await W(150);
  }
  window.__MESH.hold(L);          // settle on the final class
  await F2(); await W(200);
  const log = window.__MESH.terrainLog();
  const panel = document.getElementById('terrainLog');
  const body = document.getElementById('terrainLogBody');
  const cb = document.getElementById('showTerrainLog');
  const shown = {disp: panel.style.display, cls: panel.className,
                 dom: body.childNodes.length,
                 badge: document.getElementById('terrainNow').textContent,
                 count: document.getElementById('terrainLogCount').textContent};
  // hiding must stop the DOM appends but not the retained history, and turning
  // it back on must re-render what was missed
  cb.checked = false; cb.onchange();
  const off = {disp: panel.style.display, dom: body.childNodes.length};
  // ride back to the start and up again while the panel is closed: the history
  // grows, the closed panel does not
  for (const v of [L - 0.3, 0.3, L - 0.6]){
    window.__MESH.hold(v);
    await F2(); await W(400);          // over the 300 ms dwell, so a class commits
  }
  const hid = {dom: body.childNodes.length, log: window.__MESH.terrainLog().length};
  cb.checked = true; cb.onchange();
  const on = {disp: panel.style.display, dom: body.childNodes.length,
              log: window.__MESH.terrainLog().length};
  return JSON.stringify({
    afterLoad: afterLoad,
    len: +L.toFixed(2), pts: s.path.length,
    seq: log.map(e => ({t: +e.t.toFixed(1), cls: e.cls, text: e.text})),
    now: window.__MESH.terrainNow,
    shown: shown, off: off, hid: hid, on: on,
  });
})()
'''


EXPR_F4 = r'''
(async () => {
  const W = ms => new Promise(r => setTimeout(r, ms));
  const F2 = () => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
  // #demo boots the four-storey sample, waits for the map, picks its own
  // endpoints (lowest standable stratum to the highest reachable one) and plans;
  // this is also the boot whose #demo,close camera block used to throw
  // ReferenceError because the map handle was out of scope there
  let m = null;
  for (let i = 0; i < 200 && !(m = window.__MESH.map); ++i) await W(100);
  if (!m) return JSON.stringify({err: 'no map'});
  for (let i = 0; i < 100 && !window.__MESH.path; ++i) await W(200);
  if (!window.__MESH.path) return JSON.stringify({err: 'no path'});
  const L = window.__MESH.pathLen;
  window.__MESH.hold(0);
  await F2(); await W(200);
  for (let v = 0.4; v <= L + 0.001; v += 0.4){
    window.__MESH.hold(v);
    await F2(); await W(150);
  }
  window.__MESH.hold(L);
  await F2(); await W(200);
  const log = window.__MESH.terrainLog();
  const body = document.getElementById('terrainLogBody');
  return JSON.stringify({
    len: +L.toFixed(2),
    seq: log.map(e => ({cls: e.cls, text: e.text,
                        kind: e.kind || null, h: e.h == null ? null : +e.h.toFixed(2)})),
    dom: [...body.childNodes].map(n => n.textContent),
    count: document.getElementById('terrainLogCount').textContent,
    badge: document.getElementById('terrainNow').textContent,
  });
})()
'''


def run(expr, wait=3.0, hash_=''):
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
        time.sleep(wait)
        r = call('Runtime.evaluate', expression=expr, returnByValue=True, awaitPromise=True)
        d = r.get('exceptionDetails')
        if d:
            exc.append('EVAL ' + (d.get('text') or '') + ': ' +
                       str((d.get('exception') or {}).get('description', ''))[:400])
        out = (r.get('result') or {}).get('value')
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
    out, exc = run(EXPR)
    print('--- 地形提示 panel · lane A of the multi-terrain sample ---')
    print('exc:', exc[:2])
    if not out:
        print('  RESULT: FAIL (no value)')
        return 1
    st = out if isinstance(out, dict) else json.loads(out)
    if 'err' in st:
        print('  err:', st)
        print('  RESULT: FAIL')
        return 1
    print('path %.2f m / %d pts | after load: %d log lines' %
          (st['len'], st['pts'], st['afterLoad']))
    for e in st['seq']:
        print('  [%6.1fs] %-8s %s' % (e['t'], e['cls'], e['text']))
    print('panel: display=%s dom=%d badge=%r count=%s' %
          (st['shown']['disp'], st['shown']['dom'], st['shown']['badge'],
           st['shown']['count']))
    print('hidden: display=%s dom=%d | while hidden: dom=%d log=%d | reopened: display=%s dom=%d log=%d' %
          (st['off']['disp'], st['off']['dom'], st['hid']['dom'], st['hid']['log'],
           st['on']['disp'], st['on']['dom'], st['on']['log']))

    cls = [e['cls'] for e in st['seq']]
    txt = ' / '.join(e['text'] for e in st['seq'])
    checks = [
        ('log empty after load', st['afterLoad'] == 0),
        ('starts on 平地', cls[:1] == ['tflat']),
        ('climbs a 台阶', '台阶上行' in txt),
        ('reports the plate as 台面', 'tplate' in cls),
        ('reports 坡道上行', '坡道上行' in txt),
        ('reports 坡道下行', '坡道下行' in txt),
        ('ends on the raised floor 台面', cls[-1:] == ['tplate']),
        ('no 低台 on this route', '低台' not in txt),
        ('every line in the DOM', st['shown']['dom'] == len(cls)),
        ('badge = last line', '台面' in st['shown']['badge']),
        ('count matches', st['shown']['count'] == str(len(cls))),
        ('panel visible by default', st['shown']['disp'] == 'flex'),
        ('hide hides it', st['off']['disp'] == 'none'),
        ('closed panel takes no lines', st['hid']['dom'] == st['off']['dom']),
        ('history still grows while closed', st['hid']['log'] > len(cls)),
        ('reopen re-renders the history', st['on']['dom'] == st['on']['log']),
        ('reopen restores it', st['on']['disp'] == 'flex'),
        ('no page exceptions', not exc),
    ]
    ok = 0
    for name, good in checks:
        print('  [%s] %s' % ('ok' if good else 'XX', name))
        ok += 1 if good else 0
    total, passed = len(checks), ok

    # --- scene 2: the four-storey sample, where a flight's first treads arrive
    # as 0.15 m plateau refinements and have to be folded into the last line.
    out2, exc2 = run(EXPR_F4, wait=9.0, hash_='#demo,close')
    print()
    print('--- 地形提示 panel · four-storey sample via #demo ---')
    print('exc:', exc2[:2])
    st2 = out2 if isinstance(out2, dict) else (json.loads(out2) if out2 else {})
    if not st2 or 'err' in st2:
        print('  err:', st2)
        print('\n%d/%d checks passed' % (passed, total + 1))
        return 1
    print('path %.2f m | %d log lines / %d DOM lines' %
          (st2['len'], len(st2['seq']), len(st2['dom'])))
    for e in st2['seq']:
        print('  %-8s %s' % (e['cls'], e['text']))
    seq2 = st2['seq']
    plates = [e for e in seq2 if e['kind'] in ('plate', 'low')]
    jitter = [abs(a['h'] - b['h']) for a, b in zip(plates, plates[1:])
              if a['h'] is not None and b['h'] is not None]
    dup = [1 for a, b in zip(seq2, seq2[1:]) if a['text'] == b['text']]
    checks2 = [
        ('walks the whole four-storey route', st2['len'] > 40),
        ('starts on 平地', seq2[:1] and seq2[0]['cls'] == 'tflat'),
        ('six flights, six 台阶 lines', sum(1 for e in seq2 if e['cls'] == 'tup') == 6),
        ('no 台面 jitter under 0.35 m', all(d > 0.35 for d in jitter)),
        ('no line repeats the one before it', not dup),
        ('every line in the DOM', len(st2['dom']) == len(seq2)),
        ('folded line shows the live height',
         seq2 and seq2[-1]['text'].strip() in st2['dom'][-1]),
        ('count matches', st2['count'] == str(len(seq2))),
        ('badge = last line', st2['badge'] == seq2[-1]['text']),
        ('no page exceptions on the #demo boot', not exc2),
    ]
    for name, good in checks2:
        print('  [%s] %s' % ('ok' if good else 'XX', name))
        passed += 1 if good else 0
    total += len(checks2)
    print('\n%d/%d checks passed' % (passed, total))
    return 0 if passed == total else 1


if __name__ == '__main__':
    raise SystemExit(main())
