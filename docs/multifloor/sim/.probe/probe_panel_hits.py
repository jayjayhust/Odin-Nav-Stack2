#!/usr/bin/env python3
"""左栏面板：默认状态、排列顺序，以及按钮是不是真的点得到？

`#leftCol` 是 pointer-events:none 的（左栏空白处要能把拖拽转给画布），所以每个
面板必须自己声明 pointer-events:auto，否则它的标题栏、收起按钮、清空按钮全部
"看得见、点不到"——程序里调 .onclick() 一切正常，用户拿鼠标点却没反应，正是
地形提示收起按钮报过的那个 bug（#terrainLog 是在左栏改成 flex 之后才加的，漏了）。

这个探针用 CDP 派发真实鼠标事件（不是调 .onclick()），按下/抬起都走浏览器的命中
测试，断言：
  * 默认状态：地图信息与地形提示收起（只留标题栏），脱困日志展开；
  * 排列顺序：地图信息 → 地形提示 → 脱困日志（脱困日志在地形提示**下面**）；
  * 每个控件的中心 elementFromPoint 就是它自己；
  * 三个 ▾/▸ 按钮各点两次：类名、正文可见性、箭头三者同步翻转；
  * 地形提示的 清空 真的清掉记录；
  * 左栏面板之间的缝隙仍然穿到画布（没有把整列都变成挡板）。
"""
import base64
import json
import os
import subprocess
import time
import urllib.request

import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PORT = 9413

# 规划一条道，让地形提示面板里真的有几行可清。
EXPR_SETUP = r'''
(async () => {
  const W = ms => new Promise(r => setTimeout(r, ms));
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
  await W(400);
  // 脱困日志默认关着，先打开，好一起测它的按钮
  const e = document.getElementById('showEscLog');
  if (!e.checked){ e.checked = true; e.onchange(); }
  await W(120);

  const centre = sel => {
    const el = document.querySelector(sel);
    if (!el) return null;
    const r = el.getBoundingClientRect();
    if (!r.width || !r.height) return null;
    const x = r.left + r.width/2, y = r.top + r.height/2;
    const hit = document.elementFromPoint(x, y);
    return {x: +x.toFixed(1), y: +y.toFixed(1),
            hit: hit ? (hit.id || hit.className || hit.tagName) : null,
            self: !!hit && (hit === el || el.contains(hit))};
  };
  const snap = id => {
    const p = document.getElementById(id);
    const b = document.getElementById(id + 'Body');
    return {cls: p.className, h: +p.getBoundingClientRect().height.toFixed(1),
            body: b ? getComputedStyle(b).display : null,
            disp: getComputedStyle(p).display};
  };
  const rect = id => {
    const r = document.getElementById(id).getBoundingClientRect();
    return {top: +r.top.toFixed(1), bottom: +r.bottom.toFixed(1)};
  };
  // 面板之间的缝隙（gap 8 px）应当还是画布；整列被面板填满，只在这里能看到底
  const tl = rect('terrainLog'), el = rect('escLog');
  const gapMid = (tl.bottom + el.top) / 2;
  const col = document.getElementById('leftCol').getBoundingClientRect();
  const gap = document.elementFromPoint(col.left + 6, gapMid);

  return JSON.stringify({
    lines: document.getElementById('terrainLogBody').children.length,
    count: document.getElementById('terrainLogCount').textContent,
    hit: {
      hudToggle:      centre('#hudToggle'),
      terrainLogMin:  centre('#terrainLogMin'),
      terrainLogClear:centre('#terrainLogClear'),
      escLogMin:      centre('#escLogMin'),
      escLogClear:    centre('#escLogClear'),
    },
    order: [...document.getElementById('leftCol').children].map(el => el.id),
    terrain: rect('terrainLog'), esc: rect('escLog'),
    hud: snap('hud'), terrainLog: snap('terrainLog'), escLog: snap('escLog'),
    gap: gap ? (gap.id || gap.className || gap.tagName) : null,
  });
})()
'''

# 每次点击前重新量坐标：地形提示收/展会改变下方脱困日志的高度，把上方的面板
# 整体挪一段——拿旧坐标点第二下会戳到隔壁面板上。
EXPR_CENTRE = r'''
JSON.stringify((sel => {
  const el = document.querySelector(sel);
  if (!el) return null;
  const r = el.getBoundingClientRect();
  if (!r.width || !r.height) return null;
  const x = r.left + r.width/2, y = r.top + r.height/2;
  const hit = document.elementFromPoint(x, y);
  return {x: +x.toFixed(1), y: +y.toFixed(1),
          hit: hit ? (hit.id || hit.className || hit.tagName) : null,
          self: !!hit && (hit === el || el.contains(hit))};
})(%s))
'''

# 一个面板此刻的样子：类名 / 高度 / 标题栏高度 / 正文 display / 箭头
EXPR_PANEL = r'''
JSON.stringify((([btn, pan, body]) => {
  const b = document.querySelector(btn), p = document.querySelector(pan);
  const bd = document.querySelector(body);
  const head = p.querySelector('.ehd, #hudHead');
  return {cls: p.className,
          clsSet: p.className.split(/\s+/).filter(Boolean),
          h: +p.getBoundingClientRect().height.toFixed(1),
          headH: head ? +head.getBoundingClientRect().height.toFixed(1) : 0,
          body: getComputedStyle(bd).display,
          icon: b.textContent.trim(),
          count: (document.getElementById('terrainLogCount') || {}).textContent,
          first: (document.getElementById('terrainLogBody').firstChild || {}).className || ''};
})(%s))
'''

PANELS = {
    'hudToggle':     ['#hudToggle', '#hud', '#hudBody', 'col'],
    'terrainLogMin': ['#terrainLogMin', '#terrainLog', '#terrainLogBody', 'min'],
    'escLogMin':     ['#escLogMin', '#escLog', '#escLogBody', 'min'],
    'terrainLogClear': ['#terrainLogClear'],   # 只有坐标，没有面板状态可读
}


class Page(object):
    def __init__(self, hash_):
        url = 'file://%s/mesh_nav.html%s' % (BASE, hash_)
        self.proc = subprocess.Popen(
            ['google-chrome', '--headless=new', '--disable-gpu', '--no-sandbox',
             '--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--hide-scrollbars',
             '--disable-dev-shm-usage', '--mute-audio',
             '--remote-debugging-port=%d' % PORT,
             '--remote-allow-origins=*', '--window-size=1600,900', url],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
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
        self.ws = websocket.create_connection(pages[0]['webSocketDebuggerUrl'], timeout=240)
        self.mid = 0
        self.exc = []
        self.call('Page.enable')
        self.call('Runtime.enable')
        time.sleep(3.0)

    def call(self, method, **params):
        self.mid += 1
        self.ws.send(json.dumps({'id': self.mid, 'method': method, 'params': params}))
        while True:
            msg = json.loads(self.ws.recv())
            if msg.get('method') == 'Runtime.exceptionThrown':
                d = msg['params']['exceptionDetails']
                self.exc.append(d.get('text') or
                                (d.get('exception') or {}).get('description', ''))
            if msg.get('id') == self.mid:
                return msg.get('result', {})

    def js(self, expr):
        r = self.call('Runtime.evaluate', expression=expr, returnByValue=True,
                      awaitPromise=True)
        d = r.get('exceptionDetails')
        if d:
            self.exc.append('EVAL ' + str((d.get('exception') or {}).get('description', ''))[:300])
        return (r.get('result') or {}).get('value')

    def click(self, x, y):
        """真实鼠标事件：命中测试由浏览器做，和用户点一模一样。"""
        base = {'x': x, 'y': y, 'button': 'left', 'clickCount': 1,
                'buttons': 1, 'pointerType': 'mouse'}
        self.call('Input.dispatchMouseEvent', type='mouseMoved', **dict(base, buttons=0))
        self.call('Input.dispatchMouseEvent', type='mousePressed', **base)
        self.call('Input.dispatchMouseEvent', type='mouseReleased', **dict(base, buttons=0))
        time.sleep(0.25)

    def centre(self, sel):
        return json.loads(self.js(EXPR_CENTRE % json.dumps(sel)) or 'null')

    def panel(self, name):
        return json.loads(self.js(EXPR_PANEL % json.dumps(PANELS[name][:3])) or '{}')

    def click_at(self, name):
        """量一次、点一次。坐标不跨调用缓存，见 EXPR_CENTRE 上的注释。"""
        c = self.centre(PANELS[name][0])
        self.click(c['x'], c['y'])

    def shot(self, name):
        s = self.call('Page.captureScreenshot', format='png')
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), name), 'wb') as fh:
            fh.write(base64.b64decode(s['data']))

    def close(self):
        try:
            self.ws.close()
        except Exception:
            pass
        self.proc.terminate()
        try:
            self.proc.wait(timeout=5)
        except Exception:
            self.proc.kill()
        time.sleep(0.5)


def main():
    pg = Page('')
    checks = []
    try:
        st = json.loads(pg.js(EXPR_SETUP) or '{}')
        if 'err' in st:
            print('setup failed:', st)
            return 1
        print('地形记录 %s 行，计数徽标 %s' % (st['lines'], st['count']))

        # ---- 默认状态
        print('\n--- 默认状态 ---')
        for name in ('hud', 'terrainLog', 'escLog'):
            s = st[name]
            print('  %-11s display=%-5s cls=%-4r body=%s h=%.1f' %
                  (name, s['disp'], s['cls'], s['body'], s['h']))
        checks.append(('地图信息默认收起', 'col' in st['hud']['cls'].split()))
        checks.append(('地图信息正文默认不可见', st['hud']['body'] == 'none'))
        checks.append(('地形提示默认收起', 'min' in st['terrainLog']['cls'].split()))
        checks.append(('地形提示正文默认不可见', st['terrainLog']['body'] == 'none'))
        checks.append(('地形提示仍占一条标题栏', st['terrainLog']['h'] >= 30))
        checks.append(('脱困日志默认展开可用', st['escLog']['disp'] == 'flex' and
                       'min' not in st['escLog']['cls'].split()))

        # ---- 顺序：脱困日志在地形提示下面
        print('\n--- 排列顺序 ---')
        print('  %s' % ' -> '.join(st['order']))
        print('  地形提示 %.1f..%.1f | 脱困日志 %.1f..%.1f' %
              (st['terrain']['top'], st['terrain']['bottom'],
               st['esc']['top'], st['esc']['bottom']))
        want = ['hud', 'terrainLog', 'escLog', 'senseHud']
        checks.append(('左栏顺序 地图信息→地形提示→脱困日志', st['order'] == want))
        checks.append(('脱困日志在地形提示正下方',
                       st['esc']['top'] >= st['terrain']['bottom'] - 0.5))

        # ---- 命中测试
        print('\n--- 命中测试：控件中心 elementFromPoint ---')
        for k in ('hudToggle', 'terrainLogMin', 'terrainLogClear', 'escLogMin', 'escLogClear'):
            h = st['hit'].get(k) or {}
            print('  %-16s at (%s, %s) -> %s' % (k, h.get('x'), h.get('y'), h.get('hit')))
            checks.append(('%s 中心命中自己' % k, bool(h.get('self'))))
        print('  面板缝隙 -> %s' % st['gap'])
        checks.append(('面板缝隙仍穿到画布', st['gap'] == 'CANVAS'))

        # ---- 三个折叠按钮：各点两次，翻过去再翻回来
        print('\n--- 真实点击折叠按钮（两次，翻过去再翻回来）---')
        for name in ('hudToggle', 'terrainLogMin', 'escLogMin'):
            flag = PANELS[name][3]
            was = pg.panel(name)
            pg.click_at(name)
            now = pg.panel(name)
            pg.click_at(name)
            back = pg.panel(name)
            print('  %-14s %r/%-4s/%.1f  ->  %r/%-5s/%.1f  ->  %r/%-4s/%.1f' %
                  (name, was['cls'], was['body'], was['h'],
                   now['cls'], now['body'], now['h'],
                   back['cls'], back['body'], back['h']))
            checks.append(('%s 类名翻转' % name,
                           (flag in now['clsSet']) != (flag in was['clsSet'])))
            checks.append(('%s 正文可见性翻转' % name,
                           (now['body'] == 'none') != (was['body'] == 'none')))
            checks.append(('%s 箭头翻转' % name, now['icon'] != was['icon']))
            checks.append(('%s 收起后只剩标题栏' % name,
                           now['h'] <= now['headH'] + 3 if flag in now['clsSet']
                           else back['h'] <= back['headH'] + 3))
            checks.append(('%s 第二次点回到原样' % name,
                           back['clsSet'] == was['clsSet'] and back['body'] == was['body']
                           and back['icon'] == was['icon']))

        # ---- 清空
        print('\n--- 真实点击 地形提示 清空 ---')
        pg.click_at('terrainLogClear')
        s = pg.panel('terrainLogMin')
        print('  count=%s first=%r' % (s['count'], s['first']))
        checks.append(('计数归零', s['count'] == '0'))
        checks.append(('回到空态提示', s['first'] == 'ee'))
    finally:
        pg.close()

    print('\n--- 判定 ---')
    bad = 0
    for name, ok in checks:
        print('  [%s] %s' % ('ok' if ok else 'XX', name))
        bad += 0 if ok else 1
    print('\n%d/%d checks passed' % (len(checks) - bad, len(checks)))
    print('page exceptions: %s' % (pg.exc[:3] if pg.exc else 'none'))
    return 0 if bad == 0 and not pg.exc else 1


if __name__ == '__main__':
    raise SystemExit(main())
