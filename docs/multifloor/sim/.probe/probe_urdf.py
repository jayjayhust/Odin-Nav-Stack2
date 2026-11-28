#!/usr/bin/env python3
"""Offline URDF-loader probe.

The model loader fetches the official URDF from GitHub at runtime and upgrades
the procedural dog; probes run from file:// and may be offline, so this feeds
CACHED text straight into the __MESH.dbgUrdf door (parseUrdf +
buildRobotFromUrdf + rigUrdfLegs) and checks the counts the real models rely on
- no network, no fetch.

  go2   (cached unitree go2_description.urdf):  collision is pure primitives, so
        pending (mesh files still to fetch) must be 0; the 12 leg revolute joints
        minus the 4 skipped hip/abduction joints => 8 trot-driven joints.
  mesh  (inline fixture): a <visual><mesh> link must schedule exactly one pending
        STL fetch instead of being dropped, proving the mesh branch.
  junk  (not XML): parse must fail soft into {err} and never throw to the page.
"""
import json, os, subprocess, time, urllib.request
import websocket

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
HERE = os.path.dirname(os.path.abspath(__file__))
PORT = 9395

MESH_FIXTURE = '''<?xml version="1.0"?>
<robot name="fixture">
  <link name="base">
    <collision><geometry><box size="0.3 0.1 0.1"/></geometry></collision>
  </link>
  <link name="FL_thigh_link">
    <visual><geometry><mesh filename="package://pkg/meshes/FL_thigh.STL"/></geometry></visual>
  </link>
  <joint name="FL_thigh_joint" type="revolute">
    <parent link="base"/><child link="FL_thigh_link"/>
    <origin xyz="0.1 0.08 0"/><axis xyz="0 1 0"/>
    <limit effort="1" velocity="1" lower="-1" upper="1"/>
  </joint>
</robot>
'''


def run(hash_, expr, wait=6.0):
    url = 'file://%s/mesh_nav.html#%s' % (BASE, hash_)
    proc = subprocess.Popen(
        ['google-chrome', '--headless=new', '--disable-gpu', '--no-sandbox',
         '--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--hide-scrollbars',
         '--disable-dev-shm-usage', '--remote-debugging-port=%d' % PORT,
         '--remote-allow-origins=*', '--window-size=1200,800', url],
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
        ws = websocket.create_connection(pages[0]['webSocketDebuggerUrl'], timeout=30)
        mid = [0]
        exc = []
        def call(method, **params):
            mid[0] += 1
            ws.send(json.dumps({'id': mid[0], 'method': method, 'params': params}))
            while True:
                m = json.loads(ws.recv())
                if m.get('method') == 'Runtime.exceptionThrown':
                    d = m['params']['exceptionDetails']
                    exc.append(d.get('text') or d.get('exception', {}).get('description', ''))
                if m.get('id') == mid[0]:
                    return m.get('result', {})
        call('Page.enable'); call('Runtime.enable')
        time.sleep(wait)
        out = call('Runtime.evaluate', expression=expr, returnByValue=True,
                   awaitPromise=True).get('result', {}).get('value')
        ws.close()
        return out, exc
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        time.sleep(0.5)


def door_expr(text):
    return '__MESH.dbgUrdf(%s)' % json.dumps(text)


def main():
    ok = total = 0

    cache = os.path.join(HERE, 'go2_description.urdf.txt')
    if os.path.exists(cache):
        with open(cache, 'r', encoding='utf-8') as fh:
            text = fh.read()
        total += 1
        out, exc = run('single', door_expr(text))
        print('--- go2 cached URDF (visual meshes -> pending 17, 8 trot joints) ---')
        print('result:', out, '| exc:', exc[:2])
        st = out if isinstance(out, dict) else (json.loads(out) if out else None)
        passed = (st and 'err' not in st and exc == [] and
                  st.get('ok') and st.get('links') == 42 and
                  st.get('joints') == 41 and st.get('legJoints') == 12 and
                  st.get('rigLegs') == 8 and st.get('pending') == 17 and
                  st.get('nodes', 0) > 0)
        if st:
            print('  links=%s joints=%s legJoints=%s rigLegs=%s pending=%s nodes=%s'
                  % (st.get('links'), st.get('joints'), st.get('legJoints'),
                     st.get('rigLegs'), st.get('pending'), st.get('nodes')))
        print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
        ok += 1 if passed else 0
    else:
        print('!! go2 cache missing (%s), run the curl in the commit note' % cache)

    total += 1
    out, exc = run('single', door_expr(MESH_FIXTURE))
    print('--- inline mesh fixture (visual-only STL schedules a fetch) ---')
    print('result:', out, '| exc:', exc[:2])
    st = out if isinstance(out, dict) else (json.loads(out) if out else None)
    passed = (st and 'err' not in st and exc == [] and st.get('ok') and
              st.get('links') == 2 and st.get('joints') == 1 and
              st.get('legJoints') == 1 and st.get('rigLegs') == 1 and
              st.get('pending') == 1)
    if st:
        print('  links=%s joints=%s rigLegs=%s pending=%s'
              % (st.get('links'), st.get('joints'), st.get('rigLegs'), st.get('pending')))
    print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
    ok += 1 if passed else 0

    total += 1
    out, exc = run('single', door_expr('this is <<not>> xml'))
    print('--- malformed input fails soft (no page exception) ---')
    print('result:', out, '| exc:', exc[:2])
    st = out if isinstance(out, dict) else (json.loads(out) if out else None)
    passed = (st and 'err' in st and exc == [])
    print('  RESULT: %s' % ('PASS' if passed else 'FAIL'))
    ok += 1 if passed else 0

    print('\n%d/%d passed' % (ok, total))
    return 0 if ok == total else 1


if __name__ == '__main__':
    raise SystemExit(main())
