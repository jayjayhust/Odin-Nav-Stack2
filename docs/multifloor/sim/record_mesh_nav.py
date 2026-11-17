#!/usr/bin/env python3
"""Frame-grab a mesh_nav.html run into a PNG sequence, via CDP.

The page walks the dog along the planned route by itself, but a screenshot is a
single instant. This drives `window.recProg` from outside, so every frame sits at
an exact point on the route and the sequence can be encoded at any frame rate
without racing the wall clock.

    REPO=$PWD python3 record_mesh_nav.py "single,ridge,close,solid,stairs" out/ 96 6.6
    ffmpeg -framerate 16 -i out/f%04d.png -vf format=yuv420p walk.mp4

Needs chromium on PATH and websocket-client. REPO is the repository root.
"""

import base64, json, os, subprocess, sys, time, urllib.request
import websocket

HASH, OUT, N, LEN = sys.argv[1], sys.argv[2], int(sys.argv[3]), float(sys.argv[4])
os.makedirs(OUT, exist_ok=True)
url = "file://%s/docs/multifloor/sim/mesh_nav.html#%s" % (os.environ["REPO"], HASH)
proc = subprocess.Popen(["chromium", "--headless=new", "--disable-gpu", "--no-sandbox",
    "--use-gl=swiftshader", "--enable-unsafe-swiftshader", "--hide-scrollbars",
    "--remote-debugging-port=9333", "--remote-allow-origins=*", "--window-size=1600,900",
    url],
    stdout=subprocess.DEVNULL, stderr=open("/tmp/rec_%s.log" % OUT.split('/')[-1], "w"))
for _ in range(60):
    try:
        tgt = json.load(urllib.request.urlopen("http://127.0.0.1:9333/json"))
        page = [t for t in tgt if t["type"] == "page"][0]
        break
    except Exception:
        time.sleep(0.5)
ws = websocket.create_connection(page["webSocketDebuggerUrl"], timeout=30)
mid = [0]
def call(method, **params):
    mid[0] += 1
    ws.send(json.dumps({"id": mid[0], "method": method, "params": params}))
    while True:
        m = json.loads(ws.recv())
        if m.get("id") == mid[0]:
            return m.get("result", {})
call("Page.enable"); call("Runtime.enable")
time.sleep(4.0)   # let the sample branch plan first
for i in range(N):
    v = LEN * i / (N - 1.0) if N > 1 else 0
    call("Runtime.evaluate", expression="window.recHold=true;window.recProg=%f" % v, awaitPromise=False)
    call("Runtime.evaluate", expression="new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)))",
         awaitPromise=True)
    shot = call("Page.captureScreenshot", format="png")
    open(os.path.join(OUT, "f%04d.png" % i), "wb").write(base64.b64decode(shot["data"]))
    if i % 20 == 0:
        print("frame", i, v, flush=True)
ws.close(); proc.terminate()
print("done", N)
