#!/usr/bin/env python3
"""Render docs/multifloor/sim/nav_sim.html from nav_sim_data.json.

Three views per scenario, all driven by one timeline:

  * top     - X-Y world: live cloud, ground-truth obstacles, global route,
              driven trace and the body envelope at the playhead
  * profile - clearance and ground speed against time, with events
  * iso     - rotatable 3D cloud, route and obstacles

Camera rules, matching the stair replay page: left button only orbits and
pitch is unlimited; the wheel moves the eye (D0 = 3.8 span, floor 0.62 span)
instead of scaling the picture.
"""

import json
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]  # test -> octo_planner_ros1 -> src -> ros_ws -> repo
DATA = REPO / "docs/multifloor/sim/nav_sim_data.json"
OUT = REPO / "docs/multifloor/sim/nav_sim.html"

PAGE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Go2 点云实时导航与避障闭环仿真</title>
<style>
  :root{
    --bg:#0a111d; --panel:#101a2a; --line:#1e2d44; --txt:#e6eef8; --dim:#8fa3bd;
    --cyan:#4fd1e3; --green:#3ddc84; --amber:#f2b24a; --red:#e8574a; --violet:#9d8bf0;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--txt);
       font:14px/1.6 system-ui,-apple-system,"Segoe UI",Roboto,"PingFang SC",sans-serif}
  .wrap{max-width:1240px;margin:0 auto;padding:28px 20px 60px}
  h1{font-size:24px;margin:0 0 6px;letter-spacing:.02em}
  .sub{color:var(--dim);margin:0 0 22px}
  .planfilt{display:flex;gap:8px;margin:0 0 16px;flex-wrap:wrap}
  .planfilt button{background:#16202e;border:1px solid #2a3a4f;color:#98a9bd;
    padding:7px 14px;border-radius:8px;font-size:13px;cursor:pointer}
  .planfilt button.on{border-color:#48a7b4;color:#74dbdd;background:#132430}
  .cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;margin-bottom:22px}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
  .card .k{color:var(--dim);font-size:12px}
  .card .v{font-size:19px;font-weight:600;margin-top:2px;font-variant-numeric:tabular-nums}
  .card .n{color:var(--dim);font-size:11px;margin-top:2px}
  .scene{background:var(--panel);border:1px solid var(--line);border-radius:12px;
         padding:18px;margin-bottom:22px}
  .scene h2{margin:0;font-size:17px;display:flex;align-items:center;gap:10px}
  .tag{font-size:11px;padding:2px 8px;border-radius:20px;border:1px solid}
  .tag.ok{color:var(--green);border-color:#2a6b48;background:#0f2a1e}
  .tag.bad{color:var(--red);border-color:#7a3230;background:#2a1414}
  .desc{color:var(--dim);margin:8px 0 14px;font-size:13px}
  .kpis{display:flex;flex-wrap:wrap;gap:18px;margin-bottom:16px;font-size:13px}
  .kpis span{color:var(--dim)}
  .kpis b{color:var(--txt);font-variant-numeric:tabular-nums}
  .views{display:grid;grid-template-columns:1fr 1fr;gap:14px}
  .view{background:#0b1424;border:1px solid var(--line);border-radius:9px;padding:10px}
  .view.wide{grid-column:1 / -1}
  .view h3{margin:0 0 8px;font-size:12px;color:var(--dim);font-weight:500;
           letter-spacing:.04em;text-transform:uppercase}
  canvas{display:block;width:100%;border-radius:7px}
  .ctrl{display:flex;align-items:center;gap:12px;margin-top:14px}
  .ctrl button{background:#16263d;border:1px solid var(--line);color:var(--txt);
               border-radius:8px;padding:8px 16px;font-size:13px;cursor:pointer;min-width:76px}
  .ctrl button:hover{background:#1d3150}
  .ctrl input[type=range]{flex:1;accent-color:var(--cyan)}
  .ctrl .t{color:var(--dim);font-variant-numeric:tabular-nums;min-width:96px;text-align:right}
  .ev{margin-top:10px;color:var(--dim);font-size:12px}
  .ev i{color:var(--amber);font-style:normal}
  .legend{display:flex;flex-wrap:wrap;gap:14px;margin-top:8px;font-size:11px;color:var(--dim)}
  .legend b{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px;
            vertical-align:-1px}
  .row{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:14px}
  .row button{background:#16263d;border:1px solid var(--line);color:var(--txt);
              border-radius:8px;padding:7px 14px;font-size:13px;cursor:pointer}
  .row button:hover{background:#1d3150}
  .note{color:var(--dim);font-size:12px;border-left:2px solid var(--line);
        padding-left:12px;margin:18px 0}
  @media(max-width:820px){.views{grid-template-columns:1fr}}
</style>
</head>
<body>
<div class="wrap">
  <h1>Go2 点云实时导航与避障闭环仿真</h1>
  <p class="sub" id="sub"></p>
  <div class="planfilt" id="planfilt">
    <button data-p="all" class="on">两个规划器</button>
    <button data-p="NeuPAN">仅 NeuPAN</button>
    <button data-p="DWA">仅 DWA</button>
  </div>
  <div class="cards" id="cards"></div>
  <div class="row">
    <button id="playAll">全部播放</button>
    <button id="resetAll">全部复位</button>
  </div>
  <div id="scenes"></div>
  <div class="note" id="note"></div>
</div>
<script id="payload" type="application/json">__DATA__</script>
<script>
(function(){
  "use strict";
  const DATA = JSON.parse(document.getElementById('payload').textContent);
  const ROBOT = DATA.robot, SENSOR = DATA.sensor;
  const CLOUD = ['#1b3a52','#24506b','#2f6d84','#3a8a9b','#48a7b4','#5bc3cc','#74dbdd'];
  const $ = (id, root) => (root || document).querySelector('#' + id);

  function setup(cv, h){
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    const w = Math.max(240, cv.clientWidth || 520);
    cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
    cv.style.height = h + 'px';
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return {cv: cv, ctx: ctx, w: w, h: h};
  }
  const fmt = (v, n) => (v === undefined || v === null || !isFinite(v)) ? '—' : Number(v).toFixed(n === undefined ? 1 : n);

  // --- world -> screen, equal aspect, y up -----------------------------------
  function mapper(w, h, x0, y0, x1, y1, pad){
    pad = pad === undefined ? 30 : pad;
    const k = Math.min((w - pad * 2) / Math.max(1e-6, x1 - x0),
                       (h - pad * 2) / Math.max(1e-6, y1 - y0));
    const ox = w / 2 - ((x0 + x1) / 2) * k;
    const oy = h / 2 + ((y0 + y1) / 2) * k;
    return { k: k, sx: x => ox + x * k, sy: y => oy - y * k };
  }

  const state = {};

  function sceneShell(sc, i, host){
    const sec = document.createElement('div');
    sec.className = 'scene';
    // The name carries the planner as the last "· x" segment.
    const who = /· (NeuPAN|DWA)$/.exec(sc.name);
    sec.dataset.planner = who ? who[1] : 'all';
    const tag = sc.ok ? '<span class="tag ok">通过</span>'
                      : '<span class="tag bad">' + (sc.collided ? '碰撞' : '未到达') + '</span>';
    const m = sc.metrics;
    // A level change, not a tall room: flat scenes also span 2.6 m of z_range.
    const hasZ = Math.abs((sc.goal_z || 0) - (sc.start_z || 0)) > 0.5;
    sec.innerHTML =
      '<h2>' + sc.name + ' ' + tag + '</h2>' +
      '<p class="desc">' + sc.desc + '</p>' +
      '<div class="kpis">' +
        '<span>用时 <b>' + fmt(m.time) + ' s</b></span>' +
        '<span>里程 <b>' + fmt(m.length) + ' m</b></span>' +
        '<span>最小净距 <b>' + fmt(m.min_clearance, 2) + ' m</b></span>' +
        '<span>全局重规划 <b>' + m.replans + ' 次</b></span>' +
        '<span>局部绕行 <b>' + m.dwa_swerves + ' 次</b></span>' +
        '<span>建图刷新 <b>' + m.scans + ' 帧</b></span>' +
      '</div>' +
      '<div class="views">' +
        '<div class="view"><h3>俯视 X–Y</h3><canvas id="top' + i + '"></canvas>' +
          '<div class="legend">' +
            '<span><b style="background:#5bc3cc"></b>实时点云</span>' +
            '<span><b style="background:#f2b24a"></b>全局路径</span>' +
            '<span><b style="background:#3ddc84"></b>实际轨迹</span>' +
            '<span><b style="background:#e8574a"></b>真值障碍</span>' +
            '<span><b style="background:#f2b24a"></b>膨胀禁区 +0.20 m</span>' +
            '<span><b style="background:#5aafff"></b>局部窗口 2 m</span>' +
          '</div></div>' +
        '<div class="view"><h3>时序剖面 · 净距 / 地速</h3><canvas id="prof' + i + '"></canvas>' +
          '<div class="legend">' +
            '<span><b style="background:#4fd1e3"></b>净距（左轴自适应）</span>' +
            '<span><b style="background:#9d8bf0"></b>地速（右轴 0–0.8 m/s）</span>' +
            '<span><b style="background:#f2b24a"></b>事件</span>' +
          '</div></div>' +
        (sc.map_probe ? '<div class="view wide"><h3>栅格 X–Z · 规划器眼里的这段楼梯（y=' +
          Number(sc.map_probe.y).toFixed(2) + ' m，t≈' + Number(sc.map_probe.t).toFixed(1) + ' s）</h3>' +
          '<canvas id="grid' + i + '"></canvas>' +
          '<div class="legend">' +
            '<span><b style="background:#1a2436"></b>unknown</span>' +
            '<span><b style="background:#39506b"></b>free</span>' +
            '<span><b style="background:#e8574a"></b>occupied</span>' +
            '<span><b style="background:#f2b24a"></b>inflated</span>' +
          '</div></div>' : '') +
        (hasZ ? '<div class="view wide"><h3>侧视 X–Z · 高度剖面</h3><canvas id="xz' + i + '"></canvas>' +
          '<div class="legend">' +
            '<span><b style="background:#3ddc84"></b>实际轨迹</span>' +
            '<span><b style="background:#e8574a"></b>真值障碍（沿 y 折叠）</span>' +
            '<span><b style="background:#f2b24a"></b>目标高度</span>' +
          '</div></div>' : '') +
        '<div class="view wide"><h3>3D 点云 · 左键旋转（俯仰不限）/ 滚轮推拉 / 双击复位</h3>' +
          '<canvas id="iso' + i + '"></canvas></div>' +
      '</div>' +
      '<div class="ctrl">' +
        '<button data-play="' + i + '">播放</button>' +
        '<input type="range" id="rng' + i + '" min="0" max="' + ((sc.trace.length - 1) / 10).toFixed(1) +
          '" step="0.1" value="0">' +
        '<span class="t" id="tt' + i + '">0.0 s / ' + fmt(m.time) + ' s</span>' +
      '</div>' +
      '<div class="ev" id="ev' + i + '"></div>';
    host.appendChild(sec);
    return sec;
  }

  function eventsText(sc, t){
    // The data carries events as "6.30 s  text". Treating them as [t, text]
    // pairs produced "<full sentence> undefined" on every card.
    const evs = (sc.events || []).filter(e => parseFloat(e) <= t + 1e-6);
    if (!evs.length) return '<span>暂无事件</span>';
    const last = evs.slice(-4).map(e => {
      const m = /^([0-9.]+) s\s+(.*)$/.exec(e);
      return m ? '<i>' + m[1] + ' s</i> ' + m[2] : String(e);
    });
    return last.join(' · ');
  }

  // --- clearance for a trace point against ground-truth obstacles -----------
  function clearanceAt(sc, x, y){
    let best = 6.0;
    for (const o of sc.obstacles){
      if (o.appear >= 0 && o.appear > 1e9) continue;
      const z0 = o.z[0];
      if (z0 > ROBOT.base_z + 0.06) continue;
      if (o.k === 1){
        best = Math.min(best, Math.hypot(x - o.a[0], y - o.a[1]) - o.a[2]);
      } else {
        const dx = Math.max(Math.max(o.a[0] - x, x - o.a[2]), 0);
        const dy = Math.max(Math.max(o.a[1] - y, y - o.a[3]), 0);
        best = Math.min(best, Math.hypot(dx, dy) - Math.max(ROBOT.half_len, ROBOT.half_wid) * 0.6);
      }
    }
    return best;
  }

  // The window the local planner may use, nearest in time to the playhead.
  function nearestWin(sc, t){
    const lc = sc.local_costmap;
    if (!lc || !lc.frames || !lc.frames.length) return null;
    let k = 0;
    for (let n = 1; n < lc.frames.length; ++n)
      if (Math.abs(lc.frames[n].t - t) < Math.abs(lc.frames[k].t - t)) k = n;
    return lc.frames[k];
  }

  // Only the box travels. Inside it, everything solid - body plus the two cells
  // of inflation - is what the candidate rollouts are tested against; outside
  // it the local planner has no opinion at all.
  function drawLocalWindow(ctx, mp, sc, t){
    const lc = sc.local_costmap;
    const lw = nearestWin(sc, t);
    if (!lc || !lw) return;
    const r = lc.r, res = lc.res;
    const x0 = mp.sx(lw.cx - r), x1 = mp.sx(lw.cx + r);
    const y0 = mp.sy(lw.cy + r), y1 = mp.sy(lw.cy - r);
    ctx.fillStyle = 'rgba(90,175,255,0.05)';
    ctx.fillRect(x0, y0, x1 - x0, y1 - y0);
    ctx.fillStyle = 'rgba(242,178,74,0.40)';
    for (const c of lw.cells){
      const wx = lw.cx - r + c[0] * res, wy = lw.cy - r + c[1] * res;
      ctx.fillRect(mp.sx(wx) - 1.0, mp.sy(wy) - 1.0, 2.0, 2.0);
    }
    ctx.strokeStyle = 'rgba(90,175,255,0.85)';
    ctx.lineWidth = 1.3;
    ctx.setLineDash([5, 4]);
    ctx.beginPath(); ctx.rect(x0, y0, x1 - x0, y1 - y0); ctx.stroke();
    ctx.setLineDash([]);
  }

  function nearestCloud(sc, t){
    let best = sc.clouds[0], bd = 1e9;
    for (const c of sc.clouds){
      const d = Math.abs(c.t - t);
      if (d < bd){ bd = d; best = c; }
    }
    return best;
  }

  function topView(sc, i, t){
    const st = state[i];
    const cv = setup($('top' + i), 320);
    const ctx = cv.ctx;
    const w = sc.world;
    const mp = mapper(cv.w, cv.h, w[0], w[1], w[2], w[3], 26);
    ctx.fillStyle = '#070d17'; ctx.fillRect(0, 0, cv.w, cv.h);

    // ground grid every metre
    ctx.strokeStyle = '#131f33'; ctx.lineWidth = 1;
    for (let x = Math.ceil(w[0]); x <= w[2]; x++){
      ctx.beginPath(); ctx.moveTo(mp.sx(x), mp.sy(w[1])); ctx.lineTo(mp.sx(x), mp.sy(w[3])); ctx.stroke();
    }
    for (let y = Math.ceil(w[1]); y <= w[3]; y++){
      ctx.beginPath(); ctx.moveTo(mp.sx(w[0]), mp.sy(y)); ctx.lineTo(mp.sx(w[2]), mp.sy(y)); ctx.stroke();
    }

    // Everything the robot has mapped so far, faint, plus the current sweep
    // on top. A single frame at the playhead reads as a lone blob and hides
    // the fact that the map is built up over the run.
    ctx.fillStyle = 'rgba(70,150,170,0.30)';
    const swept = sc.clouds.filter(c => c.t <= t + 1e-6);
    const every = Math.max(1, Math.floor(swept.length / 10));
    for (let n = 0; n < swept.length; n += every){
      for (const p of swept[n].p){
        const s = mp.sx(p[0]), sy = mp.sy(p[1]);
        if (s < 0 || s > cv.w || sy < 0 || sy > cv.h) continue;
        ctx.fillRect(s - 0.7, sy - 0.7, 1.4, 1.4);
      }
    }
    const cl = nearestCloud(sc, t);
    ctx.fillStyle = 'rgba(110,225,235,0.75)';
    for (const p of cl.p){
      const s = mp.sx(p[0]), sy = mp.sy(p[1]);
      if (s < 0 || s > cv.w || sy < 0 || sy > cv.h) continue;
      ctx.fillRect(s - 1, sy - 1, 1.9, 1.9);
    }

    // ground-truth obstacles, drawn when their spawn time has passed
    const wallLike = o => o.k === 0 && ((o.a[0] <= w[0] + 1e-6 && o.a[2] >= w[2] - 1e-6) ||
                                          (o.a[1] <= w[1] + 1e-6 && o.a[3] >= w[3] - 1e-6));
    for (const o of sc.obstacles){
      const live = o.appear < 0 || t >= o.appear;
      const wall = wallLike(o);
      ctx.strokeStyle = wall ? 'rgba(120,150,190,0.55)'
                             : (live ? 'rgba(232,87,74,0.9)' : 'rgba(232,87,74,0.16)');
      ctx.fillStyle = wall ? 'rgba(120,150,190,0.05)'
                           : (live ? 'rgba(232,87,74,0.10)' : 'rgba(232,87,74,0.02)');
      ctx.lineWidth = wall ? 1.0 : 1.4;
      // The planning footprint. The grid is inflated by two cells (0.20 m) for
      // the 0.17 m half width and cellBlocked reads kInflated as solid, so this
      // dashed ring - not the red body - is what the local planner steers
      // around. It is also why a 0.70 m doorway has almost nothing usable in it.
      if (live && !wall){
        const inf = 0.20;
        ctx.strokeStyle = 'rgba(242,178,74,0.42)';
        ctx.lineWidth = 1.0;
        ctx.setLineDash([3, 3]);
        if (o.k === 1){
          ctx.beginPath();
          ctx.arc(mp.sx(o.a[0]), mp.sy(o.a[1]), (o.a[2] + inf) * mp.k, 0, Math.PI * 2);
          ctx.stroke();
        } else {
          const ix0 = mp.sx(o.a[0] - inf), ix1 = mp.sx(o.a[2] + inf);
          const iy0 = mp.sy(o.a[3] + inf), iy1 = mp.sy(o.a[1] - inf);
          ctx.beginPath(); ctx.rect(ix0, iy0, ix1 - ix0, iy1 - iy0); ctx.stroke();
        }
        ctx.setLineDash([]);
      }
      if (o.k === 1){
        ctx.beginPath();
        ctx.arc(mp.sx(o.a[0]), mp.sy(o.a[1]), o.a[2] * mp.k, 0, Math.PI * 2);
        ctx.fill(); ctx.stroke();
        if (live) dotText(ctx, mp.sx(o.a[0]), mp.sy(o.a[1]) - o.a[2] * mp.k - 6,
                          '行人 r=' + o.a[2] + ' m · t=' + o.appear + 's');
      } else {
        const x0 = mp.sx(o.a[0]), x1 = mp.sx(o.a[2]);
        const y0 = mp.sy(o.a[3]), y1 = mp.sy(o.a[1]);
        ctx.beginPath(); ctx.rect(x0, y0, x1 - x0, y1 - y0); ctx.fill(); ctx.stroke();
        if (!wall && o.appear > 0 && live)
          dotText(ctx, (x0 + x1) / 2, y0 - 6, '箱子 · t=' + o.appear + 's');
      }
    }

    drawLocalWindow(ctx, mp, sc, t);

    // global route
    ctx.strokeStyle = 'rgba(242,178,74,0.85)'; ctx.lineWidth = 1.6;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    sc.global_path.forEach((p, n) => {
      const s = mp.sx(p[0]), sy = mp.sy(p[1]);
      if (n) ctx.lineTo(s, sy); else ctx.moveTo(s, sy);
    });
    ctx.stroke(); ctx.setLineDash([]);

    // driven trace up to the playhead
    const last = Math.min(sc.trace.length - 1, Math.round(t * 10));
    ctx.lineWidth = 2.4; ctx.lineJoin = 'round';
    for (let n = 1; n <= last; n++){
      const a = sc.trace[n - 1], b = sc.trace[n];
      const al = 0.28 + 0.72 * (n / Math.max(1, last));
      ctx.strokeStyle = 'rgba(61,220,132,' + al.toFixed(3) + ')';
      ctx.beginPath(); ctx.moveTo(mp.sx(a[0]), mp.sy(a[1]));
      ctx.lineTo(mp.sx(b[0]), mp.sy(b[1])); ctx.stroke();
    }

    // body envelope at the playhead
    if (last >= 0){
      const p = sc.trace[last];
      const yaw = p[3], c = Math.cos(yaw), s = Math.sin(yaw);
      const hl = ROBOT.half_len, hw = ROBOT.half_wid;
      const pts = [[-hl,-hw],[hl,-hw],[hl,hw],[-hl,hw]];
      ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.3;
      ctx.beginPath();
      pts.forEach((q, n) => {
        const wx = p[0] + q[0] * c - q[1] * s;
        const wy = p[1] + q[0] * s + q[1] * c;
        const sx = mp.sx(wx), sy = mp.sy(wy);
        if (n) ctx.lineTo(sx, sy); else ctx.moveTo(sx, sy);
      });
      ctx.closePath(); ctx.stroke();
    }

    // start / goal
    ctx.fillStyle = '#3ddc84';
    ctx.beginPath(); ctx.arc(mp.sx(sc.start[0]), mp.sy(sc.start[1]), 4.5, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#f2b24a';
    ctx.beginPath(); ctx.arc(mp.sx(sc.goal[0]), mp.sy(sc.goal[1]), 4.5, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = '#8fa3bd'; ctx.font = '11px system-ui';
    ctx.fillText('起点', mp.sx(sc.start[0]) + 7, mp.sy(sc.start[1]) + 4);
    const gz = (sc.goal_z === undefined ? 0 : sc.goal_z);
    ctx.fillText('终点 z' + Number(gz).toFixed(2) + ' m',
                 mp.sx(sc.goal[0]) + 7, mp.sy(sc.goal[1]) + 4);
    ctx.fillText((w[2] - w[0]) + ' × ' + (w[3] - w[1]) + ' m', 10, cv.h - 10);
  }

  function dotText(ctx, x, y, txt){
    ctx.save();
    ctx.fillStyle = 'rgba(232,87,74,0.95)';
    ctx.font = '10px system-ui';
    ctx.fillText(txt, Math.max(4, x - ctx.measureText(txt).width / 2), Math.max(11, y));
    ctx.restore();
  }

  function profileView(sc, i, t){
    const cv = setup($('prof' + i), 320);
    const ctx = cv.ctx;
    const W = cv.w, H = cv.h, P = { l: 38, r: 12, t: 14, b: 26 };
    const T = Math.max(1, (sc.trace.length - 1) / 10);
    ctx.fillStyle = '#070d17'; ctx.fillRect(0, 0, W, H);

    const clearVals = sc.trace.map(p => clearanceAt(sc, p[0], p[1]));
    // trace stores x,y,z,yaw; speed is the 0.1 s displacement
    const spdVals = sc.trace.map((p, n) => n === 0 ? 0
      : Math.hypot(p[0] - sc.trace[n-1][0], p[1] - sc.trace[n-1][1]) / 0.1);
    const cmax = Math.max(1.0, ...clearVals);
    const smax = 0.80;  // fixed right axis, comfortably above the 0.55 m/s cap
    const px = s => P.l + (W - P.l - P.r) * (s / T);
    const pyC = v => P.t + (H - P.t - P.b) * (1 - v / cmax);
    const pyS = v => P.t + (H - P.t - P.b) * (1 - v / smax);

    // axes
    ctx.strokeStyle = '#1e2d44'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(P.l, P.t); ctx.lineTo(P.l, H - P.b);
    ctx.lineTo(W - P.r, H - P.b); ctx.stroke();
    ctx.fillStyle = '#8fa3bd'; ctx.font = '10px system-ui';
    for (let s = 0; s <= T; s += Math.max(1, Math.round(T / 6))){
      ctx.fillText(s.toFixed(0) + 's', px(s) - 6, H - P.b + 14);
    }
    ctx.fillText(cmax.toFixed(1), 6, P.t + 10);
    ctx.fillText('0', 22, H - P.b - 2);

    // events
    ctx.strokeStyle = 'rgba(242,178,74,0.55)'; ctx.setLineDash([3, 3]);
    (sc.events || []).forEach(e => {
      const tt = parseFloat(e[0]);
      if (!isFinite(tt)) return;
      ctx.beginPath(); ctx.moveTo(px(tt), P.t); ctx.lineTo(px(tt), H - P.b); ctx.stroke();
    });
    ctx.setLineDash([]);

    // clearance
    ctx.strokeStyle = '#4fd1e3'; ctx.lineWidth = 1.8;
    ctx.beginPath();
    clearVals.forEach((v, n) => {
      const x = px(n / 10), y = pyC(v);
      if (n) ctx.lineTo(x, y); else ctx.moveTo(x, y);
    });
    ctx.stroke();

    // Speed on its own fixed right axis (0..0.6 m/s) so a 0.18 m/s walk
    // does not sit glued to the top of the clearance scale.
    ctx.strokeStyle = 'rgba(157,139,240,0.95)'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    spdVals.forEach((v, n) => {
      const x = px(n / 10), y = pyS(v);
      if (n) ctx.lineTo(x, y); else ctx.moveTo(x, y);
    });
    ctx.stroke();

    // playhead
    ctx.strokeStyle = '#e6eef8'; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.moveTo(px(t), P.t); ctx.lineTo(px(t), H - P.b); ctx.stroke();

    // no in-plot captions: they collided with the traces. The legend under the
    // canvas carries the axis ranges instead.
  }

  // --- 3D view ---------------------------------------------------------------
  function cloudPoints(sc){
    let n = 0;
    const out = [];
    for (const c of sc.clouds){ n += c.p.length; }
    const stride = Math.max(1, Math.floor(n / 6000));
    let k = 0;
    for (const c of sc.clouds){
      for (const p of c.p){
        if (k++ % stride === 0) out.push(p);
      }
    }
    return out;
  }

  // The only panel where a climb cannot be mistaken for a flat walk: X across,
  // Z up. Obstacles are collapsed along y, so the treads read as a stair profile
  // and the trace either follows them or it does not.
  function xzView(sc, i, t){
    const el = $('xz' + i);
    if (!el) return;
    const cv = setup(el, 260);
    const ctx = cv.ctx, w = cv.w, h = cv.h;
    try {
      const zr = sc.z_range || [-0.2, 1], wx = sc.world;
      ctx.fillStyle = '#070d17'; ctx.fillRect(0, 0, w, h);
      const pad = 40;
      const zk = (h - pad * 1.2) / (zr[1] - zr[0]);
      // world is [x0, y0, x1, y1] - x1 is index 2, using index 1 gave a
      // negative span and a negative scale, so everything landed off-canvas.
      const xk = (w - pad * 1.6) / (wx[2] - wx[0]);
      const k = Math.min(zk, xk);
      const sx = x => pad + (x - wx[0]) * k;
      const sy = z => h - pad * 0.6 - (z - zr[0]) * k;

      ctx.font = '10px system-ui';
      for (let z = Math.ceil(zr[0]); z <= zr[1]; z += 1){
        const yy = sy(z);
        ctx.strokeStyle = '#1a2942'; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(pad, yy); ctx.lineTo(w - pad * 0.5, yy); ctx.stroke();
        ctx.fillStyle = '#64798f'; ctx.fillText(z.toFixed(0) + ' m', 8, yy + 3);
      }

      // Treads, if the obstacle data lets us draw them. Anything malformed is
      // skipped rather than allowed to abort the frame.
      for (const o of (sc.obstacles || [])){
        if (!o.a || o.a.length < 4) continue;
        const x0 = sx(o.a[0]), x1 = sx(o.a[2]);
        const t0 = sy(o.z[1]), t1 = sy(o.z[0]);
        if (!isFinite(x0) || !isFinite(x1) || !isFinite(t0) || !isFinite(t1)) continue;
        ctx.fillStyle = 'rgba(232,87,74,0.13)';
        ctx.strokeStyle = 'rgba(232,87,74,0.45)'; ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.rect(Math.min(x0, x1), Math.min(t0, t1), Math.abs(x1 - x0), Math.abs(t1 - t0));
        ctx.fill(); ctx.stroke();
      }

      // heights that matter, as horizontals
      const lvl = (z, label, color) => {
        if (!isFinite(z)) return;
        ctx.strokeStyle = color; ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath(); ctx.moveTo(pad, sy(z)); ctx.lineTo(w - pad * 0.5, sy(z)); ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = color; ctx.fillText(label, w - pad * 0.5 - 74, sy(z) - 4);
      };
      lvl(sc.start_z || 0, '起点层', 'rgba(61,220,132,0.75)');
      lvl(sc.goal_z || 0, '目标层', 'rgba(242,178,74,0.8)');

      const last = Math.min(sc.trace.length - 1, Math.round(t * 10));
      ctx.strokeStyle = 'rgba(61,220,132,0.95)'; ctx.lineWidth = 2.4;
      ctx.beginPath();
      for (let n = 0; n <= last; n++){
        const p = sc.trace[n];
        const X = sx(p[0]), Y = sy(p[2]);
        if (n === 0) ctx.moveTo(X, Y); else ctx.lineTo(X, Y);
      }
      ctx.stroke();

      const dot = (x, z, c) => {
        ctx.fillStyle = c;
        ctx.beginPath(); ctx.arc(sx(x), sy(z), 4.5, 0, Math.PI * 2); ctx.fill();
      };
      dot(sc.start[0], sc.start_z || 0, '#3ddc84');
      dot(sc.goal[0], sc.goal_z || 0, '#f2b24a');
      ctx.fillStyle = '#8fa3bd'; ctx.font = '11px system-ui';
      const szv = Number(sc.start_z || 0), gzv = Number(sc.goal_z || 0);
      ctx.fillText('起点 z' + szv.toFixed(2) + ' m', sx(sc.start[0]) + 8, sy(szv) + 16);
      ctx.fillText('目标 z' + gzv.toFixed(2) + ' m', sx(sc.goal[0]) + 8, sy(gzv) - 10);
      ctx.fillText('X–Z 高度剖面 · 障碍沿 y 折叠', 10, h - 10);
    } catch (err){
      ctx.fillStyle = '#f26b5e'; ctx.font = '12px system-ui';
      ctx.fillText('xzView: ' + (err && err.message ? err.message : err), 12, 24);
    }
  }

  // The voxel map the planner is actually reasoning about, as a slab through
  // the return run. A flight that is walkable in truth but unreachable here is
  // visible as unknown cells where the treads should be.
  function gridView(sc, i){
    const p = sc.map_probe;
    const el = $('grid' + i);
    if (!p || !el) return;
    try {
      const cv = setup(el, 320);
      const ctx = cv.ctx, w = cv.w, h = cv.h;
      ctx.fillStyle = '#070d17'; ctx.fillRect(0, 0, w, h);
      const rows = p.cells.length, cols = p.cells[0].length;
      const padL = 46, padT = 14, padR = 12, padB = 26;
      const cw = (w - padL - padR) / cols, rh = (h - padT - padB) / rows;
      const col = {0: '#1a2436', 1: '#39506b', 2: '#e8574a', 3: '#f2b24a'};
      for (let r = 0; r < rows; ++r){
        for (let c = 0; c < cols; ++c){
          const v = p.cells[r][c];
          if (v < 0) continue;
          ctx.fillStyle = col[v] || '#1a2436';
          ctx.fillRect(padL + c * cw, padT + r * rh, Math.ceil(cw), Math.ceil(rh));
        }
      }
      ctx.font = '10px system-ui';
      ctx.fillStyle = '#64798f';
      for (let r = 0; r < rows; r += 5){
        const z = p.z0 + r * p.z_step;
        ctx.fillText(z.toFixed(1), 6, padT + r * rh + 4);
      }
      for (let c = 0; c < cols; c += 10){
        const x = p.x0 + c * p.x_step;
        ctx.fillText(x.toFixed(0) + 'm', padL + c * cw - 6, h - 8);
      }
      ctx.fillStyle = '#8fa3bd';
      ctx.fillText('x →', w - 42, h - 8);
    } catch (err){
      const c2 = el.getContext('2d');
      if (c2){ c2.fillStyle = '#f26b5e'; c2.fillText('gridView: ' + err, 10, 20); }
    }
  }

  function isoView(sc, i, t){
    const st = state[i];
    if (!st.pts) st.pts = cloudPoints(sc);
    const cv = setup($('iso' + i), 400);
    const ctx = cv.ctx;
    ctx.fillStyle = '#070d17'; ctx.fillRect(0, 0, cv.w, cv.h);

    const w = sc.world;
    const cx = (w[0] + w[2]) / 2, cy = (w[1] + w[3]) / 2;
    const span = Math.max(w[2] - w[0], w[3] - w[1], 4.0);
    const cam = st.cam;
    const D0 = span * 3.8;
    const D = Math.max(span * 0.62, D0 / cam.zoom);
    const cyaw = Math.cos(cam.yaw), syaw = Math.sin(cam.yaw);
    const cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
    const mkProj = dist => (x, y, z) => {
      const X = (x - cx) * cyaw - (y - cy) * syaw;
      const Y = (x - cx) * syaw + (y - cy) * cyaw;
      const Y2 = Y * cp + z * sp;
      const Z2 = -Y * sp + z * cp;
      const d = dist + Z2;
      if (d < 0.05) return null;
      return { u: X / d, v: -Y2 / d, d: d };
    };
    const projectRef = mkProj(D0), project = mkProj(D);

    // fit once at the reference distance so the wheel really moves the eye
    let umin = 1e9, umax = -1e9, vmin = 1e9, vmax = -1e9;
    const pts = st.pts;
    const sample = Math.max(1, Math.floor(pts.length / 900));
    for (let n = 0; n < pts.length; n += sample){
      const q = projectRef(pts[n][0], pts[n][1], pts[n][2]);
      if (!q) continue;
      if (q.u < umin) umin = q.u; if (q.u > umax) umax = q.u;
      if (q.v < vmin) vmin = q.v; if (q.v > vmax) vmax = q.v;
    }
    for (const p of [[w[0],w[1]],[w[2],w[1]],[w[2],w[3]],[w[0],w[3]]]){
      const q = projectRef(p[0], p[1], 0);
      if (!q) continue;
      if (q.u < umin) umin = q.u; if (q.u > umax) umax = q.u;
      if (q.v < vmin) vmin = q.v; if (q.v > vmax) vmax = q.v;
    }
    const PAD = 20;
    const fitF = Math.min((cv.w - PAD * 2) / Math.max(1e-6, umax - umin),
                          (cv.h - PAD * 2) / Math.max(1e-6, vmax - vmin));
    const ox = cv.w / 2 - ((umax + umin) / 2) * fitF;
    const oy = cv.h / 2 - ((vmax + vmin) / 2) * fitF;
    const toS = q => [ox + q.u * fitF, oy + q.v * fitF];

    const uScale = Math.min(cv.w, cv.h) / 300;

    // points, coloured by height
    for (let n = 0; n < pts.length; n++){
      const q = project(pts[n][0], pts[n][1], pts[n][2]);
      if (!q) continue;
      const s = toS(q);
      const hz = Math.max(0, Math.min(1, (pts[n][2] + 0.1) / 1.2));
      const r = Math.max(0.6, Math.min(2.0, uScale * D / q.d));
      ctx.globalAlpha = Math.max(0.35, Math.min(1, 1.2 - (q.d / D) * 0.6));
      ctx.fillStyle = CLOUD[Math.min(CLOUD.length - 1, Math.floor(hz * CLOUD.length))];
      ctx.fillRect(s[0] - r, s[1] - r, r * 2, r * 2);
    }
    ctx.globalAlpha = 1;

    // obstacle wireframes
    for (const o of sc.obstacles){
      const live = o.appear < 0 || t >= o.appear;
      ctx.strokeStyle = live ? 'rgba(232,87,74,0.85)' : 'rgba(232,87,74,0.14)';
      ctx.lineWidth = 1.2;
      if (o.k === 1){
        const seg = 16;
        for (const z of [o.z[0], o.z[1]]){
          let prev = null;
          for (let a = 0; a <= seg; a++){
            const th = a / seg * Math.PI * 2;
            const q = project(o.a[0] + Math.cos(th) * o.a[2], o.a[1] + Math.sin(th) * o.a[2], z);
            if (!q){ prev = null; continue; }
            const s = toS(q);
            if (prev){ ctx.beginPath(); ctx.moveTo(prev[0], prev[1]); ctx.lineTo(s[0], s[1]); ctx.stroke(); }
            prev = s;
          }
        }
      } else {
        const bx = [
          [[o.a[0],o.a[1]], [o.a[2],o.a[1]]], [[o.a[2],o.a[1]], [o.a[2],o.a[3]]],
          [[o.a[2],o.a[3]], [o.a[0],o.a[3]]], [[o.a[0],o.a[3]], [o.a[0],o.a[1]]]
        ];
        for (const z of [o.z[0], o.z[1]]){
          for (const e of bx){
            const A = project(e[0][0], e[0][1], z), B = project(e[1][0], e[1][1], z);
            if (!A || !B) continue;
            const a = toS(A), b = toS(B);
            ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
          }
        }
        for (const p of bx){
          const A = project(p[0][0], p[0][1], o.z[0]), B = project(p[0][0], p[0][1], o.z[1]);
          if (!A || !B) continue;
          const a = toS(A), b = toS(B);
          ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
        }
      }
    }

    // global route (dashed amber) and driven trace (green)
    const poly = (arr, color, width, dash) => {
      ctx.strokeStyle = color; ctx.lineWidth = width;
      if (dash) ctx.setLineDash(dash); else ctx.setLineDash([]);
      let prev = null;
      // Each sample carries its own height; projecting everything at z 0
      // flattened the whole climb onto the ground slab.
      for (const p of arr){
        const q = project(p[0], p[1], p.length > 2 ? p[2] : 0);
        if (!q){ prev = null; continue; }
        const s = toS(q);
        if (prev){ ctx.beginPath(); ctx.moveTo(prev[0], prev[1]); ctx.lineTo(s[0], s[1]); ctx.stroke(); }
        prev = s;
      }
      ctx.setLineDash([]);
    };
    poly(sc.global_path, 'rgba(242,178,74,0.9)', 1.6, [5, 4]);
    const last = Math.min(sc.trace.length - 1, Math.round(t * 10));
    poly(sc.trace.slice(0, last + 1), 'rgba(61,220,132,0.95)', 2.2, null);

    // start / goal / body
    // Pass the real height. Pinning both markers at z 0.02 put a second-floor
    // goal on the ground floor in every multi-storey scene - the map was right
    // and the drawing was wrong.
    const sdot = (x, y, z, c, r) => {
      const q = project(x, y, z);
      if (!q) return;
      const s = toS(q);
      ctx.fillStyle = c;
      ctx.beginPath(); ctx.arc(s[0], s[1], r, 0, Math.PI * 2); ctx.fill();
    };
    sdot(sc.start[0], sc.start[1], sc.start_z || 0, '#3ddc84', 4.2);
    sdot(sc.goal[0], sc.goal[1], sc.goal_z || 0, '#f2b24a', 4.2);
    if (last >= 0){
      const p = sc.trace[last];
      const yaw = p[3], c = Math.cos(yaw), s = Math.sin(yaw);
      const hl = ROBOT.half_len, hw = ROBOT.half_wid;
      const quad = [[-hl,-hw],[hl,-hw],[hl,hw],[-hl,hw]];
      ctx.strokeStyle = '#e6eef8'; ctx.lineWidth = 1.4;
      let prev = null;
      for (let n = 0; n <= 4; n++){
        const q = quad[n % 4];
        const wx = p[0] + q[0] * c - q[1] * s;
        const wy = p[1] + q[0] * s + q[1] * c;
        const pr = project(wx, wy, p[2] + ROBOT.base_z);
        if (!pr){ prev = null; continue; }
        const sp2 = toS(pr);
        if (prev){ ctx.beginPath(); ctx.moveTo(prev[0], prev[1]); ctx.lineTo(sp2[0], sp2[1]); ctx.stroke(); }
        prev = sp2;
      }
    }

    // axis hint
    ctx.fillStyle = '#8fa3bd'; ctx.font = '10px system-ui';
    ctx.fillText('偏航 ' + (cam.yaw * 180 / Math.PI).toFixed(0) + '° · 俯仰 ' +
                 (cam.pitch * 180 / Math.PI).toFixed(0) + '° · 推拉 ' + cam.zoom.toFixed(2) + '×',
                 10, cv.h - 10);
  }

  function render(i, t){
    const sc = DATA.scenarios[i];
    topView(sc, i, t);
    profileView(sc, i, t);
    isoView(sc, i, t);
    if (sc.map_probe) gridView(sc, i);
    if (Math.abs((sc.goal_z || 0) - (sc.start_z || 0)) > 0.5) xzView(sc, i, t);
    $('tt' + i).textContent = t.toFixed(1) + ' s / ' + fmt(sc.metrics.time) + ' s';
    $('ev' + i).innerHTML = eventsText(sc, t);
  }

  function bindCamera(i){
    const st = state[i];
    const cv = $('iso' + i);
    cv.style.cursor = 'grab';
    let drag = null;
    cv.addEventListener('mousedown', e => {
      if (e.button !== 0) return;                       // left button only
      drag = { x: e.clientX, y: e.clientY };
      cv.style.cursor = 'grabbing';
      e.preventDefault();
    });
    window.addEventListener('mousemove', e => {
      if (!drag) return;
      st.cam.yaw += (e.clientX - drag.x) * 0.008;
      st.cam.pitch += (e.clientY - drag.y) * 0.006;      // unlimited: no clamp
      drag.x = e.clientX; drag.y = e.clientY;
      render(i, st.t);
    });
    window.addEventListener('mouseup', () => { drag = null; cv.style.cursor = 'grab'; });
    cv.addEventListener('contextmenu', e => e.preventDefault());
    cv.addEventListener('wheel', e => {
      // the wheel moves the eye, exponential so a notch feels the same anywhere
      st.cam.zoom = Math.max(0.25, Math.min(6, st.cam.zoom * Math.exp(-e.deltaY * 0.0015)));
      e.preventDefault();
      render(i, st.t);
    }, { passive: false });
    cv.addEventListener('dblclick', () => {
      const w = sc.world, ar = Math.max(w[2] - w[0], w[3] - w[1]) / Math.min(w[2] - w[0], w[3] - w[1]);
      st.cam = { yaw: ar > 3 ? 0 : -0.9, pitch: ar > 3 ? 0.62 : 0.42, zoom: 1 };
      render(i, st.t);
    });
  }

  function start(){
    const host = $('scenes');
    $('sub').textContent = DATA.scenarios.length + ' 个场景 × 2 种局部规划器 · 点云建图 → 全局 3D A* → 局部避障 → 真值包络碰撞判定' +
      ' · 感知 ' + SENSOR.max_range + ' m / ' + SENSOR.azimuth + ' 线 × ' + SENSOR.rings + ' 层';
    const cards = $('cards');
    cards.innerHTML =
      '<div class="card"><div class="k">机器人包络</div><div class="v">' +
        (ROBOT.half_len * 2).toFixed(2) + ' × ' + (ROBOT.half_wid * 2).toFixed(2) + ' m</div>' +
        '<div class="n">躯干 collision box，来自 go2_description.urdf</div></div>' +
      '<div class="card"><div class="k">整机质量</div><div class="v">' + ROBOT.mass.toFixed(2) + ' kg</div>' +
        '<div class="n">URDF 各连杆累加</div></div>' +
      '<div class="card"><div class="k">传感器高度</div><div class="v">' + ROBOT.sensor_z.toFixed(2) + ' m</div>' +
        '<div class="n">Odin1 装在背部，base_z ' + ROBOT.base_z.toFixed(2) + ' m</div></div>' +
      '<div class="card"><div class="k">感知</div><div class="v">' + SENSOR.max_range + ' m</div>' +
        '<div class="n">' + SENSOR.azimuth + ' 线 / ' + SENSOR.rings + ' 层 / σ ' + SENSOR.sigma + ' m</div></div>';

    $('planfilt').addEventListener('click', e => {
      const b = e.target.closest('button');
      if (!b) return;
      const want = b.dataset.p;
      [].forEach.call(document.querySelectorAll('.planfilt button'), x => x.classList.toggle('on', x === b));
      [].forEach.call(document.querySelectorAll('.scene'), sec => {
        sec.style.display = (want === 'all' || sec.dataset.planner === want) ? '' : 'none';
      });
    });

    DATA.scenarios.forEach((sc, i) => {
      sceneShell(sc, i, host);
      const w = sc.world, ar = Math.max(w[2] - w[0], w[3] - w[1]) / Math.min(w[2] - w[0], w[3] - w[1]);
      state[i] = { t: (sc.trace.length - 1) / 10, playing: false,
                   cam: { yaw: ar > 3 ? 0 : -0.9, pitch: ar > 3 ? 0.62 : 0.42, zoom: 1 } };
      bindCamera(i);
      $('rng' + i).value = ((sc.trace.length - 1) / 10).toFixed(1);
      $('rng' + i).addEventListener('input', e => {
        state[i].t = parseFloat(e.target.value); render(i, state[i].t);
      });
      document.querySelector('[data-play="' + i + '"]').addEventListener('click', () => toggle(i));
    });

    $('playAll').addEventListener('click', () => {
      const anyStopped = DATA.scenarios.some((_, i) => !state[i].playing);
      DATA.scenarios.forEach((sc, i) => setPlay(i, anyStopped, sc));
    });
    $('resetAll').addEventListener('click', () => {
      DATA.scenarios.forEach((sc, i) => {
        setPlay(i, false, sc);
        state[i].t = 0; $('rng' + i).value = 0; render(i, 0);
      });
    });

    $('note').innerHTML = '机器人几何取自 ' + ROBOT.source +
      '。仿真里没有 Gazebo：本环境没有 /opt/ros、没有 Noetic/catkin，因此这里是一个纯 STL 闭环——' +
      '射线扫描 → 体素占据 → 3D A* 全局路径 → 局部避障（NeuPAN：差速 + 1.0 s MPC + 点集硬约束；DWA：全向采样 + 膨胀栅格）→ 真值 OBB 碰撞。' +
      '地面的稀疏回波做了 3×3 稠密化（地面是连续面，稠密化不会藏住脚边的障碍）；' +
      '远端目标靠先验建图可达，近端每 0.5 s 用实时点云重建，每 1.5 s 或偏离路线 0.45 m 时重规划。';

    // open on the end state so the finished run is readable without playing
    DATA.scenarios.forEach((sc, i) => render(i, state[i].t));
  }

  function setPlay(i, on, sc){
    const st = state[i];
    st.playing = on;
    document.querySelector('[data-play="' + i + '"]').textContent = on ? '暂停' : '播放';
    if (on && st.t >= (sc.trace.length - 1) / 10) {
      st.t = 0; $('rng' + i).value = 0;
    }
  }

  function toggle(i){
    const sc = DATA.scenarios[i];
    setPlay(i, !state[i].playing, sc);
  }

  let last = 0;
  function loop(ts){
    const dt = Math.min(0.25, (ts - last) / 1000 || 0);
    last = ts;
    DATA.scenarios.forEach((sc, i) => {
      const st = state[i];
      if (!st.playing) return;
      const T = (sc.trace.length - 1) / 10;
      st.t = Math.min(T, st.t + dt * 0.6);
      $('rng' + i).value = st.t.toFixed(1);
      render(i, st.t);
      if (st.t >= T) setPlay(i, false, sc);
    });
    requestAnimationFrame(loop);
  }

  window.addEventListener('resize', () => {
    DATA.scenarios.forEach((sc, i) => render(i, state[i].t));
  });

  try { start(); requestAnimationFrame(loop); } catch (err) {
    document.getElementById('scenes').innerHTML =
      '<p style="color:#e8574a">渲染失败：' + (err && err.message) + '</p>';
  }
})();
</script>
</body>
</html>
"""


def main():
    payload = json.loads(DATA.read_text())
    body = PAGE.replace("__DATA__", json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    OUT.write_text(body)
    print(f"wrote {OUT} ({OUT.stat().st_size / 1024:.1f} KB)")


if __name__ == "__main__":
    main()
