#!/usr/bin/env python3
# Build a self-contained HTML report from planner_sim JSON output.
#
# The page embeds the data, so it opens offline with no server and no
# external library: two canvases (top view and side view), a scenario switcher
# and a timeline that replays the A* expansion, then the path, then the robot.
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "..", "..", "..", "docs", "multifloor", "sim",
                    "planner_sim_data.json")
OUT = os.path.join(HERE, "..", "..", "..", "..", "docs", "multifloor", "sim",
                   "planner_sim.html")

TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>3D 规划器离线仿真回放</title>
<style>
  :root{
    --bg:#0a1020; --panel:#131c33; --line:#24304d; --ink:#e8eefb;
    --dim:#8fa3c8; --teal:#25d0c0; --amber:#f2b24a; --green:#3ddc84; --red:#ff7a7a;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);
       font-family:'PingFang SC','Microsoft YaHei',Helvetica,Arial,sans-serif;
       line-height:1.55}
  .wrap{max-width:1040px;margin:0 auto;padding:22px 18px 40px}
  h1{font-size:22px;margin:0 0 6px;font-weight:700}
  .sub{font-size:13px;color:var(--dim);margin-bottom:16px}
  .params{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:14px}
  .chip{font-size:12px;color:#a9bcdc;background:#101828;border:1px solid var(--line);
        border-radius:5px;padding:3px 8px}
  .tabs{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:12px}
  .tab{font-size:13px;padding:7px 13px;border-radius:7px;border:1px solid var(--line);
       background:#101828;color:#a9bcdc;cursor:pointer;min-height:38px}
  .tab.on{border-color:var(--teal);color:var(--teal);background:#0e2a2a}
  .desc{font-size:13px;color:var(--dim);margin:0 0 10px;min-height:20px}
  .views{display:flex;flex-wrap:wrap;gap:10px}
  .view{flex:1 1 420px;min-width:300px;border:1px solid var(--line);border-radius:9px;
        background:#0d1526;padding:8px 8px 4px}
  .view.wide{flex:1 1 100%}
  .view.wide canvas{height:540px}
  .view h3{font-size:12px;margin:2px 4px 6px;color:var(--dim);font-weight:600;
           letter-spacing:.06em}
  canvas{display:block;width:100%;height:290px}
  .ctrl{display:flex;flex-wrap:wrap;align-items:center;gap:10px;margin:14px 0 4px}
  button{font-size:13px;padding:8px 15px;border-radius:7px;border:1px solid var(--line);
         background:#101828;color:#cfe0fb;cursor:pointer;min-height:40px}
  button:hover{border-color:#3a4a70}
  #timeline{flex:1 1 200px;accent-color:#25d0c0;min-width:140px}
  .status{font-size:12.5px;color:var(--dim);font-variant-numeric:tabular-nums}
  .legend{display:flex;flex-wrap:wrap;gap:12px;font-size:12px;color:var(--dim);
          margin:8px 0 0}
  .legend span{display:inline-flex;align-items:center;gap:5px}
  .sw{width:10px;height:10px;border-radius:2px;display:inline-block}
  .checks{margin-top:16px}
  .check{display:flex;gap:9px;align-items:flex-start;padding:7px 10px;margin-bottom:6px;
         border:1px solid var(--line);border-radius:7px;background:var(--panel);font-size:13px}
  .badge{font-weight:700;font-size:12px;padding:1px 7px;border-radius:4px;white-space:nowrap}
  .ok{color:#0c2b1c;background:var(--green)}
  .bad{color:#2b0c0c;background:var(--red)}
  .detail{color:var(--dim);font-size:12.5px}
  @media (max-width:640px){canvas{height:220px}.view.wide canvas{height:380px}h1{font-size:19px}}
</style>
</head>
<body>
<div class="wrap">
  <h1>3D 规划器离线仿真回放</h1>
  <div class="sub">直接链接 <code>voxel_grid.cpp</code> / <code>astar3d.cpp</code> 运行，与机器上跑的是同一份代码；场景为合成点云，无 ROS 依赖。<br>点场景标签切换，拖时间轴或按「播放动画」回放 A* 的探索与出图过程。</div>
  <div class="params" id="params"></div>
  <div class="tabs" id="tabs"></div>
  <div class="desc" id="desc"></div>

  <div class="views">
    <div class="view">
      <h3>俯视 X–Y</h3>
      <canvas id="top"></canvas>
    </div>
    <div class="view">
      <h3 id="sideTitle">侧视 X–Z（高度剖面）</h3>
      <canvas id="side"></canvas>
    </div>
    <div class="view wide">
      <h3>3D 点云（左键拖拽旋转 · 滚轮缩放 · 双击复位）</h3>
      <canvas id="iso"></canvas>
    </div>
  </div>

  <div class="ctrl">
    <button id="play">播放动画</button>
    <button id="reset">重置</button>
    <input id="timeline" type="range" min="0" max="1000" value="1000">
    <span class="status" id="status"></span>
  </div>
  <div class="legend">
    <span><i class="sw" style="background:#6b7fa8"></i>占据体素</span>
    <span><i class="sw" style="background:#1e3a5f"></i>A* 已展开</span>
    <span><i class="sw" style="background:linear-gradient(90deg,#25d0c0,#f59a22)"></i>规划路径（按高度着色，侧视淡色段在切片外）</span>
    <span><i class="sw" style="background:#3ddc84"></i>起点</span>
    <span><i class="sw" style="background:#f2b24a"></i>目标</span>
  </div>

  <div class="checks" id="checks"></div>
</div>

<script>
const DATA = __DATA__;
let SC = 0, playing = false, T = 1, lastTs = 0;
(function pickSceneFromHash(){
  const h = decodeURIComponent(location.hash.slice(1));
  const i = DATA.scenarios.findIndex(s=>s.name===h);
  if(i>=0) SC = i;
})();
const LOOP_S = 6.0, DUR_TRACE = 0.55, DUR_PATH = 0.20;
const $ = (id) => document.getElementById(id);

function fmtParams(p){
  return [
    'resolution '+p.resolution+' m',
    'robot_radius '+p.robot_radius+' m',
    'max_iterations '+p.max_iterations,
    'ground_support '+p.require_ground_support,
    'support_r '+p.ground_support_xy_radius_cells+' cell',
    'snap_r '+p.snap_search_radius_cells+' cell',
    'max_step_z '+p.max_step_z_cells+' cell'
  ].map(t=>'<span class="chip">'+t+'</span>').join('');
}

function buildTabs(){
  $('tabs').innerHTML = DATA.scenarios.map((s,i)=>
    '<button class="tab'+(i===SC?' on':'')+'" data-i="'+i+'">'+s.name+'</button>').join('');
  $('tabs').querySelectorAll('.tab').forEach(b=>{
    b.onclick = ()=>{ SC = +b.dataset.i; T = 1; playing=false;
                      $('play').textContent='播放动画'; buildTabs(); syncUI(); draw(); };
  });
}

function syncUI(){
  const s = DATA.scenarios[SC];
  $('desc').textContent = s.desc + ' · ' + s.message + ' · expanded ' + s.expanded +
                          ' · ' + s.elapsed_ms.toFixed(1) + ' ms';
  $('checks').innerHTML = s.checks.map(c=>
    '<div class="check"><span class="badge '+(c.pass?'ok':'bad')+'">'+(c.pass?'通过':'未通过')+
    '</span><div><div>'+c.name+'</div><div class="detail">'+c.detail+'</div></div></div>').join('');
}

function setupCanvas(cv){
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth || 600, h = cv.clientHeight || 280;
  cv.width = Math.round(w*dpr); cv.height = Math.round(h*dpr);
  const ctx = cv.getContext('2d');
  ctx.setTransform(dpr,0,0,dpr,0,0);
  return {ctx, w, h};
}

// flipY=true puts larger world values towards the top of the canvas, which is
// what both the map (y = north) and the elevation (z = up) views want.
function mapper(w, h, worldW, worldH, flipY){
  const sc = Math.min(w/worldW, h/worldH);
  const ox = (w - worldW*sc)/2, oy = (h - worldH*sc)/2;
  return {
    sc, ox, oy,
    x:(wx)=> ox + wx*sc,
    y:(wy)=> flipY ? (oy + (worldH - wy)*sc) : (oy + wy*sc)
  };
}

// Height coding: floor-level voxels recede, high voxels (walls, tabletops,
// upper treads) pop out. Without this the floor dominates the top view and
// reads as a grid instead of a room.
function zColor(iz, lo, hi){
  const t = hi>lo ? (iz-lo)/(hi-lo) : 0;
  const r = Math.round(38 + t*129), g = Math.round(50 + t*128), b = Math.round(75 + t*137);
  return 'rgb('+r+','+g+','+b+')';
}

// The elevation view slices a band of y around the route. Projecting every
// voxel collapses the along-x wall into a full-height curtain and hides the
// tread / tabletop profile that actually matters.
function aggregate(s){
  const cy = s.goal[1];
  const map = new Map();
  for(const k of s.occ){
    if(Math.abs(k[1]-cy) > 3) continue;
    const cur = map.get(k[0]);
    if(cur){ cur.lo = Math.min(cur.lo,k[2]); cur.hi = Math.max(cur.hi,k[2]); }
    else map.set(k[0], {lo:k[2], hi:k[2]});
  }
  return {map, cy};
}

function draw(){
  const s = DATA.scenarios[SC];
  const res = s.resolution, nx = s.grid[0], ny = s.grid[1], nz = s.grid[2];
  const zmin = s.occ.reduce((a,k)=>Math.min(a,k[2]), 1e9);
  const zmax = s.occ.reduce((a,k)=>Math.max(a,k[2]), -1e9);

  const top = setupCanvas($('top'));
  const m = mapper(top.w, top.h, nx*res, ny*res, true);
  const ctx = top.ctx;
  ctx.clearRect(0,0,top.w,top.h);
  const cw = Math.max(2, m.sc*res*0.9);
  for(const k of s.occ){
    ctx.fillStyle = zColor(k[2], zmin, zmax);
    ctx.fillRect(m.x((k[0]+0.5)*res)-cw/2, m.y((k[1]+0.5)*res)-cw/2, cw, cw);
  }

  const traceN = Math.floor(s.trace.length * Math.min(1, T/DUR_TRACE));
  ctx.fillStyle = 'rgba(45,110,180,0.42)';
  for(let i=0;i<traceN;i++){
    const k = s.trace[i];
    ctx.fillRect(m.x((k[0]+0.5)*res)-cw/2, m.y((k[1]+0.5)*res)-cw/2, cw*0.85, cw*0.85);
  }

  const pathN = (T<=DUR_TRACE) ? 0
              : Math.floor(s.path.length * Math.min(1, (T-DUR_TRACE)/DUR_PATH));
  drawPath(ctx, s.path, pathN, (k,p)=>[m.x((k[0]+0.5)*res), m.y((k[1]+0.5)*res)], zmin, zmax);

  dot(ctx, m.x((s.start[0]+0.5)*res), m.y((s.start[1]+0.5)*res), 4.6, '#3ddc84');
  dot(ctx, m.x((s.goal[0]+0.5)*res), m.y((s.goal[1]+0.5)*res), 4.6, '#f2b24a');
  if(pathN>0){
    const rk = s.path[pathN-1];
    dot(ctx, m.x((rk[0]+0.5)*res), m.y((rk[1]+0.5)*res), 3.6, '#ffffff');
  }

  // -------- elevation --------
  const side = setupCanvas($('side'));
  const m2 = mapper(side.w, side.h, nx*res, nz*res, true);
  const c2 = side.ctx;
  c2.clearRect(0,0,side.w,side.h);
  const agg = aggregate(s);
  $('sideTitle').textContent = '侧视 X–Z（y ≈ ' +
      (s.origin[1] + (agg.cy+0.5)*res).toFixed(2) + ' m 切片 · 淡色段在此切片外）';
  const bw = Math.max(1.6, m2.sc*res);
  for(const [ix, col] of agg.map){
    c2.fillStyle = '#3d4c6b';
    const yTop = m2.y((col.hi+1)*res);
    const yBot = m2.y(col.lo*res);
    c2.fillRect(m2.x((ix)*res), yTop, bw, Math.max(1.6, yBot-yTop));
  }
  const traceN2 = Math.floor(s.trace.length * Math.min(1, T/DUR_TRACE));
  c2.fillStyle='rgba(45,110,180,0.45)';
  for(let i=0;i<traceN2;i++){
    const k=s.trace[i];
    c2.fillRect(m2.x(k[0]*res), m2.y((k[2]+1)*res), Math.max(1.6,bw*0.9), Math.max(1.6,m2.sc*res*0.9));
  }
  drawPath(c2, s.path, pathN, (k,p)=>[m2.x((k[0]+0.5)*res), m2.y((k[2]+0.5)*res)], zmin, zmax, agg.cy);
  dot(c2, m2.x((s.start[0]+0.5)*res), m2.y((s.start[2]+0.5)*res), 3.8, '#3ddc84');
  dot(c2, m2.x((s.goal[0]+0.5)*res), m2.y((s.goal[2]+0.5)*res), 3.8, '#f2b24a');

  drawCloud(s, T);

  $('timeline').value = Math.round(T*1000);
  $('status').textContent = '搜索 ' + traceN + '/' + s.trace.length +
                            ' · 路径 ' + pathN + '/' + s.path.length;
}

// Colour the path by height so the top view still shows the climb on a
// multi-floor route where every point shares the same y.
function pathColor(iz, lo, hi){
  const t = hi>lo ? (iz-lo)/(hi-lo) : 0;
  const r = Math.round(37 + t*205), g = Math.round(208 - t*30), b = Math.round(192 - t*158);
  return 'rgb('+r+','+g+','+b+')';
}

// ------------------------------------------------------------ 3D point cloud
// Perspective projection of the voxel centres: no face occlusion, so no
// painter sort is needed - depth only drives point size and fade. Drag to
// orbit, wheel to zoom, double click to reset.
const cloudCache = new Map();
const CAM0 = { yaw: 2.42, pitch: 0.86, zoom: 1.0 };
const cam = Object.assign({}, CAM0);

function buildCloud(s){
  const res = s.resolution;
  let mnx = 1e9, mny = 1e9, mnz = 1e9, mxx = -1e9, mxy = -1e9, mxz = -1e9;
  for (const k of s.occ) {
    const x = k[0]*res, y = k[1]*res, z = k[2]*res;
    if (x < mnx) mnx = x; if (x > mxx) mxx = x;
    if (y < mny) mny = y; if (y > mxy) mxy = y;
    if (z < mnz) mnz = z; if (z > mxz) mxz = z;
  }
  const cx = (mnx+mxx)/2, cy = (mny+mxy)/2, cz = (mnz+mxz)/2;
  const span = Math.max(mxx-mnx, mxy-mny, mxz-mnz, 1);
  const pts = new Float32Array(s.occ.length * 4);
  let i = 0, zlo = 1e9, zhi = -1e9;
  for (const k of s.occ) {
    pts[i++] = k[0]*res - cx;
    pts[i++] = k[1]*res - cy;
    pts[i++] = k[2]*res - cz;
    pts[i++] = k[2];
    if (k[2] < zlo) zlo = k[2];
    if (k[2] > zhi) zhi = k[2];
  }
  return { pts, span, zlo, zhi, c: {x: cx, y: cy, z: cz},
           half: { x: (mxx-mnx)/2, y: (mxy-mny)/2, z: (mxz-mnz)/2 } };
}

// 16 pre-baked shades; building an rgba() string per point per frame is what
// makes naive point clouds crawl.
const CLOUD_SHADES = (function(){
  const cold = [46, 78, 150], mid = [96, 214, 206], warmC = [252, 196, 92];
  const out = [];
  for (let i = 0; i < 16; i++) {
    const t = i/15;
    const a = t < 0.5 ? cold : mid, b = t < 0.5 ? mid : warmC, w = t < 0.5 ? t*2 : (t-0.5)*2;
    out.push('rgb(' + Math.round(a[0]+(b[0]-a[0])*w) + ',' +
                      Math.round(a[1]+(b[1]-a[1])*w) + ',' +
                      Math.round(a[2]+(b[2]-a[2])*w) + ')');
  }
  return out;
})();

function drawCloud(s, T){
  const cv = setupCanvas($('iso'));
  const key = SC;
  let cache = cloudCache.get(key);
  if (!cache) { cache = buildCloud(s); cloudCache.set(key, cache); }

  const ctx = cv.ctx;
  ctx.clearRect(0, 0, cv.w, cv.h);

  // D is the eye-to-centre distance, so the wheel moves the camera instead of
  // scaling the picture. It is floored at 0.62 span so a close zoom cannot push
  // the eye inside the bounding sphere.
  const D0 = cache.span * 3.8;
  const D = Math.max(cache.span * 0.62, D0 / cam.zoom);
  const cyaw = Math.cos(cam.yaw), syaw = Math.sin(cam.yaw);
  const cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);

  // Camera-space coordinates (f = 1), parameterised by eye distance.
  const mkProj = (dist) => (x, y, z) => {
    const X = x*cyaw - y*syaw;
    const Y = x*syaw + y*cyaw;
    const Y2 = Y*cp + z*sp;
    const Z2 = -Y*sp + z*cp;
    const d = dist + Z2;
    if (d < 0.05) return null;
    return { u: X/d, v: -Y2/d, d: d };
  };
  // Fit the frustum once, at the reference distance, then hold that scale. Refit
  // per frame and the new scale cancels the shorter D exactly - the wheel then
  // changes nothing you can see, which is precisely how it felt.
  const projectRef = mkProj(D0);
  const project = mkProj(D);

  const ptsArr = cache.pts, nPts = ptsArr.length;
  let umin = 1e9, umax = -1e9, vmin = 1e9, vmax = -1e9;
  const step = 4 * Math.max(1, Math.floor((nPts/4) / 900));
  for (let i = 0; i < nPts; i += step) {
    const p = projectRef(ptsArr[i], ptsArr[i+1], ptsArr[i+2]);
    if (!p) continue;
    if (p.u < umin) umin = p.u; if (p.u > umax) umax = p.u;
    if (p.v < vmin) vmin = p.v; if (p.v > vmax) vmax = p.v;
  }
  const PAD = 26;
  const fitF = Math.min((cv.w - PAD*2)/Math.max(1e-6, umax-umin),
                        (cv.h - PAD*2)/Math.max(1e-6, vmax-vmin));
  const ox = cv.w/2 - ((umax+umin)/2)*fitF;
  const oy = cv.h/2 - ((vmax+vmin)/2)*fitF;
  const toScreen = (p) => [ox + p.u*fitF, oy + p.v*fitF];

  const pts = cache.pts, n = pts.length;
  const zspan = (cache.zhi - cache.zlo) || 1;
  const uScale = Math.min(cv.w, cv.h) / 300;  // point radius, view-independent
  for (let i = 0; i < n; i += 4) {
    const p = project(pts[i], pts[i+1], pts[i+2]);
    if (!p) continue;
    const t = (pts[i+3] - cache.zlo) / zspan;
    const sc = toScreen(p);
    const r = Math.max(0.7, Math.min(2.2, uScale * D / p.d));
    const a = Math.max(0.42, Math.min(1, 1.20 - (p.d / D) * 0.62));
    ctx.globalAlpha = a;
    ctx.fillStyle = CLOUD_SHADES[Math.min(15, Math.floor(t*16))];
    ctx.fillRect(sc[0] - r, sc[1] - r, r*2, r*2);
  }
  ctx.globalAlpha = 1;

  const traceN = Math.floor(s.trace.length * Math.min(1, T/DUR_TRACE));
  ctx.fillStyle = 'rgba(90,180,255,0.32)';
  for (let i = 0; i < traceN; i++) {
    const k = s.trace[i];
    const p = project(k[0]*s.resolution - cache.c.x,
                      k[1]*s.resolution - cache.c.y,
                      k[2]*s.resolution - cache.c.z);
    if (!p) continue;
    const sc = toScreen(p);
    const r = Math.max(0.7, Math.min(2.2, uScale*0.8 * D / p.d));
    ctx.globalAlpha = 0.42;
    ctx.fillRect(sc[0] - r, sc[1] - r, r*2, r*2);
  }
  ctx.globalAlpha = 1;

  const pathN = (T <= DUR_TRACE) ? 0
              : Math.floor(s.path.length * Math.min(1, (T-DUR_TRACE)/DUR_PATH));
  if (pathN > 1) {
    ctx.lineWidth = 2.6; ctx.lineJoin = 'round';
    let prev = null;
    for (let i = 0; i < pathN; i++) {
      const k = s.path[i];
      const p = project(k[0]*s.resolution - cache.c.x,
                        k[1]*s.resolution - cache.c.y,
                        k[2]*s.resolution - cache.c.z);
      if (p) {
        const sc = toScreen(p);
        if (prev) {
          ctx.strokeStyle = pathColor(k[2], cache.zlo, cache.zhi);
          ctx.beginPath(); ctx.moveTo(prev[0], prev[1]); ctx.lineTo(sc[0], sc[1]); ctx.stroke();
        }
        prev = sc;
      }
    }
    if (prev) dot(ctx, prev[0], prev[1], 3.6, '#ffffff');
  }

  const sp0 = project(s.start[0]*s.resolution - cache.c.x, s.start[1]*s.resolution - cache.c.y,
                      s.start[2]*s.resolution - cache.c.z);
  const gp0 = project(s.goal[0]*s.resolution - cache.c.x, s.goal[1]*s.resolution - cache.c.y,
                      s.goal[2]*s.resolution - cache.c.z);
  if (sp0) { const sc = toScreen(sp0); dot(ctx, sc[0], sc[1], 4.4, '#3ddc84'); }
  if (gp0) { const sc = toScreen(gp0); dot(ctx, sc[0], sc[1], 4.4, '#f2b24a'); }
}

(function bindCloudControls(){
  const cv = $('iso');
  if (!cv) return;
  cv.style.cursor = 'grab';
  let drag = null;
  // Left button orbits, right and middle do nothing. Pitch is deliberately
  // unlimited: a three-storey stair core has to be inspectable from underneath
  // and from above, and clamping the elevation only gets in the way.
  cv.addEventListener('mousedown', (e) => {
    if (e.button !== 0) return;
    drag = { x: e.clientX, y: e.clientY };
    cv.style.cursor = 'grabbing';
    e.preventDefault();
  });
  window.addEventListener('mousemove', (e) => {
    if (!drag) return;
    cam.yaw += (e.clientX - drag.x) * 0.008;
    cam.pitch += (e.clientY - drag.y) * 0.006;
    drag.x = e.clientX; drag.y = e.clientY;
    draw();
  });
  window.addEventListener('mouseup', () => { drag = null; cv.style.cursor = 'grab'; });
  cv.addEventListener('contextmenu', (e) => e.preventDefault());
  // Wheel zooms in and out; exponential so a notch feels the same at any scale.
  cv.addEventListener('wheel', (e) => {
    cam.zoom = Math.max(0.25, Math.min(6, cam.zoom * Math.exp(-e.deltaY * 0.0015)));
    e.preventDefault();
    draw();
  }, { passive: false });
  cv.addEventListener('dblclick', () => {
    cam.yaw = CAM0.yaw; cam.pitch = CAM0.pitch; cam.zoom = CAM0.zoom; draw();
  });
})();

// cy is the slice centre in grid units. In the elevation view the voxels are
// only drawn for a band around it, so any path segment outside that band lands
// on empty canvas and reads as hovering in mid-air. Fade those back instead of
// drawing them at full strength.
function drawPath(ctx, path, n, proj, zLo, zHi, cy){
  if(n<1) return;
  ctx.lineWidth=2.4; ctx.lineJoin='round';
  const sliced = (typeof cy === 'number');
  for(let i=1;i<n;i++){
    ctx.globalAlpha = (!sliced || Math.abs(path[i][1]-cy) <= 3) ? 1.0 : 0.16;
    const a = proj(path[i-1]), b = proj(path[i]);
    ctx.strokeStyle = pathColor(path[i][2], zLo, zHi);
    ctx.beginPath(); ctx.moveTo(a[0],a[1]); ctx.lineTo(b[0],b[1]); ctx.stroke();
  }
  ctx.globalAlpha = 1.0;
}

function dot(ctx, x, y, r, color){
  ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x,y,r,0,6.2832); ctx.fill();
  ctx.strokeStyle = 'rgba(10,16,32,0.85)'; ctx.lineWidth = 1.5; ctx.stroke();
}

function loop(ts){
  if(!lastTs){ lastTs = ts; requestAnimationFrame(loop); return; }
  const dt = Math.min(0.25, (ts-lastTs)/1000);
  lastTs = ts;
  if(playing){
    T += dt/LOOP_S;
    if(T >= 1){ T = 1; playing = false; $('play').textContent = '重播动画'; }
    draw();
  }
  requestAnimationFrame(loop);
}

$('play').onclick = ()=>{
  if(playing){ playing=false; $('play').textContent='继续'; }
  else { if(T>=1) T=0; playing=true; $('play').textContent='暂停'; draw(); }
};
$('reset').onclick = ()=>{ T=1; playing=false; $('play').textContent='播放动画';
                           $('timeline').value=1000; draw(); };
$('timeline').oninput = (e)=>{ playing=false; $('play').textContent='播放动画';
                               T = (+e.target.value)/1000; draw(); };
window.addEventListener('resize', ()=>{ draw(); });

$('params').innerHTML = fmtParams(DATA.params);
buildTabs(); syncUI(); draw(); requestAnimationFrame(loop);
</script>
</body>
</html>
"""


def main():
    with open(DATA, "r", encoding="utf-8") as f:
        data = json.load(f)
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    html = TEMPLATE.replace("__DATA__", payload)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(html)
    size = os.path.getsize(OUT)
    print("written %s (%.1f KB)" % (OUT, size / 1024.0))


if __name__ == "__main__":
    main()
