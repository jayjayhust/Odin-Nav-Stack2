// Regression: the rise-into-air gate must not break the discrete-tread demos.
// Pick the lowest and highest standable cells near the flights and plan between.
const fs = require('fs');
const base = '/home/jay/workspace/robotics/odin1/Odin-Nav-Stack2/docs/multifloor/sim/';
const tpl = fs.readFileSync(base + 'mesh_nav.template.html', 'utf8');
const between = (a, b) => { const i = tpl.indexOf(a), j = tpl.indexOf(b, i); if (i < 0 || j < 0) throw new Error('slice ' + a); return tpl.slice(i, j); };
global.window = global;
global.location = {hash: ''};
global.document = {getElementById: () => ({value: '0.45'})};
global.state = {stairOpen: 1, climbMid: 0.35, pipe: 0.60, bandSoft: 0, foot: 0.18, stairDecouple: 1,
                erased: new Set(), added: new Set()};
const gateLine = 'if (dk > 0 && stairCol && stairCol[i + j*nx]){';
const plannerSrc = between('// ---------- 3D A*', '// A* walks the lattice');
(0, eval)(between('const UNKNOWN = 0', '// ---------- 3D A*') + '\n' +
          between('function edgeRoom(m, id){', '// Why a cell is or is not usable right now') + '\n' +
          plannerSrc + '\n' +
          between('function flood(m, x, y, z){', '// ---------- obstacles'));
const planNoGate = (() => { const src = plannerSrc.replace(gateLine, 'if (false){'); if (src === plannerSrc) throw new Error('gate line not found'); (0, eval)('var UNKNOWN = 0, FREE = 1, OCC = 2, INFL = 3, MAX_STEP_Z = 3, SNAP_R = 12;\n' + src.replace('function planPath', 'function planPathNoGate').replace('const fmt3', 'var fmt3')); return global.planPathNoGate; })();
(0, eval)(fs.readFileSync(base + 'recognizers/terrain.js', 'utf8'));
(0, eval)(fs.readFileSync(base + 'recognizers/stairs.js', 'utf8'));
function parseObj(text){
  const pos = [], idx = [];
  for (const ln of text.split('\n')){
    if (ln.startsWith('v ')){ const a = ln.trim().split(/\s+/); pos.push(+a[1], +a[2], +a[3]); }
    else if (ln.startsWith('f ')){
      const a = ln.trim().split(/\s+/), vs = [];
      for (let k = 1; k < a.length; ++k) vs.push(parseInt(a[k].split('/')[0], 10) - 1);
      for (let k = 1; k + 1 < vs.length; ++k) idx.push(vs[0], vs[k], vs[k+1]);
    }
  }
  return {pos: new Float32Array(pos), idx: new Uint32Array(idx)};
}
const tasks = {
  'example_single_floor.obj': [[3.9, 7.88, 0.2], [7.0, 7.88, 1.7]],
  'example_four_floor.obj': [[1.2, 1.9, 0.2], [3.28, 1.9, 9.5]],
};
for (const file of ['example_single_floor.obj', 'example_four_floor.obj', 'test_four_flights.obj']){
  const raw = parseObj(fs.readFileSync(base + file, 'utf8'));
  const m = buildMap(raw.pos, raw.idx, 0.15, 0.30);
  let lo = null, hi = null;
  for (let k = 1; k < m.nz; ++k)
    for (let j = 0; j < m.ny; ++j)
      for (let i = 0; i < m.nx; ++i){
        if (!m.stairCol[i + j*m.nx]) continue;
        if (!m.walkable(i, j, k)) continue;
        // Standable means honest ground, not the hover layer the landed test
        // accepts up to MAX_STEP_Z over a wide face - a goal picked in air
        // forces every planner to end with a hop onto nothing.
        const f0 = (k*m.ny + j)*m.nx + i, pl = m.nx*m.ny;
        const grounded = (m.surfaceCell && m.surfaceCell(i, j, k)) ||
          m.topFace[f0] === 1 ||
          (m.data[f0-pl] === 2 && m.topFace[f0-pl] === 1) ||
          (k >= 2 && m.data[f0-2*pl] === 2 && m.topFace[f0-2*pl] === 1);
        if (!grounded) continue;
        const p = [m.lo[0]+(i+0.5)*m.res, m.lo[1]+(j+0.5)*m.res, m.lo[2]+(k+0.5)*m.res];
        if (!lo || p[2] < lo[2]) lo = p;
        if (!hi || p[2] > hi[2]) hi = p;
      }
  if (tasks[file]){ lo = tasks[file][0]; hi = tasks[file][1]; }
  if (file === 'example_four_floor.obj'){
    // same picks as the page's pickDemo: widest low stratum -> flood -> highest
    // reachable real plate
    const F2 = (i, j, k) => (k*m.ny + j)*m.nx + i;
    const cnt = new Int32Array(m.nz);
    for (let k = 0; k < m.nz; ++k) for (let j = 0; j < m.ny; ++j) for (let i = 0; i < m.nx; ++i){
      const v = m.data[F2(i, j, k)];
      if (v !== 1 && v !== 3) continue;
      if (k < 1 || !m.supportOrUnknown(i, j, k)) continue;
      cnt[k]++;
    }
    let maxC = 0; for (let k = 0; k < m.nz; ++k) if (cnt[k] > maxC) maxC = cnt[k];
    let kLo = -1; for (let k = 0; k < m.nz; ++k) if (cnt[k] >= maxC*0.5){ kLo = k; break; }
    const bestOn = (k, mask, strict) => {
      let bi = -1, bj = -1, bn = -1;
      for (let j = 1; j < m.ny-1; ++j) for (let i = 1; i < m.nx-1; ++i){
        const v = m.data[F2(i, j, k)];
        if (v !== 1 && v !== 3) continue;
        if (strict ? !m.support(i, j, k) : !m.supportOrUnknown(i, j, k)) continue;
        if (mask && !mask[F2(i, j, k)]) continue;
        let n = 0;
        for (let dj = -1; dj <= 1; ++dj) for (let di = -1; di <= 1; ++di){
          if (!di && !dj) continue;
          const v2 = m.data[F2(i+di, j+dj, k)];
          if (v2 !== 1 && v2 !== 3) continue;
          if (m.supportOrUnknown(i+di, j+dj, k)) ++n;
        }
        if (n > bn){ bn = n; bi = i; bj = j; }
      }
      return bn > 0 ? [bi, bj, k] : null;
    };
    const s0 = bestOn(kLo, null);
    const wS = [m.lo[0]+(s0[0]+0.5)*m.res, m.lo[1]+(s0[1]+0.5)*m.res, m.lo[2]+(s0[2]+0.5)*m.res];
    const reach = flood(m, wS[0], wS[1], wS[2]);
    const cntR = new Int32Array(m.nz);
    for (let k = 0; k < m.nz; ++k){ let c = 0; for (let j = 0; j < m.ny; ++j) for (let i = 0; i < m.nx; ++i) if (reach.seen[F2(i, j, k)]) ++c; cntR[k] = c; }
    let topK = -1;
    for (let k = 0; k < m.nz; ++k){
      if (cntR[k] < 150) continue;
      let real = 0;
      for (let j = 0; j < m.ny; ++j) for (let i = 0; i < m.nx; ++i){
        const f = F2(i, j, k);
        if (!reach.seen[f]) continue;
        if (m.data[f] !== 1 && m.data[f] !== 3) continue;
        if (m.support(i, j, k)) ++real;
      }
      if (real >= 400) topK = k;
    }
    const g0 = bestOn(topK, reach.seen, true);
    const toW = v => [m.lo[0]+(v[0]+0.5)*m.res, m.lo[1]+(v[1]+0.5)*m.res, m.lo[2]+(v[2]+0.5)*m.res];
    console.log('  demo picks: reach=' + reach.cells + ' topK=' + topK + ' s0=' + s0 + ' g0=' + g0);
    if (g0){ lo = toW(s0); hi = toW(g0); }
  }
  const r = planPath(m, lo[0], lo[1], lo[2], hi[0], hi[1], hi[2], null);
  const r0 = planNoGate(m, lo[0], lo[1], lo[2], hi[0], hi[1], hi[2], null);
  console.log('=== ' + file + ' flights=' + m.flights.length);
  console.log('  start ' + lo.map(v => v.toFixed(2)) + ' goal ' + hi.map(v => v.toFixed(2)));
  console.log('  no-gate: ' + (r0.error ? 'ERROR' : 'pts=' + r0.pts.length + ' len=' + r0.len.toFixed(2)));
  if (!r0.error){
    let s0 = 0;
    for (let t = 1; t < r0.pts.length; ++t){
      const a = r0.pts[t-1], b = r0.pts[t];
      const dh = Math.hypot(b[0]-a[0], b[1]-a[1]), dz = b[2]-a[2];
      if (dz > 0.2 && dh > 1e-6 && dz/dh > 1.5){ ++s0; console.log('  no-gate STEEP z ' + a[2].toFixed(2) + '->' + b[2].toFixed(2) + ' at (' + b[0].toFixed(2) + ',' + b[1].toFixed(2) + ')'); }
    }
    console.log('  no-gate steep=' + s0);
  }
  if (r.error){ console.log('  ERROR ' + r.error); continue; }
  let steep = 0;
  for (let t = 1; t < r.pts.length; ++t){
    const a = r.pts[t-1], b = r.pts[t];
    const dh = Math.hypot(b[0]-a[0], b[1]-a[1]), dz = b[2]-a[2];
    if (dz > 0.2 && dh > 1e-6 && dz/dh > 1.5){ ++steep; console.log('  STEEP z ' + a[2].toFixed(2) + '->' + b[2].toFixed(2) + ' at (' + b[0].toFixed(2) + ',' + b[1].toFixed(2) + ') dh=' + dh.toFixed(2)); }
  }
  console.log('  pts=' + r.pts.length + ' len=' + r.len.toFixed(2) + ' steep=' + steep +
    ' z ' + Math.min(...r.pts.map(p => p[2])).toFixed(2) + '..' + Math.max(...r.pts.map(p => p[2])).toFixed(2));
}