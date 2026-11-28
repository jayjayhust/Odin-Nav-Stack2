// Node harness: runs the page's real buildMap + recognizers on synthetic
// meshes, so stair detection can be regression-tested without a browser.
//   node test_recognizers.js            -> four-flight scene + bundled demos
//   node test_recognizers.js my.obj     -> any OBJ on disk
'use strict';
const fs = require('fs');
const path = require('path');

// ---- stubs for the page environment --------------------------------------
global.window = global;
global.location = {hash: ''};
global.document = {getElementById: () => ({value: '0.45'})};
global.state = {stairOpen: 1};
global.__TERRAIN_DBG = true;

const here = __dirname;
const tpl = fs.readFileSync(path.join(here, 'mesh_nav.template.html'), 'utf8');
const a = tpl.indexOf('const UNKNOWN = 0');
const b = tpl.indexOf('// ---------- 3D A*');
if (a < 0 || b < 0) throw new Error('cannot slice buildMap out of the template');
// indirect eval: the slice must land in the global scope, not a strict-eval one
(0, eval)(tpl.slice(a, b));                  // defines buildMap & constants
// the import preprocessing lives further down the page; reuse it verbatim so
// an engine export (cm, Y up) is judged on exactly the grid the page builds
const c = tpl.indexOf('const CELL_CAP');
const d = tpl.indexOf('function loadModel(');
if (c < 0 || d < 0) throw new Error('cannot slice fitScale/fitRes out of the template');
(0, eval)(tpl.slice(c, d));
(0, eval)(fs.readFileSync(path.join(here, 'recognizers/terrain.js'), 'utf8'));
(0, eval)(fs.readFileSync(path.join(here, 'recognizers/stairs.js'), 'utf8'));

// ---- tiny OBJ box builder -------------------------------------------------
function boxesToObj(boxes){
  const pos = [], idx = [];
  const V = (x, y, z) => { pos.push(x, y, z); return pos.length/3 - 1; };
  for (const [x0, y0, z0, x1, y1, z1] of boxes){
    const v = [V(x0,y0,z0), V(x1,y0,z0), V(x1,y1,z0), V(x0,y1,z0),
               V(x0,y0,z1), V(x1,y0,z1), V(x1,y1,z1), V(x0,y1,z1)];
    const q = [[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]];
    for (const f of q){ idx.push(v[f[0]], v[f[1]], v[f[2]], v[f[0]], v[f[2]], v[f[3]]); }
  }
  return {pos: new Float32Array(pos), idx: new Uint32Array(idx)};
}

// A flight of solid steps: rise r, tread t, width w, n steps, from (x,y,z),
// advancing along dir ('x+','x-','y+','y-'). Steps are solid bodies down to z.
function flight(x, y, z, dir, n, r, t, w, base){
  const boxes = [];
  for (let s = 0; s < n; ++s){
    const h = z + (s+1)*r;
    let x0, y0, x1, y1;
    if (dir === 'x+'){ x0 = x + s*t; x1 = x + (s+1)*t; y0 = y; y1 = y + w; }
    if (dir === 'x-'){ x1 = x - s*t; x0 = x - (s+1)*t; y0 = y; y1 = y + w; }
    if (dir === 'y+'){ y0 = y + s*t; y1 = y + (s+1)*t; x0 = x; x1 = x + w; }
    if (dir === 'y-'){ y1 = y - s*t; y0 = y - (s+1)*t; x0 = x; x1 = x + w; }
    boxes.push([x0, y0, base, x1, y1, h]);
  }
  return boxes;
}

function fourFlightScene(){
  const boxes = [];
  // slab 24 x 16, top at z=0, with two openings
  boxes.push([-12, -8, -0.2, 12, 8, 0]);
  const hole = (x0, y0, x1, y1) => { /* slab is one box; openings cut below */ };
  // flight A: on slab, +X, like the highlighted one in the report
  boxes.push(...flight(-2, -1, 0, 'x+', 12, 0.18, 0.30, 1.2, -0.2));
  // flight B: on slab, +Y
  boxes.push(...flight(6, -6, 0, 'y+', 12, 0.18, 0.30, 1.2, -0.2));
  // flight C: on slab, -X
  boxes.push(...flight(-6, 5, 0, 'x-', 12, 0.18, 0.30, 1.2, -0.2));
  // flight D: U-shaped double run (9 + landing + 9 back)
  boxes.push(...flight(2, 3, 0, 'x+', 9, 0.18, 0.30, 1.2, -0.2));
  boxes.push([2 + 9*0.30, 3, -0.2, 2 + 9*0.30 + 1.2, 3 + 2*1.2 + 0.3, 9*0.18]);
  boxes.push(...flight(2 + 9*0.30 + 1.2, 3 + 1.2 + 0.3, 9*0.18, 'x-', 9, 0.18, 0.30, 1.2, -0.2));
  return boxesToObj(boxes);
}

function report(name, mesh, res, inf){
  const t0 = Date.now();
  const m = buildMap(mesh.pos, mesh.idx, res, inf);
  if (!m || m.tooBig){ console.log(name, '-> map refused', m); return; }
  console.log('\n=== ' + name + '  res=' + res + '  (' + (Date.now()-t0) + ' ms)');
  console.log('stairCol=' + m.nStair, 'flights=' + m.flights.length,
              'corr=' + m.stairCorr.length);
  for (const f of m.flights)
    console.log('  flight', JSON.stringify({c: f.cx.toFixed(2), cy: f.cy.toFixed(2),
      axis: [f.ux.toFixed(2), f.uy.toFixed(2)], half: +f.half.toFixed(2),
      len: +f.len.toFixed(2), rise: +f.rise.toFixed(2), cells: f.n,
      z: [f.zmin.toFixed(2), f.zmax.toFixed(2)]}));
  const rej = global.__TERRAIN_REJECTS || [];
  const by = {};
  for (const r of rej) by[r.why] = (by[r.why] || 0) + 1;
  console.log('  rejects:', JSON.stringify(by));
  for (const r of rej.slice(0, 14)) console.log('   ', JSON.stringify(r));
  return m;
}

// ---- run ------------------------------------------------------------------
// Same order as loadModel: scale, turn, then let the bounding box pick the
// finest resolution under the cell cap. UP / UNIT / RES mirror the page selects.
function prep(pos, idx){
  const P = new Float32Array(pos);
  const fx = fitScale(P, process.env.UNIT || 'auto');
  if (fx.scale !== 1) for (let i = 0; i < P.length; ++i) P[i] *= fx.scale;
  const up = process.env.UP || 'z';
  if (up === 'y')
    for (let i = 0; i < P.length; i += 3){ const y = P[i+1], z = P[i+2]; P[i+1] = -z; P[i+2] = y; }
  else if (up === 'x')
    for (let i = 0; i < P.length; i += 3){ const x = P[i], z = P[i+2]; P[i] = -z; P[i+2] = x; }
  const lo = [1e18, 1e18, 1e18], hi = [-1e18, -1e18, -1e18];
  for (let i = 0; i < P.length; i += 3)
    for (let ax = 0; ax < 3; ++ax){
      if (P[i+ax] < lo[ax]) lo[ax] = P[i+ax];
      if (P[i+ax] > hi[ax]) hi[ax] = P[i+ax];
    }
  return {pos: P, idx, res: fitRes(lo, hi, +(process.env.RES || 0.15))};
}

function readObj(file){
  const txt = fs.readFileSync(file, 'utf8');
  const pos = [], idx = [];
  for (const ln of txt.split('\n')){
    if (ln.startsWith('v ')){ const p = ln.trim().split(/\s+/); pos.push(+p[1], +p[2], +p[3]); }
    else if (ln.startsWith('f ')){
      const p = ln.trim().split(/\s+/).slice(1).map(s => parseInt(s.split('/')[0], 10) - 1);
      for (let k = 1; k + 1 < p.length; ++k) idx.push(p[0], p[k], p[k+1]);
    }
  }
  return {pos: new Float32Array(pos), idx: new Uint32Array(idx)};
}

const arg = process.argv[2];
if (arg){
  const o = prep(...Object.values(readObj(arg)));
  report(path.basename(arg), o, o.res, 0.30);
} else {
  const m = report('four-flight synthetic', fourFlightScene(), +(process.env.RES || 0.15), 0.30);
  if (m && m.treadTop){
    console.log('\nprobe flight A (x+ from -2,-1, rise .18 tread .30):');
    const {res, lo, nx} = m;
    for (let s = 0; s < 12; s += 2){
      const wx = -2 + s*0.30 + 0.15, wy = -1 + 0.6;
      const i = Math.floor((wx-lo[0])/res), j = Math.floor((wy-lo[1])/res);
      const f = i + j*nx, tt = m.treadTop[f];
      const zTop = (s+1)*0.18;
      const kExpect = Math.floor((zTop-lo[2])/res) + 1;
      const tops = [];
      for (let k = 0; k < m.nz; ++k){
        const f3 = (k*m.ny + j)*m.nx + i;
        if (m.topFace[f3]) tops.push(k + (m.capWide[f3] >= 7 ? 'w' : 'n'));
      }
      console.log('  s' + s, 'prof=' + m.stairProfile[f], 'tops=[' + tops + ']',
        'treadTop=' + tt, 'expectK=' + kExpect);
    }
  }
  for (const f of ['example_single_floor.obj', 'example_multi_terrain.obj', 'example_four_floor.obj']){
    const o = prep(...Object.values(readObj(path.join(here, f))));
    report(f, o, o.res, 0.30);
  }
}