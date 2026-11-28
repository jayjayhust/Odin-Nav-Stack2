// Prints the body-width runs the planner sees along a lane, so a descent that
// "should" be legal can be explained instead of guessed.
const fs = require('fs');
const path = require('path');
const base = path.join(__dirname, '..') + '/';
const tpl = fs.readFileSync(base + 'mesh_nav.template.html', 'utf8');
const between = (a, b) => {
  const i = tpl.indexOf(a), j = tpl.indexOf(b, i);
  if (i < 0 || j < 0) throw new Error('slice ' + a);
  return tpl.slice(i, j);
};
global.window = global;
global.location = {hash: ''};
global.document = {getElementById: () => ({value: '0.45'})};
global.state = {stairOpen: 1, climbMid: 0.35, pipe: 0.60, bandSoft: 0, foot: 0.18,
                stairDecouple: 1, erased: new Set(), added: new Set()};
(0, eval)(between('const UNKNOWN = 0', '// ---------- 3D A*') + '\n' +
          between('function edgeRoom(m, id){', '// Why a cell is or is not usable right now'));
// buildMap calls TerrainSemantics.run(), which these two files define
(0, eval)(fs.readFileSync(base + 'recognizers/terrain.js', 'utf8'));
(0, eval)(fs.readFileSync(base + 'recognizers/stairs.js', 'utf8'));
function parseObj(file){
  const text = fs.readFileSync(file, 'utf8');
  const pos = [], idx = [];
  for (const ln of text.split('\n')){
    if (ln.startsWith('v ')){ const q = ln.trim().split(/\s+/); pos.push(+q[1], +q[2], +q[3]); }
    else if (ln.startsWith('f ')){
      const q = ln.trim().split(/\s+/), vs = [];
      for (let k = 1; k < q.length; ++k) vs.push(parseInt(q[k].split('/')[0], 10) - 1);
      for (let k = 1; k + 1 < vs.length; ++k) idx.push(vs[0], vs[k], vs[k+1]);
    }
  }
  return {pos: new Float32Array(pos), idx: new Uint32Array(idx)};
}
const raw = parseObj(base + 'example_multi_terrain.obj');
const m = buildMap(raw.pos, raw.idx, 0.15, 0.30);
const {res, nx, ny, nz, data} = m;
const F = (i, j, k) => (k*ny + j)*nx + i;
const ri = new Int16Array(nx*ny*nz), rj = new Int16Array(nx*ny*nz);
for (let k = 0; k < nz; ++k)
  for (let j = 0; j < ny; ++j){
    let s = -1;
    for (let i = 0; i <= nx; ++i){
      const free = i < nx && data[F(i, j, k)] === 1;
      if (free && s < 0) s = i;
      else if (!free && s >= 0){ for (let t = s; t < i; ++t) ri[F(t, j, k)] = i - s; s = -1; }
    }
  }
for (let k = 0; k < nz; ++k)
  for (let i = 0; i < nx; ++i){
    let s = -1;
    for (let j = 0; j <= ny; ++j){
      const free = j < ny && data[F(i, j, k)] === 1;
      if (free && s < 0) s = j;
      else if (!free && s >= 0){ for (let t = s; t < j; ++t) rj[F(i, t, k)] = j - s; s = -1; }
    }
  }
const I = x => Math.floor((x - m.lo[0]) / res), J = y => Math.floor((y - m.lo[1]) / res);
const K = k => (m.lo[2] + (k + 0.5) * res).toFixed(2);
const CH = ['U', '.', '#', '+'];
const ranges = process.argv.length > 2
  ? [[process.argv[2], +process.argv[3], +process.argv[4], +process.argv[5]]]
  : [['B tail', 7.40, 11.2, 12.6], ['A tail', 2.20, 12.9, 14.1],
     ['step', 4.90, 8.0, 9.3]];
for (const [name, y, x0, x1] of ranges){
  console.log('\n-- ' + name);
  const j = J(y);
  for (let i = I(x0); i <= I(x1); ++i){
    let out = '  x=' + (m.lo[0] + (i + 0.5) * res).toFixed(2);
    for (let k = 3; k <= 12; ++k){
      const v = data[F(i, j, k)];
      if (v !== 1 && !m.walkable(i, j, k)) continue;
      out += '  z' + K(k) + ':' + CH[v] + (m.walkable(i, j, k) ? 'W' : '-') +
             ' ri=' + ri[F(i, j, k)] + ' rj=' + rj[F(i, j, k)];
    }
    console.log(out);
  }
}
