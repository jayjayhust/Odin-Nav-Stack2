// Breadth-first reachability with the planner's own gates (traversable /
// walkable / fits), printing where the frontier stops and why the next cell
// was refused. No A*: just "can the body stand on it and step there".
// fits/traversable are local to planPathCore, so the three tests are mirrored
// here from the page source rather than reached for.
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
const raw = parseObj(base + (process.env.MAP || 'example_multi_terrain.obj'));
const m = buildMap(raw.pos, raw.idx, 0.15, 0.30);
const {res, lo, nx, ny, nz, data, topFace, steep, surfaceCell} = m;
const F = (i, j, k) => (k*ny + j)*nx + i;
const X = i => lo[0] + (i + 0.5)*res, Z = k => lo[2] + (k + 0.5)*res;
const I = x => Math.floor((x - lo[0]) / res), J = y => Math.floor((y - lo[1]) / res);
const MAX_STEP_Z = 3;                       // 0.45 m / 0.15 m, the page default
const bodyWant = Math.ceil((state.foot * 2) / res);
console.log('bodyWant=' + bodyWant + ' cells  (' + (state.foot*2).toFixed(2) + ' m)');

// -- the page's traversable(), minus the keep-out band (nothing is painted)
const traversable = (i, j, k) => {
  if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz) return false;
  const v = data[F(i, j, k)];
  if (v === 1) return true;
  return v === 2 && surfaceCell && surfaceCell(i, j, k);
};
// -- the page's spanRuns() + fits()
const ri = new Int16Array(nx*ny*nz), rj = new Int16Array(nx*ny*nz);
for (let k = 0; k < nz; ++k){
  for (let j = 0; j < ny; ++j){
    let s = -1;
    for (let i = 0; i <= nx; ++i){
      const free = i < nx && data[F(i, j, k)] === 1;
      if (free && s < 0) s = i;
      else if (!free && s >= 0){ for (let t = s; t < i; ++t) ri[F(t, j, k)] = i - s; s = -1; }
    }
  }
  for (let i = 0; i < nx; ++i){
    let s = -1;
    for (let j = 0; j <= ny; ++j){
      const free = j < ny && data[F(i, j, k)] === 1;
      if (free && s < 0) s = j;
      else if (!free && s >= 0){ for (let t = s; t < j; ++t) rj[F(i, t, k)] = j - s; s = -1; }
    }
  }
}
const fits = (i, j, k) => {
  const id = F(i, j, k);
  if (state.erased.has(id)) return true;
  if (surfaceCell && surfaceCell(i, j, k)) return true;
  if (k >= 1){
    const fb = id - nx*ny;
    if (topFace[fb] === 1 && steep[fb]) return true;
  }
  return ri[id] >= bodyWant && rj[id] >= bodyWant;
};

const sx = +process.argv[2] || 7.00, sy = +process.argv[3] || 7.40;
const sz = +process.argv[4] || 0.75, gx = +process.argv[5] || 13.00;
let s = -1, bd = 1e30;
const i0 = I(sx), j0 = J(sy), k0 = Math.floor((sz - lo[2]) / res);
for (let dk = -12; dk <= 12; ++dk)
  for (let dj = -12; dj <= 12; ++dj)
    for (let di = -12; di <= 12; ++di){
      const i = i0+di, j = j0+dj, k = k0+dk;
      if (!m.walkable(i, j, k) || !fits(i, j, k) || !traversable(i, j, k)) continue;
      const d2 = di*di + dj*dj + dk*dk;
      if (d2 < bd){ bd = d2; s = F(i, j, k); }
    }
if (s < 0){ console.log('no walkable start'); process.exit(1); }
const seen = new Uint8Array(nx*ny*nz), q = [s];
seen[s] = 1;
let head = 0, maxI = s % nx;
const fails = {};
const bump = (g, i, k) => {
  const key = g + '  x=' + X(i).toFixed(2) + '  z=' + Z(k).toFixed(2);
  fails[key] = (fails[key] || 0) + 1;
};
while (head < q.length){
  const cur = q[head++];
  const ci = cur % nx, cj = ((cur/nx)|0) % ny, ck = (cur/(nx*ny))|0;
  for (let dk = -MAX_STEP_Z; dk <= MAX_STEP_Z; ++dk){
    const k = ck + dk;
    if (k < 0 || k >= nz) continue;
    for (let dj = -1; dj <= 1; ++dj){
      const j = cj + dj;
      if (j < 0 || j >= ny) continue;
      for (let di = -1; di <= 1; ++di){
        if (di === 0 && dj === 0 && dk === 0) continue;
        if (dk !== 0 && di !== 0 && dj !== 0) continue;   // corner cut ban
        const i = ci + di;
        if (i < 0 || i >= nx) continue;
        const nf = F(i, j, k);
        if (seen[nf]) continue;
        if (!traversable(i, j, k)){ bump('trav', i, k); continue; }
        if (k < 1 || !m.walkable(i, j, k)){ bump('walk', i, k); continue; }
        if (!fits(i, j, k)){ bump('fits', i, k); continue; }
        seen[nf] = 1;
        q.push(nf);
        if (i > maxI) maxI = i;
      }
    }
  }
}
console.log('reachable cells=' + q.length + '  max x=' + X(maxI).toFixed(2) +
            '  (goal x=' + gx.toFixed(2) + ')');
const jr = J(sy);
let line = '';
for (let i = I(sx) - 1; i <= I(gx) + 1; ++i){
  const ks = [];
  for (let k = 1; k < nz; ++k) if (seen[F(i, jr, k)]) ks.push(Z(k).toFixed(2));
  line += '\n  x=' + X(i).toFixed(2) + '  ' + (ks.join(' ') || '-');
}
console.log('reachable along y=' + sy + line);
const top = Object.entries(fails).sort((p, r) => r[1] - p[1]).slice(0, 14);
console.log('\nrefusals (gate, cell, count):');
for (const [k, n] of top) console.log('  ' + n + 'x  ' + k);
