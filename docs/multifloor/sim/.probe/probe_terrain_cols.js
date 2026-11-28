// Debug: dump lane A / lane B cell columns to see what the ramp and kerb
// rasterise into.
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
    if (ln.startsWith('v ')){ const a = ln.trim().split(/\s+/); pos.push(+a[1], +a[2], +a[3]); }
    else if (ln.startsWith('f ')){
      const a = ln.trim().split(/\s+/), vs = [];
      for (let k = 1; k < a.length; ++k) vs.push(parseInt(a[k].split('/')[0], 10) - 1);
      for (let k = 1; k + 1 < vs.length; ++k) idx.push(vs[0], vs[k], vs[k+1]);
    }
  }
  return {pos: new Float32Array(pos), idx: new Uint32Array(idx)};
}
const raw = parseObj(base + 'example_multi_terrain.obj');
const m = buildMap(raw.pos, raw.idx, 0.15, 0.30);
const I = x => Math.floor((x - m.lo[0]) / m.res);
const J = y => Math.floor((y - m.lo[1]) / m.res);
const K = k => (m.lo[2] + (k + 0.5) * m.res).toFixed(2);
const CH = ['U', '.', '#', '+'];   // UNKNOWN FREE OCC INFL
function column(i, j){
  let s = '';
  for (let k = m.nz - 1; k >= 0; --k) s += CH[m.data[(k*m.ny + j)*m.nx + i]];
  // highest standable layer
  let top = -1;
  for (let k = m.nz - 1; k >= 1; --k) if (m.walkable(i, j, k)){ top = k; break; }
  return s + ' top=' + (top < 0 ? '-' : K(top));
}
function why(x, y){
  const i = I(x), j = J(y);
  const F = (i2, j2, k) => (k*m.ny + j2)*m.nx + i2;
  for (let k = 1; k < m.nz; ++k){
    const f = F(i, j, k);
    if (m.data[f] !== 1) continue;
    if (m.walkable(i, j, k)) continue;
    const fb = f - m.nx*m.ny;
    console.log('  x=' + x + ' k=' + k + ' z=' + K(k) + ' FREE but not walkable: ' +
      'below=' + CH[m.data[fb]] + ' topFace=' + m.topFace[fb] + ' steep=' + m.steep[fb] +
      ' stairCol=' + m.stairCol[i + j*m.nx] + ' support=' + m.support(i, j, k) +
      ' supU=' + m.supportOrUnknown(i, j, k) + ' surf=' + m.surfaceCell(i, j, k));
  }
}
console.log('-- why (lane A x=4.58 / 5.33)');
why(4.58, 2.20); why(5.33, 2.20); why(4.28, 2.20);
console.log('lo=' + m.lo.map(v => v.toFixed(2)) +
            ' nx=' + m.nx + ' ny=' + m.ny + ' nz=' + m.nz);
for (const [name, y, x0, x1] of [
    ['lane A ramp', 2.20, 2.6, 6.2],
    ['lane A tail', 2.20, 11.9, 14.2],
    ['lane B ramp down', 7.40, 8.4, 12.2],
    ['raised step', 4.90, 7.6, 9.6]]){
  console.log('\n-- ' + name + '  y=' + y + '  j=' + J(y));
  for (let i = I(x0); i <= I(x1); ++i)
    console.log('  x=' + (m.lo[0] + (i + 0.5) * m.res).toFixed(2) + '  ' + column(i, J(y)));
}
