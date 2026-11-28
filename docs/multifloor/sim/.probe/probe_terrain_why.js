// Why does lane A not plan with a body width? Re-evals the planner source with
// each gate instrumented, and prints the rejection histogram per gate.
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
global.__WHY = {fits:0, walk:0, trav:0, macro:0, push:0, byx:{}, byk:{}};
let src = between('// ---------- 3D A*', '// A* walks the lattice');
const a = 'if (!fits(i, j, k)) continue;';
const b = 'if (k < 1 || !walkable(i, j, k)) continue;';
if (src.indexOf(a) < 0) throw new Error('fits gate not found');
src = src.replace(a, 'if (!fits(i, j, k)){ __WHY.fits++; __WHY.byx[i]=(__WHY.byx[i]||0)+1;' +
                     ' __WHY.byk[k]=(__WHY.byk[k]||0)+1; continue; }');
src = src.replace(b, 'if (k < 1 || !walkable(i, j, k)){ __WHY.walk++; continue; }');
src = src.replace('function planPathCore', 'function planPathWhy').replace('const fmt3', 'var fmt3');
// count macro offers: the whole flight core is closed to voxel steps, so a
// route that needs a flight either takes the macro edge or does not move
const mo = 'const offerMacro = (via, fl, r0, r1, dest) => {';
if (src.indexOf(mo) < 0) throw new Error('offerMacro not found');
src = src.replace(mo, mo + ' __WHY.macro++;' +
  ' __WHY.md = __WHY.md || new Map();' +
  ' __WHY.md.set(dest, (__WHY.md.get(dest)||0)+1);');
// one eval call: let/const in an indirect eval do not survive into the next
(0, eval)(between('const UNKNOWN = 0', '// ---------- 3D A*') + '\n' +
          between('function edgeRoom(m, id){', '// Why a cell is or is not usable right now') + '\n' +
          src + '\n' +
          between('function flood(m, x, y, z){', '// ---------- obstacles'));
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
const num = process.argv.slice(2).map(Number);
const P = num.length >= 6 ? num
  : [7.00, 7.40, 0.75, 13.00, 7.40, 0.02];
const r = planPathWhy(m, P[0], P[1], P[2], P[3], P[4], P[5], null);
console.log('plan: ' + (r.error ? r.error : 'ok pts=' + r.pts.length));
console.log('rejects fits=' + __WHY.fits + ' walk=' + __WHY.walk +
            ' macroOffers=' + __WHY.macro);
const xOf = i => (m.lo[0] + (i + 0.5) * m.res).toFixed(2);
for (const [id, n] of [...(__WHY.md || new Map())].sort((p, q) => q[1] - p[1]).slice(0, 8)){
  const i = id % m.nx, j = ((id/m.nx)|0) % m.ny, k = (id/(m.nx*m.ny))|0;
  console.log('  macro dest ' + n + 'x  ' +
    (m.lo[0]+(i+0.5)*m.res).toFixed(2) + ',' +
    (m.lo[1]+(j+0.5)*m.res).toFixed(2) + ',' +
    (m.lo[2]+(k+0.5)*m.res).toFixed(2));
}
const top = Object.entries(__WHY.byx).sort((p, q) => q[1] - p[1]).slice(0, 12);
for (const [i, n] of top) console.log('  fits rejected ' + n + 'x at x=' + xOf(+i));
const topK = Object.entries(__WHY.byk).sort((p, q) => q[1] - p[1]).slice(0, 8);
for (const [k, n] of topK) console.log('  fits rejected ' + n + 'x at k=' + k +
  ' z=' + (m.lo[2] + (+k + 0.5) * m.res).toFixed(2));
// and where the goal-side floor is: is it walkable and reachable at all?
for (const x of [11.5, 11.8, 12.1, 12.4, 12.7, 13.0]){
  const i = Math.floor((x - m.lo[0]) / m.res), j = Math.floor((7.40 - m.lo[1]) / m.res);
  const hits = [];
  for (let k = 1; k < m.nz; ++k)
    if (m.walkable(i, j, k)) hits.push(k + ':' + m.data[(k*m.ny + j)*m.nx + i]);
  console.log('  x=' + x + ' walkable k=' + (hits.join(',') || '-'));
}
