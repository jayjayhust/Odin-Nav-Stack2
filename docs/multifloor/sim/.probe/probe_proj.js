const fs = require('fs');
const base = '/home/jay/workspace/robotics/odin1/Odin-Nav-Stack2/docs/multifloor/sim/';
const tpl = fs.readFileSync(base + 'mesh_nav.template.html', 'utf8');
const between = (a, b) => { const i = tpl.indexOf(a), j = tpl.indexOf(b, i); if (i < 0 || j < 0) throw new Error('slice ' + a); return tpl.slice(i, j); };
global.window = global; global.location = {hash: ''};
global.document = {getElementById: () => ({value: '0.45'})};
global.state = {stairOpen: 1, climbMid: 0.35, pipe: 0.60, bandSoft: 0, foot: 0.18, erased: new Set(), added: new Set()};
(0, eval)(between('const UNKNOWN = 0', '// ---------- 3D A*') + '\n' +
          between('function edgeRoom(m, id){', '// Why a cell is or is not usable right now') + '\n' +
          between('// ---------- 3D A*', '// A* walks the lattice'));
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
const raw = parseObj(fs.readFileSync('/home/jay/workspace/robotics/3d_resources/sim_env/ue-test_1/ThirdPersonExampleMap.obj', 'utf8'));
const pos = raw.pos;
for (let i = 0; i < pos.length; i += 3){ const y = pos[i+1]; pos[i+1] = -pos[i+2]; pos[i+2] = y; }
for (let i = 0; i < pos.length; ++i) pos[i] *= 0.01;
const m = buildMap(pos, raw.idx, 0.15, 0.30);
const res = 0.15, lo = m.lo;
const P = [-8.72, 9.37, 8.23];
const i = Math.floor((P[0]-lo[0])/res), j = Math.floor((P[1]-lo[1])/res), k = Math.floor((P[2]-lo[2])/res);
console.log('cell', i, j, k);
for (const fl of m.flights){
  const wx = lo[0]+(i+0.5)*res, wy = lo[1]+(j+0.5)*res, wz = lo[2]+(k+0.5)*res;
  const zok = wz >= fl.zmin-0.35 && wz <= fl.zmax+0.35;
  const ox = wx-fl.cx, oy = wy-fl.cy;
  const along = ox*fl.ux+oy*fl.uy, perp = -ox*fl.uy+oy*fl.ux;
  const aok = Math.abs(along) <= fl.len/2+0.35;
  const at0 = k-fl.kLo;
  const tok = at0 >= -2 && at0 < fl.spine.length+2;
  const ts = [], vs = [];
  for (let t = 0; t < fl.alongAt.length; ++t)
    if (fl.alongAt[t] === fl.alongAt[t] && fl.halfK[t] === fl.halfK[t]){ ts.push(t); vs.push(fl.alongAt[t]); }
  let row = -1, rbd = 4;
  for (const t of ts){ const d = Math.abs(t-at0); if (d < rbd){ rbd = d; row = t; } }
  const sp = row >= 0 ? fl.spine[row] : NaN, hw = row >= 0 ? fl.halfK[row] : NaN;
  const pok = row >= 0 && Math.abs(perp-sp) <= hw+0.30;
  console.log('flight c(' + fl.cx.toFixed(2) + ',' + fl.cy.toFixed(2) + ') axis', fl.ux, fl.uy,
    'z[' + fl.zmin.toFixed(2) + ',' + fl.zmax.toFixed(2) + ']', 'kLo', fl.kLo, 'spineLen', fl.spine.length,
    '| zok', zok, 'aok', aok, 'along', along.toFixed(2), 'tok', tok, 'at0', at0, 'pok', pok, 'perp', perp.toFixed(2), 'sp', sp, 'hw', hw);
  if (zok && aok && tok && pok){
    console.log('  validRows', JSON.stringify(ts), 'along', JSON.stringify(vs.map(v => +v.toFixed(2))));
    let h = -1;
    for (let t = 0; t < vs.length; ++t){
      if (Math.abs(vs[t]-along) <= res*1.2){ h = ts[t]; break; }
      if (t+1 < vs.length && (vs[t]-along)*(vs[t+1]-along) <= 0){
        const f2 = (along-vs[t])/((vs[t+1]-vs[t])||1);
        h = ts[t] + Math.max(0, Math.min(1, f2))*(ts[t+1]-ts[t]); break;
      }
    }
    console.log('  h=' + h, 'ks=' + (h >= 0 ? fl.kLo+h : -1), 'k=' + k);
  }
}
