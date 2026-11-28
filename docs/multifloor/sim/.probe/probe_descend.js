const fs = require('fs');
const base = '/home/jay/workspace/robotics/odin1/Odin-Nav-Stack2/docs/multifloor/sim/';
const tpl = fs.readFileSync(base + 'mesh_nav.template.html', 'utf8');
const between = (a, b) => { const i = tpl.indexOf(a), j = tpl.indexOf(b, i); if (i < 0 || j < 0) throw new Error('slice ' + a); return tpl.slice(i, j); };
global.window = global;
global.location = {hash: '#chain'};
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
const m = buildMap(pos, raw.idx, 0.15, 0.45);
// capture CHAIN
const chains = [];
const origLog = console.log;
console.log = (...a) => { if (typeof a[0] === 'string' && a[0].startsWith('CHAIN ')) chains.push(JSON.parse(a[0].slice(6))); else origLog(...a); };
const r = planPath(m, -8.1, 10.9, 6.45+0.05, -4.99, -3.63, 1.48+0.05, m.stairCorr);
console.log = origLog;
console.log('pts=' + r.pts.length);
const ch = chains[0];
// audit raw chain for non-monotone z and big jumps
let back = 0;
for (let t = 1; t < ch.length; ++t){
  const dz = ch[t][2] - ch[t-1][2];
  if (dz < 0){ ++back; if (back <= 10) origLog('RAW back dk=' + dz, 'at', ch[t-1].slice(0,3).join(','), '->', ch[t].slice(0,3).join(',')); }
}
origLog('raw chain backs=' + back + ' of ' + ch.length);
// drawn pts backs
for (let t = 1; t < r.pts.length; ++t){
  const a = r.pts[t-1], b = r.pts[t];
  const dz = b[2]-a[2];
  if (dz < -0.3) origLog('DRAW back dz=' + dz.toFixed(2), a.map(v=>v.toFixed(2)).join(','), '->', b.map(v=>v.toFixed(2)).join(','));
}
// align: print raw k and drawn z side by side around first drawn back
// side-by-side raw vs drawn (both start->goal order)
origLog('--- aligned raw/drawn (first 60)');
for (let t = 0; t < Math.min(60, ch.length); ++t){
  const c = ch[t], p = r.pts[t];
  const f = (c[2]*m.ny+c[1])*m.nx+c[0];
  origLog(t, 'raw', c.slice(0,3).join(','), 'v='+c[3],
    'top='+(m.topFace[f]||0), 'steep='+(m.steep[f]||0), 'surf='+m.surfaceCell(c[0],c[1],c[2]),
    '| drawn', p.map(v=>v.toFixed(2)).join(','));
}
// column attributes along the descent
origLog('--- columns');
const seenCol = new Set();
for (const c of ch){
  const col = c[0] + c[1]*m.nx;
  if (seenCol.has(col)) continue; seenCol.add(col);
  const tr = m.treadRaw[col] || [];
  origLog('col', c[0], c[1], 'stairCol=' + (m.stairCol[col]||0), 'rampK=' + (m.stairRamp[col]!==undefined?m.stairRamp[col].toFixed(1):'-'), 'treads=[' + tr.join(',') + ']');
}
// plunge edge anatomy
for (const [a, b] of [[[67,168,52],[66,168,49]], [[71,155,36],[72,155,33]]]){
  const [i,j,k] = b, col = i+j*m.nx, f = (k*m.ny+j)*m.nx+i;
  const fb = ((k-1)*m.ny+j)*m.nx+i;
  origLog('edge ->', b.join(','), 'stairCol=' + (m.stairCol[col]||0),
    'v=' + m.data[f], 'surf=' + m.surfaceCell(i,j,k),
    'below: v=' + m.data[fb], 'top=' + (m.topFace[fb]||0), 'steep=' + (m.steep[fb]||0),
    'rampK=' + (m.stairRamp[col]!==undefined?m.stairRamp[col].toFixed(1):'-'),
    'srcStairCol=' + (m.stairCol[a[0]+a[1]*m.nx]||0));
}
// projection candidates for the cliff cell
{
  const [i,j,k] = [71,155,36];
  const wx = m.lo[0]+(i+0.5)*m.res, wy = m.lo[1]+(j+0.5)*m.res, wz = m.lo[2]+(k+0.5)*m.res;
  origLog('cell world', wx.toFixed(2), wy.toFixed(2), wz.toFixed(2));
  for (const fl of m.flights){
    if (wz < fl.zmin-0.35 || wz > fl.zmax+0.35) { origLog('fl', fl.cx.toFixed(1), fl.cy.toFixed(1), 'z-window out', fl.zmin.toFixed(2), fl.zmax.toFixed(2)); continue; }
    const ox = wx-fl.cx, oy = wy-fl.cy;
    const along = ox*fl.ux+oy*fl.uy, perp = -ox*fl.uy+oy*fl.ux;
    const at0 = k-fl.kLo;
    origLog('fl', fl.cx.toFixed(2), fl.cy.toFixed(2), 'z', fl.zmin.toFixed(2), '..', fl.zmax.toFixed(2),
      'along=' + along.toFixed(2), 'len/2=' + (fl.len/2).toFixed(2), 'perp=' + perp.toFixed(2),
      'at0=' + at0, 'spineLen=' + fl.spine.length, 'kLo=' + fl.kLo);
    if (Math.abs(along) <= fl.len/2+0.35 && at0 >= -3 && at0 < fl.spine.length+6){
      // invert
      const ts = [], vs = [];
      for (let t = 0; t < fl.alongAt.length; ++t)
        if (fl.alongAt[t] === fl.alongAt[t] && fl.halfK[t] === fl.halfK[t]){ ts.push(t); vs.push(fl.alongAt[t]); }
      let h = -1;
      for (let t = 0; t < vs.length; ++t){
        if (Math.abs(vs[t]-along) <= m.res*1.2){ h = ts[t]; break; }
        if (t+1 < vs.length && (vs[t]-along)*(vs[t+1]-along) <= 0){
          const f2 = (along-vs[t])/((vs[t+1]-vs[t])||1);
          h = ts[t]+Math.max(0,Math.min(1,f2))*(ts[t+1]-ts[t]); break;
        }
      }
      origLog('   invert h=' + (h>=0?h.toFixed(2):'none'), 'ks=' + (h>=0?(fl.kLo+h).toFixed(2):'-'), 'd=' + (h>=0?Math.abs(fl.kLo+h-k).toFixed(2):'-'));
    }
  }
}
