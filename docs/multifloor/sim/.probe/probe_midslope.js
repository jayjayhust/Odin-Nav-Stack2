const fs = require('fs');
const base = '/home/jay/workspace/robotics/odin1/Odin-Nav-Stack2/docs/multifloor/sim/';
const tpl = fs.readFileSync(base + 'mesh_nav.template.html', 'utf8');
const between = (a, b) => { const i = tpl.indexOf(a), j = tpl.indexOf(b, i); if (i < 0 || j < 0) throw new Error('slice ' + a); return tpl.slice(i, j); };
global.window = global;
global.location = {hash: ''};
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
const fl = m.flights[2];
const x = fl.cx, y = fl.cy, z = (fl.zmin + fl.zmax)/2;
console.log('mid-slope world', x.toFixed(2), y.toFixed(2), z.toFixed(2), 'flight len', fl.len, 'zmin', fl.zmin, 'zmax', fl.zmax);
const i = Math.floor((x-m.lo[0])/m.res), j = Math.floor((y-m.lo[1])/m.res);
console.log('col', i, j, 'stairCol', m.stairCol[i+j*m.nx], 'rampK', m.stairRamp[i+j*m.nx]);
const kc = Math.floor((z-m.lo[2])/m.res);
for (let k = kc-3; k <= kc+8; ++k){
  const f = (k*m.ny+j)*m.nx+i;
  console.log('k='+k, 'z='+(m.lo[2]+(k+0.5)*m.res).toFixed(2),
    ['UNK','FREE','OCC','INFL'][m.data[f]],
    'top='+(m.topFace?m.topFace[f]:'-'), 'steep='+(m.steep?m.steep[f]:'-'),
    'surf='+m.surfaceCell(i,j,k), 'walk='+m.walkable(i,j,k));
}
const sn = snap(m, x, y, z+0.05);
console.log('snap:', sn ? [sn.i, sn.j, sn.k, (m.lo[2]+(sn.k+0.5)*m.res).toFixed(2)] : 'FAIL');
