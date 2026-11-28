// Column anatomy mid-flight3: why do honest standable tread cells disappear?
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
const F = (i, j, k) => (k*m.ny + j)*m.nx + i;
const res = 0.15, lo = m.lo;
// a row of columns across flight3 at mid-run y=9.3 (surface z ~ 7.4)
for (const x of [-8.87, -8.7, -8.4, -8.1, -7.8]){
  const i = Math.floor((x-lo[0])/res), j = Math.floor((9.3-lo[1])/res);
  const col = i + j*m.nx;
  console.log('\nx=' + x + ' y=9.3 cell(' + i + ',' + j + ') stairCol=' + m.stairCol[col] +
    ' treadRaw=' + JSON.stringify(m.treadRaw[col]) + ' treadKs=' + JSON.stringify(m.treadKs[col]) +
    ' ramp=' + (m.stairRamp[col] >= 0 ? m.stairRamp[col].toFixed(1) : '-'));
  for (let k = 48; k <= 70; ++k){
    const v = m.data[F(i,j,k)];
    if (v === 0) continue;
    const z = (lo[2] + (k+0.5)*res).toFixed(2);
    console.log('  k' + k + ' z' + z + ' v=' + v + ' top=' + m.topFace[F(i,j,k)] +
      ' steep=' + m.steep[F(i,j,k)] + ' surf=' + m.surfaceCell(i,j,k) + ' w=' + m.walkable(i,j,k));
  }
}