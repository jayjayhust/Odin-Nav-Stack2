const fs = require('fs');
const base = '/home/jay/workspace/robotics/odin1/Odin-Nav-Stack2/docs/multifloor/sim/';
const tpl = fs.readFileSync(base + 'mesh_nav.template.html', 'utf8');
const between = (a, b) => { const i = tpl.indexOf(a), j = tpl.indexOf(b, i); if (i < 0 || j < 0) throw new Error('slice ' + a); return tpl.slice(i, j); };
global.window = global; global.location = {hash: '#diag'};
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
// landing 8.6 anatomy: world x[-9,-7], y[7,8], z 8.6
for (const [x, y, z] of [[-8.1, 7.6, 8.65], [-8.5, 7.6, 8.65], [-8.1, 10.9, 8.7], [-8.1, 10.9, 6.45]]){
  const i = Math.floor((x-m.lo[0])/0.15), j = Math.floor((y-m.lo[1])/0.15), k0 = Math.floor((z-m.lo[2])/0.15);
  console.log('\npoint', x, y, z, 'cell', i, j, k0);
  for (let k = k0-4; k <= k0+4; ++k){
    if (k < 1 || k >= m.nz) continue;
    const v = m.data[F(i,j,k)];
    console.log('  k' + k + ' z' + (m.lo[2]+(k+0.5)*0.15).toFixed(2) + ' v=' + v + ' top=' + m.topFace[F(i,j,k)] + ' w=' + m.walkable(i,j,k) + ' surf=' + m.surfaceCell(i,j,k));
  }
}
const r = planPath(m, -10.1, 7.0, 4.15, -8.1, 10.9, 8.7, null);
console.log('task2 end', r.pts[r.pts.length-1]);
