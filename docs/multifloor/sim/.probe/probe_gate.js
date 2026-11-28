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
const F = (i, j, k) => (k*m.ny + j)*m.nx + i;
const P = [-8.1, 10.9, 6.45];
const onT = (i,j,k) => { const col=i+j*m.nx; if(!m.stairCol[col]) return true; if(k<1) return false; const fb=F(i,j,k-1); return m.data[fb]===2 && m.topFace[fb]===1; };
const i0 = Math.floor((P[0]-m.lo[0])/0.15), j0 = Math.floor((P[1]-m.lo[1])/0.15), k0 = Math.floor((P[2]-m.lo[2])/0.15);
console.log('start cell', i0, j0, k0, 'stairCol', m.stairCol[i0+j0*m.nx], 'treadKs', JSON.stringify(m.treadKs[i0+j0*m.nx]), 'ramp', m.stairRamp[i0+j0*m.nx]);
let bi=-1,bj=-1,bk=-1,bd=1e30;
for (let dk=-12; dk<=12; ++dk) for (let dj=-12; dj<=12; ++dj) for (let di=-12; di<=12; ++di){
  const i=i0+di,j=j0+dj,k=k0+dk;
  if (i<0||j<0||k<1||i>=m.nx||j>=m.ny||k>=m.nz) continue;
  if (m.data[F(i,j,k)]!==1) continue;
  if (!m.walkable(i,j,k) || !onT(i,j,k)) continue;
  const d2=di*di+dj*dj+dk*dk; if (d2<bd){bd=d2;bi=i;bj=j;bk=k;}
}
console.log('SNAPPED', bi, bj, bk);
const spans = spanRuns(m); const bodyWant = Math.ceil((state.foot*2)/0.15);
for (let dk = -3; dk <= 3; ++dk) for (let dj = -1; dj <= 1; ++dj) for (let di = -1; di <= 1; ++di){
  const i = bi+di, j = bj+dj, k = bk+dk;
  const v = m.data[F(i,j,k)];
  const w = m.walkable(i,j,k);
  const col = i+j*m.nx, list = m.treadKs[col];
  const on = !m.stairCol[col] || !list || list.indexOf(k) >= 0 ||
    (m.stairRamp[col] >= 0 && Math.abs(k-m.stairRamp[col]) <= 1.6 && list.some(t => Math.abs(t-k) <= 2));
  if (v === 1) console.log('  cand', di, dj, dk, 'k' + k, 'v=' + v, 'w=' + w, 'on=' + on, 'fits=' + (spans.ri[F(i,j,k)] >= bodyWant && spans.rj[F(i,j,k)] >= bodyWant), 'ri=' + spans.ri[F(i,j,k)], 'rj=' + spans.rj[F(i,j,k)], 'belowOCC=' + (m.data[F(i,j,k-1)]===2) + ' belowTop=' + m.topFace[F(i,j,k-1)] + ' raw=' + JSON.stringify(m.treadRaw[col]));
}
