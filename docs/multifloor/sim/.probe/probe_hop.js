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
const fl = m.flights[0];
const along = (0.6-0.5)*fl.len, z = fl.zmin + 0.6*(fl.zmax-fl.zmin);
const s = [fl.cx + fl.ux*along, fl.cy + fl.uy*along, z+0.1];
const chains = [];
const origLog = console.log;
console.log = (...a) => { if (typeof a[0] === 'string' && a[0].startsWith('CHAIN ')) chains.push(JSON.parse(a[0].slice(6))); else origLog(...a); };
const r = planPath(m, s[0], s[1], s[2]+0.05, 0.05, 10.6, 4.35+0.05, m.stairCorr);
console.log = origLog;
const ch = chains[0];
for (let t = 0; t < ch.length; ++t){
  const c = ch[t], p = r.pts[t];
  const dz = t ? p[2]-r.pts[t-1][2] : 0, dh = t ? Math.hypot(p[0]-r.pts[t-1][0], p[1]-r.pts[t-1][1]) : 0;
  if (dz > 0.25 && dh < 0.35){
    for (let u = Math.max(0,t-3); u <= Math.min(ch.length-1, t+2); ++u){
      const cc = ch[u], f = (cc[2]*m.ny+cc[1])*m.nx+cc[0];
      origLog(u, 'raw', cc.slice(0,3).join(','), 'v='+cc[3], 'top='+(m.topFace[f]||0), 'steep='+(m.steep[f]||0),
        'surf='+m.surfaceCell(cc[0],cc[1],cc[2]), '| drawn', r.pts[u].map(v=>v.toFixed(2)).join(','));
    }
    break;
  }
}
