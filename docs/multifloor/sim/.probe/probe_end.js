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
const chains = [];
const origLog = console.log;
console.log = (...a) => { if (typeof a[0] === 'string' && a[0].startsWith('CHAIN ')) chains.push(JSON.parse(a[0].slice(6))); else origLog(...a); };
const r = planPath(m, -10.1, 7.0, 4.15+0.05, -8.1, 7.6, 8.7+0.05, m.stairCorr);
console.log = origLog;
origLog('task2 last 4 pts:', JSON.stringify(r.pts.slice(-4)));
origLog('task2 raw last 4:', JSON.stringify(chains[0].slice(-4)));
const r2 = planPath(m, -10.1, 7.0, 4.15+0.05, -10.1, 10.9, 11.2+0.05, m.stairCorr);
origLog('land11 last 4 pts:', JSON.stringify(r2.pts.slice(-4)));
origLog('land11 raw last 4:', JSON.stringify(chains[1].slice(-4)));
origLog('first 4 pts:', JSON.stringify(r2.pts.slice(0,4)));
origLog('first 4 raw:', JSON.stringify(chains[1].slice(0,4)));
