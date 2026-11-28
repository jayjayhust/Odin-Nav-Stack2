// Sweep plausible user tasks on the UE tower: every landing/floor pair, with
// the SAME call the page makes (z + 0.05, stairCorr). Flag near-vertical rises.
const fs = require('fs');
const base = '/home/jay/workspace/robotics/odin1/Odin-Nav-Stack2/docs/multifloor/sim/';
const tpl = fs.readFileSync(base + 'mesh_nav.template.html', 'utf8');
const between = (a, b) => { const i = tpl.indexOf(a), j = tpl.indexOf(b, i); if (i < 0 || j < 0) throw new Error('slice ' + a); return tpl.slice(i, j); };
global.window = global;
global.location = {hash: ''};
global.document = {getElementById: () => ({value: '0.45'})};
global.state = {stairOpen: 1, climbMid: 0.35, pipe: 0.60, bandSoft: 0, foot: 0.18, stairDecouple: 1,
                erased: new Set(), added: new Set()};
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
console.log('flights=' + m.flights.length);

// spots a user would click: floors, landings, flight feet/heads
const spots = [
  ['floorW',   [-4.99, -3.63, 1.48]],
  ['floorE',   [0.05, 6.0, 1.48]],
  ['plat1',    [0.05, 10.6, 4.35]],
  ['land6.3a', [-8.1, 10.9, 6.45]],
  ['land6.3b', [-9.5, 10.5, 6.45]],
  ['land8.6a', [-8.1, 7.6, 8.7]],
  ['land8.6b', [-9.0, 7.8, 8.7]],
  ['land11.1', [-10.1, 10.9, 11.2]],
  ['top11.1b', [-10.1, 10.6, 11.2]],
  ['f2foot',   [-10.1, 7.0, 4.15]],
  ['f1foot',   [-8.1, 11.2, 1.48]],
];
// mid-slope clicks: users click the visible incline, not just landings.
// The click belongs ON the centre line at that fraction of its arc length -
// the old linear z-over-len guess landed a metre off the surface on fits
// whose padded length outruns the real rows.
for (let fi = 0; fi < m.flights.length; ++fi){
  const fl = m.flights[fi];
  for (const frac of [0.3, 0.6]){
    const target = frac * fl.arc[fl.arc.length-1];
    let q = 0;
    while (q + 1 < fl.arc.length && fl.arc[q+1] < target) ++q;
    const p = fl.rowPts[q];
    spots.push(['f' + fi + '@' + frac, [p[0], p[1], p[2] + 0.1]]);
  }
}
let nBad = 0;
for (const [sn, s] of spots)
  for (const [gn, g] of spots){
    if (sn === gn) continue;
    const r = planPath(m, s[0], s[1], s[2] + 0.05, g[0], g[1], g[2] + 0.05, m.stairCorr);
    if (r.error){ console.log('ERR  ' + sn + ' -> ' + gn + ' : ' + r.error.slice(0, 40)); continue; }
    let vert = 0, steep = 0, back = 0; const bad = [];
    for (let t = 1; t < r.pts.length; ++t){
      const a = r.pts[t-1], b = r.pts[t];
      const dh = Math.hypot(b[0]-a[0], b[1]-a[1]), dz = b[2]-a[2];
      if (dz > 0.25 && dh < 0.35){ ++vert; bad.push('z' + a[2].toFixed(1) + '->' + b[2].toFixed(1) + '@(' + b[0].toFixed(1) + ',' + b[1].toFixed(1) + ')'); }
      if (dz < -0.25 && dh < 0.35){ ++vert; bad.push('DROP z' + a[2].toFixed(1) + '->' + b[2].toFixed(1) + '@(' + b[0].toFixed(1) + ',' + b[1].toFixed(1) + ')'); }
      if (dz > 0.2 && dh > 1e-6 && dz/dh > 1.5) ++steep;
      if (dz < -0.3) ++back;
    }
    const tag = (vert || steep || back) ? 'BAD ' : 'ok  ';
    if (vert || steep || back) ++nBad;
    console.log(tag + sn + ' -> ' + gn + ' pts=' + r.pts.length + ' vert=' + vert + ' steep=' + steep + ' back=' + back +
      (bad.length ? '  ' + bad.slice(0, 4).join(' ') : ''));
  }
console.log('bad tasks: ' + nBad);