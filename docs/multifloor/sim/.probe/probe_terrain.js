// Regression for the multi-terrain sample: the two terrain lanes must be
// plannable end to end, and the staircase in lane B must still be a flight.
//   node .probe/probe_terrain.js
// Prints, per lane, the plan and its z profile - a ramp is a set of gentle
// risers spread over 3.00 m, a flight is four 0.15 m steps, and the goal z
// proves which one the plan actually used.
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
global.state = {stairOpen: 1, climbMid: 0.35, pipe: 0.60, bandSoft: 0, foot: +(process.env.FOOT || 0.18),
                stairDecouple: 1, erased: new Set(), added: new Set()};
(0, eval)(between('const UNKNOWN = 0', '// ---------- 3D A*') + '\n' +
          between('function edgeRoom(m, id){', '// Why a cell is or is not usable right now') + '\n' +
          between('// ---------- 3D A*', '// A* walks the lattice') + '\n' +
          between('function flood(m, x, y, z){', '// ---------- obstacles'));
(0, eval)(fs.readFileSync(base + 'recognizers/terrain.js', 'utf8'));
(0, eval)(fs.readFileSync(base + 'recognizers/stairs.js', 'utf8'));

function parseObj(file){
  const text = fs.readFileSync(file, 'utf8');
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

const raw = parseObj(base + 'example_multi_terrain.obj');
const m = buildMap(raw.pos, raw.idx, 0.15, 0.30);
console.log('map ' + m.nx + 'x' + m.ny + 'x' + m.nz + '  res=' + m.res +
            '  stairCol=' + m.nStair + '  flights=' + m.flights.length);
for (const f of m.flights)
  console.log('  flight cx=' + f.cx.toFixed(2) + ' cy=' + f.cy.toFixed(2) +
              ' len=' + f.len.toFixed(2) + ' rise=' + f.rise.toFixed(2) +
              ' cells=' + f.n + ' z=' + f.zmin.toFixed(2) + '..' + f.zmax.toFixed(2));
// The macro edge is the only way across a flight core, so a flight whose
// portals did not snap is a flight the route cannot use at all.
const cellOf = id => id < 0 ? '-' : [
  m.lo[0] + ((id % m.nx) + 0.5) * m.res,
  m.lo[1] + ((((id / m.nx) | 0) % m.ny) + 0.5) * m.res,
  m.lo[2] + (((id / (m.nx * m.ny)) | 0) + 0.5) * m.res].map(v => v.toFixed(2)).join(',');
for (const f of m.flights)
  console.log('  portals ' + cellOf(f.portalFoot) + ' -> ' + cellOf(f.portalHead) +
              '  macroOK=' + f.macroOK + '  rows=' + f.rows.length);

// (start, goal, expected: min z the plan must reach, max |y| drift)
const lanes = [
  ['A apron -> stairs -> plate -> ramp -> dock -> ramp -> raised floor',
    [1.00, 2.20, 0.02], [16.60, 2.20, 0.70], 1.15],
  ['A apron -> dock top only',                 [1.00, 2.20, 0.02], [10.60, 2.20, 1.25], 1.15],
  ['B apron -> stairs -> platform',            [1.00, 7.40, 0.02], [6.60, 7.40, 0.70], 0.55],
  ['B apron -> two flights -> upper floor',    [1.00, 7.40, 0.02], [16.60, 7.40, 1.25], 1.10],
  ['middle aisle, flat, past the columns',     [1.00, 4.90, 0.02], [16.60, 4.90, 0.02], 0.10],
];
let bad = 0;
for (const [name, s, g, wantTop] of lanes){
  const r = planPath(m, s[0], s[1], s[2], g[0], g[1], g[2], null);
  if (r.error){
    console.log('\n=== ' + name + '\n  ERROR ' + r.error);
    ++bad; continue;
  }
  let zhi = -1e9, zlo = 1e9, steep = 0, drift = 0;
  for (let t = 0; t < r.pts.length; ++t){
    const p = r.pts[t];
    zhi = Math.max(zhi, p[2]); zlo = Math.min(zlo, p[2]);
    drift = Math.max(drift, Math.abs(p[1] - s[1]));
    if (!t) continue;
    const a = r.pts[t-1];
    const dh = Math.hypot(p[0]-a[0], p[1]-a[1]), dz = p[2]-a[2];
    if (dz > 0.2 && dh > 1e-6 && dz/dh > 1.5) ++steep;
  }
  const ok = zhi >= wantTop && steep === 0 && drift <= 1.25;
  if (!ok) ++bad;
  console.log('\n=== ' + name + (ok ? '' : '   <-- FAIL'));
  console.log('  pts=' + r.pts.length + ' len=' + r.len.toFixed(2) +
              ' expanded=' + r.expanded + ' z=' + zlo.toFixed(2) + '..' + zhi.toFixed(2) +
              ' (want top >= ' + wantTop + ')');
  console.log('  steep=' + steep + ' yDrift=' + drift.toFixed(2));
  // sample the profile every few points so a wrong shape is visible
  const step = Math.max(1, Math.floor(r.pts.length / 12));
  const prof = [];
  for (let t = 0; t < r.pts.length; t += step)
    prof.push('(' + r.pts[t][0].toFixed(1) + ',' + r.pts[t][2].toFixed(2) + ')');
  console.log('  x/z ' + prof.join(' '));
}
console.log('\n' + (bad ? bad + ' lane(s) FAILED' : 'all lanes OK'));
process.exitCode = bad ? 1 : 0;
