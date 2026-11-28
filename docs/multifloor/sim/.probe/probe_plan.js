// Plan a two-switchback climb on the UE map and audit every rising edge:
// which walkable branch lets the search stand in air?
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
const m = buildMap(pos, raw.idx, 0.15, 0.30);
console.log('flights=' + m.flights.length, 'grid', m.nx, m.ny, m.nz);

const F = (i, j, k) => (k*m.ny + j)*m.nx + i;
const cellOf = (x, y, z) => [Math.floor((x-m.lo[0])/0.15), Math.floor((y-m.lo[1])/0.15), Math.floor((z-m.lo[2])/0.15)];
function branch(i, j, k){
  const out = [];
  if (m.supportOrUnknown(i, j, k)) out.push('support');
  if (m.stairCol[i + j*m.nx]){
    const rk = m.stairRamp[i + j*m.nx];
    if (rk >= 0 && Math.abs(k - rk) <= 1.6 && m.data[F(i,j,k)] === 1) out.push('ramp(rk=' + rk.toFixed(1) + ')');
    for (let d = 1; d <= 3; ++d){
      const kk = k - d; if (kk < 1) break;
      if (m.data[F(i,j,kk)] !== 2) continue;
      let span = 0;
      for (let sy = -2; sy <= 2; ++sy) for (let sx = -2; sx <= 2; ++sx){
        const i2 = i+sx, j2 = j+sy;
        if (i2 < 0 || j2 < 0 || i2 >= m.nx || j2 >= m.ny) continue;
        if (m.data[F(i2,j2,kk)] === 2) ++span;
      }
      if (span >= 8){ out.push('landed(d=' + d + ')'); break; }
    }
  }
  return out.join('+') || 'NONE';
}

const tasks = [
  ['landing6.3 -> landing11.1 (flight3+flight4)', [-8.1, 10.9, 6.45], [-10.1, 10.9, 11.2]],
  ['flight2 foot -> landing8.6 (flight2+flight3)', [-10.1, 7.0, 4.15], [-8.1, 7.6, 8.7]],
  ['floor -> top landing (platform+flight2+3+4)', [-4.99, -3.63, 1.48], [-10.1, 10.6, 11.2]],
];
for (const [name, s, g] of tasks){
  const r = planPath(m, s[0], s[1], s[2], g[0], g[1], g[2], null);
  console.log('\n=== ' + name);
  if (r.error){ console.log('  ERROR ' + r.error); continue; }
  console.log('  pts=' + r.pts.length + ' len=' + r.len.toFixed(2) + ' expanded=' + r.expanded);
  let air = 0, steep = 0, back = 0;
  for (let t = 1; t < r.pts.length; ++t){
    const a = r.pts[t-1], b = r.pts[t];
    const dh = Math.hypot(b[0]-a[0], b[1]-a[1]), dz = b[2]-a[2];
    if (dz < -0.3){ ++back; console.log('  BACK dz=' + dz.toFixed(2) + ' at (' + b[0].toFixed(2) + ',' + b[1].toFixed(2) + ') z ' + a[2].toFixed(2) + '->' + b[2].toFixed(2)); }
    if (dz > 0.2 && dh > 1e-6 && dz/dh > 1.5){
      console.log('  STEEP dz/dh=' + (dz/dh).toFixed(1) + ' z ' + a[2].toFixed(2) + '->' + b[2].toFixed(2) +
        ' at (' + b[0].toFixed(2) + ',' + b[1].toFixed(2) + ')');
      ++steep;
    }
  }
  for (const p of r.pts){
    const [i, j, k] = cellOf(p[0], p[1], p[2]);
    const list = m.treadRaw[i + j*m.nx] || [];
    let bd = 99; for (const t of list) bd = Math.min(bd, Math.abs(t - k));
    if (m.stairCol[i + j*m.nx] && bd > 2){
      const fb = F(i, j, k-1);
      // Same read as standsOn: the surface stamp, a self-stamped plate on the
      // free cell itself, or a flat face on the mass one/two below.
      const on = m.surfaceCell(i,j,k) || m.topFace[F(i,j,k)] === 1 ||
        (m.data[fb] === 2 && (m.topFace[fb] === 1 || (k >= 2 && m.topFace[fb - m.nx*m.ny] === 1)));
      if (!on){ ++air; if (air <= 8) console.log('  AIR z ' + p[2].toFixed(2) + ' at (' + p[0].toFixed(2) + ',' + p[1].toFixed(2) + ') k=' + k + ' nearestTread=' + bd); }
    }
  }
  console.log('  steep edges=' + steep + ' air pts=' + air + ' back=' + back + ' of ' + r.pts.length);
  // Decoupling guarantee: every flight-segment point lies on the recognised
  // centre line itself (the same polyline the overlay draws), not on a
  // re-projection of it. Point-to-segment distance over all flight polylines.
  if (r.segs){
    const dSeg = (p, a, b) => {
      const abx = b[0]-a[0], aby = b[1]-a[1], abz = b[2]-a[2];
      const l2 = abx*abx + aby*aby + abz*abz;
      const t = l2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((p[0]-a[0])*abx + (p[1]-a[1])*aby + (p[2]-a[2])*abz)/l2));
      return Math.hypot(p[0]-(a[0]+t*abx), p[1]-(a[1]+t*aby), p[2]-(a[2]+t*abz));
    };
    let off = 0, maxD = 0, nf = 0;
    for (let t = 0; t < r.pts.length; ++t){
      if (r.segs[t] !== 'f') continue;
      ++nf;
      let bd = 1e30;
      for (const fl of m.flights){
        if (!fl.macroOK) continue;
        for (let q = 1; q < fl.rowPts.length; ++q)
          bd = Math.min(bd, dSeg(r.pts[t], fl.rowPts[q-1], fl.rowPts[q]));
      }
      if (bd > maxD) maxD = bd;
      if (bd >= 0.02){ ++off; console.log('  SPINE-OFF d=' + bd.toFixed(3) + ' at (' + r.pts[t].map(v => v.toFixed(2)).join(',') + ')'); }
    }
    console.log('  flight pts=' + nf + ' spine off=' + off + ' maxDev=' + maxD.toFixed(4));
  }
  const zs = r.pts.map(p => p[2]);
  console.log('  z range ' + Math.min(...zs).toFixed(2) + '..' + Math.max(...zs).toFixed(2));
}

// full point dump for the second task + max deviation from the flight surface
const r2 = planPath(m, -10.1, 7.0, 4.15, -8.1, 7.6, 8.7, null);
if (!r2.error){
  console.log('\n=== full pts task2');
  for (const p of r2.pts) console.log('  ' + p.map(v => v.toFixed(2)).join(', '));
}
