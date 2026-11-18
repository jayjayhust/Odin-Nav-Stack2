// Stairs: a flight is a run of columns that actually step, climbs, and whose
// tread centre advances along the run as it rises.
//
// Only the read-only grid facade is used, so this can be run on a synthetic
// grid without the page. Three things were learned the hard way on a real scan:
//
//   - a riser is up to one step height, not always 1-2 cells: at 0.45 m steps
//     on a 0.15 m grid that is three cells, and a fixed 2 missed whole flights;
//   - the axis must come from where the tread centre actually goes, not from
//     the cluster's silhouette: a handrail or a landing rim drags a PCA axis
//     sideways, and then every perp/along test - and the drawn band - is off
//     the stair even though the cells are right;
//   - what a flight claims is the walkable top surface at each height, not the
//     columns that happened to step. Two thirds of a tread is why the overlay
//     looked detached, and leftovers of a bad axis read as scattered debris.
(function () {
  'use strict';

  const MIN_CELLS = 30;            // a flight covers more than a corner
  const MIN_RISE = 4;              // cells; flatter than this is a floor
  const MIN_HALF = 0.18, MAX_HALF = 1.10;
  const MIN_SLOPE = 0.12, MAX_SLOPE = 2.2;
  const MAX_LEN = 6.5;             // storey scale, not one flight
  const MIN_HEIGHTS = 6;           // fewer steps than a stair
  const MIN_TRAVEL_RISE = 0.30;    // |travel| >= this * rise
  const MIN_R2 = 0.35;             // and the centre must move in a straight line
  const MIN_COVER = 0.45;          // claimed tread / band area, rejects slivers

  // Principal direction of a point set, in the grid's xy plane.
  function axisOf(points) {
    let mx = 0, my = 0;
    for (const p of points) { mx += p[0]; my += p[1]; }
    mx /= points.length; my /= points.length;
    let sxx = 0, syy = 0, sxy = 0;
    for (const p of points) {
      const dx = p[0] - mx, dy = p[1] - my;
      sxx += dx*dx; syy += dy*dy; sxy += dx*dy;
    }
    const ang = 0.5 * Math.atan2(2*sxy, sxx - syy);
    return [Math.cos(ang), Math.sin(ang)];
  }

  function detect(g, opts) {
    const {res, lo, nx, ny, nz, walkable, tread} = g;
    const maxStep = Math.max(1, Math.round((opts && opts.maxStepZ) || 3));

    // A column steps when its tread height differs from a neighbour's by at
    // least one cell and no more than one riser. A sloping slab does not step;
    // a wall face has no walkable top at the same height on both sides.
    const stepped = new Int32Array(nx*ny).fill(-1);
    for (let j = 0; j < ny; ++j)
      for (let i = 0; i < nx; ++i) {
        const f = i + j*nx, rk = tread[f];
        if (rk < 1) continue;
        let step = 0;
        for (const [di, dj] of [[1,0],[-1,0],[0,1],[0,-1]]) {
          const i2 = i + di, j2 = j + dj;
          if (i2 < 0 || j2 < 0 || i2 >= nx || j2 >= ny) continue;
          const rk2 = tread[i2 + j2*nx];
          if (rk2 < 1) continue;
          const d = Math.abs(rk2 - rk);
          if (d >= 1 && d <= maxStep) step = Math.max(step, d);
        }
        if (step) stepped[f] = rk;
      }

    const seen = new Uint8Array(nx*ny), stack = [];
    const regions = [];
    const dbg = (typeof window !== 'undefined' && window.__TERRAIN_DBG) ? [] : null;
    if (dbg) window.__TERRAIN_REJECTS = dbg;
    const drop = (why, info) => { if (dbg) dbg.push(Object.assign({why}, info)); };

    for (let j0 = 0; j0 < ny; ++j0)
      for (let i0 = 0; i0 < nx; ++i0) {
        const id0 = i0 + j0*nx;
        if (stepped[id0] < 0 || seen[id0]) continue;
        stack.length = 0; stack.push(id0); seen[id0] = 1;
        const mem = [];
        while (stack.length) {
          const q = stack.pop(); mem.push(q);
          const qj = (q / nx) | 0, qi = q - qj*nx;
          for (const [di, dj] of [[1,0],[-1,0],[0,1],[0,-1]]) {
            const i2 = qi + di, j2 = qj + dj;
            if (i2 < 0 || j2 < 0 || i2 >= nx || j2 >= ny) continue;
            const nq = i2 + j2*nx;
            if (stepped[nq] < 0 || seen[nq]) continue;
            if (Math.abs(stepped[nq] - stepped[q]) > maxStep) continue;
            seen[nq] = 1; stack.push(nq);
          }
        }
        if (mem.length < MIN_CELLS) { drop('columns', {n: mem.length}); continue; }

        // World positions of the cluster's columns, grouped by their height.
        const byK = new Map();
        let rkMin = 1e9, rkMax = -1e9;
        const cols = [];
        for (const q of mem) {
          const jj = (q / nx) | 0, ii = q - jj*nx;
          const p = [lo[0] + (ii + 0.5)*res, lo[1] + (jj + 0.5)*res];
          cols.push(p);
          const k = stepped[q];
          rkMin = Math.min(rkMin, k); rkMax = Math.max(rkMax, k);
          const list = byK.get(k) || (byK.set(k, []), byK.get(k));
          list.push(p);
        }
        if (rkMax - rkMin < MIN_RISE) { drop('rise', {n: mem.length, rise: rkMax - rkMin}); continue; }

        // Axis from the footprint. A refinement on the tread centres was tried
        // and rejected: on the reference scan it rotated a correct axis by 10
        // degrees, and a wrong axis is worse than a round one.
        let axis = axisOf(cols);
        const ux = axis[0], uy = axis[1];

        // Frame at the centroid of all columns, measured with this axis.
        let mx = 0, my = 0;
        for (const p of cols) { mx += p[0]; my += p[1]; }
        mx /= cols.length; my /= cols.length;

        let alongMin = 1e9, alongMax = -1e9;
        const perps = [];
        for (const p of cols) {
          const ox = p[0] - mx, oy = p[1] - my;
          const al = ox*ux + oy*uy;
          alongMin = Math.min(alongMin, al); alongMax = Math.max(alongMax, al);
          perps.push(Math.abs(-ox*uy + oy*ux));
        }
        perps.sort((a, b) => a - b);
        const half = perps[Math.floor(perps.length * 0.9)];
        const length = Math.max(alongMax - alongMin, res);
        const rise = (rkMax - rkMin) * res;
        const slope = rise / length;
        if (half < MIN_HALF || half > MAX_HALF) { drop('half', {n: mem.length, cx: +mx.toFixed(2), cy: +my.toFixed(2), half: +half.toFixed(2)}); continue; }
        if (slope < MIN_SLOPE || slope > MAX_SLOPE) { drop('slope', {n: mem.length, cx: +mx.toFixed(2), cy: +my.toFixed(2), slope: +slope.toFixed(2), len: +length.toFixed(2)}); continue; }
        if (length > MAX_LEN) { drop('length', {n: mem.length, cx: +mx.toFixed(2), cy: +my.toFixed(2), len: +length.toFixed(2)}); continue; }

        // One centre for the whole run leaves the first steps out: at the foot
        // they merge into the floor. Scan the walkable section at each height.
        const kLo = Math.max(0, Math.round(rkMin) - 2);
        const kHi = Math.min(nz - 1, Math.round(rkMax) + 2);
        const nK = kHi - kLo + 1;
        const spine = new Float32Array(nK).fill(NaN);
        const halfK = new Float32Array(nK).fill(NaN);
        const along = new Float32Array(nK).fill(NaN);
        const mn = new Float32Array(nK).fill(Infinity);
        const mx2 = new Float32Array(nK).fill(-Infinity);
        const alSum = new Float64Array(nK), alNum = new Int32Array(nK);

        for (const q of mem) {
          const kk = stepped[q];
          if (kk < kLo || kk > kHi) continue;
          const jj = (q / nx) | 0, ii = q - jj*nx;
          const ox = lo[0] + (ii+0.5)*res - mx, oy = lo[1] + (jj+0.5)*res - my;
          alSum[kk - kLo] += ox*ux + oy*uy; alNum[kk - kLo] += 1;
        }

        // The tread's own section at each height: walkable, on this flight's
        // footprint, inside the run. This is what gets claimed, so the band has
        // to be measured on the surface, not on the sampled columns.
        const section = new Map();
        const bi0 = Math.max(0, Math.floor((mx - lo[0] - half - 0.30) / res));
        const bi1 = Math.min(nx - 1, Math.ceil((mx - lo[0] + half + 0.30) / res));
        const bj0 = Math.max(0, Math.floor((my - lo[1] - half - 0.30) / res));
        const bj1 = Math.min(ny - 1, Math.ceil((my - lo[1] + half + 0.30) / res));
        let bandCells = 0;
        for (let k = kLo; k <= kHi; ++k) {
          const at = k - kLo, list = [];
          for (let j = bj0; j <= bj1; ++j)
            for (let i = bi0; i <= bi1; ++i) {
              const f = i + j*nx;
              // the tread's top surface; one cell of slack, because the raster
              // gives a step edge either height depending on where the facet
              // crosses the cell, and a strict test leaves holes in the tread
              if (Math.abs(tread[f] - k) > 1) continue;
              if (!walkable(i, j, k)) continue;
              const ox = lo[0] + (i+0.5)*res - mx, oy = lo[1] + (j+0.5)*res - my;
              if (Math.abs(ox*ux + oy*uy) > length/2 + 0.30) continue;
              const perp = -ox*uy + oy*ux;
              if (Math.abs(perp) > half + 0.20) continue;
              list.push(f);
              mn[at] = Math.min(mn[at], perp); mx2[at] = Math.max(mx2[at], perp);
            }
          section.set(at, list);
          bandCells += list.length;
        }
        let last = NaN;
        for (let at = 0; at < nK; ++at) {
          if (!(mn[at] <= mx2[at])) { spine[at] = last; continue; }
          spine[at] = (mn[at] + mx2[at]) / 2;
          halfK[at] = Math.max((mx2[at] - mn[at]) / 2, 0.10);
          last = spine[at];
          along[at] = alNum[at] ? alSum[at] / alNum[at] : 0;
        }
        for (let at = nK - 1; at >= 0; --at)
          if (!(spine[at] === spine[at]) && at + 1 < nK) spine[at] = spine[at+1];

        // A cluster of stepped columns is not a flight unless its tread centre
        // advances along the run as it rises: otherwise it is a step-edged slab
        // or a landing rim, and charging it would tilt the plan where it is flat.
        let nH = 0, sA = 0, sL = 0, sAA = 0, sLL = 0, sAL = 0;
        for (let at = 0; at < nK; ++at) {
          if (alNum[at] === 0) continue;
          const L = alSum[at] / alNum[at];
          ++nH; sA += at; sL += L; sAA += at*at; sLL += L*L; sAL += at*L;
        }
        if (nH < MIN_HEIGHTS) { drop('heights', {n: mem.length, cx: +mx.toFixed(2), cy: +my.toFixed(2), nH}); continue; }
        const vA = sAA - sA*sA/nH, vL = sLL - sL*sL/nH, cov = sAL - sA*sL/nH;
        const r2 = (vA > 1e-9 && vL > 1e-9) ? (cov*cov)/(vA*vL) : 0;
        const travel = Math.abs(cov) / Math.max(1e-9, vA) * (nH - 1);
        if (travel < MIN_TRAVEL_RISE * rise || r2 < MIN_R2) {
          drop('straightness', {n: mem.length, cx: +mx.toFixed(2), cy: +my.toFixed(2),
                                travel: +travel.toFixed(2), r2: +r2.toFixed(2), rise: +rise.toFixed(2)});
          continue;
        }

        // Per-height averages zig-zag where a riser edge shares a step's height;
        // a flight's centre is linear in height, so keep the fit, clamped to the
        // run so the first and last step do not overshoot.
        const slopeA = cov / vA, intercept = (sL - slopeA * sA) / nH;
        for (let at = 0; at < nK; ++at)
          along[at] = Math.max(-length/2, Math.min(length/2, intercept + slopeA * at));

        // Claim the tread, one quad per column at that column's own top: a
        // column listed at every height near it would stack into a curtain.
        const claimed = [];
        for (let j = bj0; j <= bj1; ++j)
          for (let i = bi0; i <= bi1; ++i) {
            const f = i + j*nx, k = tread[f];
            if (k < kLo || k > kHi) continue;
            if (!walkable(i, j, k)) continue;
            const ox = lo[0] + (i+0.5)*res - mx, oy = lo[1] + (j+0.5)*res - my;
            if (Math.abs(ox*ux + oy*uy) > length/2 + 0.30) continue;
            const at = k - kLo;
            const perp = -ox*uy + oy*ux;
            // the tread's own section at this height, or the flight's half
            // width where the raster gives that height no section
            const b0 = (mn[at] <= mx2[at]) ? mn[at] - 0.20 : -half - 0.20;
            const b1 = (mn[at] <= mx2[at]) ? mx2[at] + 0.20 :  half + 0.20;
            if (perp < b0 || perp > b1) continue;
            let same = 0;
            for (const [di, dj] of [[1,0],[-1,0],[0,1],[0,-1]]) {
              const i2 = i + di, j2 = j + dj;
              if (i2 < 0 || j2 < 0 || i2 >= nx || j2 >= ny) continue;
              if (Math.abs(tread[i2 + j2*nx] - k) <= 1) ++same;
            }
            // a lone top - a handrail post, a stray box - is not tread
            if (same === 0) continue;
            claimed.push(f + k*nx*ny);
          }

        regions.push({
          kind: 'stairs',
          centre: [mx, my], axis: [ux, uy],
          half, length, rise,
          z: [lo[2] + rkMin*res - 0.15, lo[2] + rkMax*res + 0.35],
          cells: claimed,
          steps: {kLo, kHi, spine, half: halfK, along},
          metrics: {columns: mem.length, cells: claimed.length, heights: nH,
                    travel: +travel.toFixed(2), r2: +r2.toFixed(3),
                    slope: +slope.toFixed(2)},
        });
      }
    return regions;
  }

  window.TerrainSemantics.register({
    id: 'stairs',
    title: '楼梯段',
    version: 4,
    detects: 'walkable stair flights with per-step tread centres',
    detect,
  });
})();
