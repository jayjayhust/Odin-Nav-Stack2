// Stairs: a flight means a cluster of columns that actually step, climbs,
// and whose tread centre advances along the run as it rises.
//
// Every rule below reads the grid only through the facade, so this file can be
// exercised on a synthetic grid without the page. The cells it claims are the
// walkable cross-section at each height - the surface the planner is charged
// for - not just the sampled columns that happened to step, because painting
// two thirds of a tread is how the overlay came to look detached from the stair.
(function () {
  'use strict';

  const MIN_CELLS = 30;          // a flight covers more than a corner
  const MIN_RISE = 4;            // cells; anything flatter is a floor
  const MIN_HALF = 0.18, MAX_HALF = 1.10;
  const MIN_SLOPE = 0.12, MAX_SLOPE = 2.2;
  const MAX_LEN = 6.5;           // storey scale, not one flight
  const MIN_HEIGHTS = 6;         // fewer steps than a stair
  const MIN_TRAVEL_RISE = 0.30;  // tread centre must move, |travel| >= this * rise
  const MIN_R2 = 0.35;           // and it must move in a straight line

  function detect(g) {
    const {res, lo, nx, ny, nz, walkable, tread} = g;
    const regions = [];

    // A column steps when its tread height differs from a neighbour's by one or
    // two cells and no more - a real riser. A gently sloping slab does not step,
    // and a wall face has no walkable top at every height, so neither passes.
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
          if (d >= 1 && d <= 2) step = Math.max(step, d);
        }
        if (step) stepped[f] = rk;
      }

    const seen = new Uint8Array(nx*ny), stack = [];
    for (let j0 = 0; j0 < ny; ++j0)
      for (let i0 = 0; i0 < nx; ++i0) {
        const id0 = i0 + j0*nx;
        if (stepped[id0] < 0 || seen[id0]) continue;
        stack.length = 0; stack.push(id0); seen[id0] = 1;
        const mem = [];
        let rkMin = 1e9, rkMax = -1e9;
        while (stack.length) {
          const q = stack.pop(); mem.push(q);
          const qj = (q / nx) | 0, qi = q - qj*nx;
          rkMin = Math.min(rkMin, stepped[q]); rkMax = Math.max(rkMax, stepped[q]);
          for (const [di, dj] of [[1,0],[-1,0],[0,1],[0,-1]]) {
            const i2 = qi + di, j2 = qj + dj;
            if (i2 < 0 || j2 < 0 || i2 >= nx || j2 >= ny) continue;
            const nq = i2 + j2*nx;
            if (stepped[nq] < 0 || seen[nq]) continue;
            if (Math.abs(stepped[nq] - stepped[q]) > 2.0) continue;
            seen[nq] = 1; stack.push(nq);
          }
        }
        if (mem.length < MIN_CELLS) continue;
        if (rkMax - rkMin < MIN_RISE) continue;

        // principal axis of the footprint, so a diagonal flight fits as one run
        let mx = 0, my = 0;
        for (const q of mem) {
          mx += lo[0] + ((q % nx) + 0.5)*res;
          my += lo[1] + (((q / nx) | 0) + 0.5)*res;
        }
        mx /= mem.length; my /= mem.length;
        let sxx = 0, syy = 0, sxy = 0;
        for (const q of mem) {
          const wx = lo[0] + ((q % nx) + 0.5)*res - mx;
          const wy = lo[1] + (((q / nx) | 0) + 0.5)*res - my;
          sxx += wx*wx; syy += wy*wy; sxy += wx*wy;
        }
        sxx /= mem.length; syy /= mem.length; sxy /= mem.length;
        const ang = 0.5 * Math.atan2(2*sxy, sxx - syy);
        const ux = Math.cos(ang), uy = Math.sin(ang);
        let alongMin = 1e9, alongMax = -1e9;
        const perps = [];
        for (const q of mem) {
          const wx = lo[0] + ((q % nx) + 0.5)*res - mx;
          const wy = lo[1] + (((q / nx) | 0) + 0.5)*res - my;
          const al = wx*ux + wy*uy;
          alongMin = Math.min(alongMin, al); alongMax = Math.max(alongMax, al);
          perps.push(Math.abs(-wx*uy + wy*ux));
        }
        perps.sort((a, b) => a - b);
        const half = perps[Math.floor(perps.length * 0.9)];
        const length = Math.max(alongMax - alongMin, res);
        const rise = (rkMax - rkMin) * res;
        const slope = rise / length;
        if (half < MIN_HALF || half > MAX_HALF) continue;
        if (slope < MIN_SLOPE || slope > MAX_SLOPE) continue;
        if (length > MAX_LEN) continue;

        // One centre for the whole run leaves the first steps out: at the foot
        // they merge into the floor and the cluster above misses them. Scan the
        // walkable cross-section at each height, keep a centre per step, and
        // claim those cells - the tread, not the sampled columns on it.
        const kLo = Math.max(0, Math.round(rkMin) - 2);
        const kHi = Math.min(nz - 1, Math.round(rkMax) + 2);
        const nK = kHi - kLo + 1;
        const spine = new Float32Array(nK).fill(NaN);
        const halfK = new Float32Array(nK).fill(NaN);
        const along = new Float32Array(nK).fill(NaN);
        const mn = new Float32Array(nK).fill(Infinity);
        const mx2 = new Float32Array(nK).fill(-Infinity);
        const alSum = new Float64Array(nK), alNum = new Int32Array(nK);
        const band = new Map();                     // at -> [i,j] inside the tread

        for (const id of mem) {
          const jj = (id / nx) | 0, ii = id - jj*nx;
          const kk = stepped[id];
          if (kk < kLo || kk > kHi) continue;
          const al = (lo[0] + (ii+0.5)*res - mx)*ux + (lo[1] + (jj+0.5)*res - my)*uy;
          alSum[kk - kLo] += al; alNum[kk - kLo] += 1;
        }
        const bi0 = Math.max(0, Math.floor((mx - lo[0] - half - 0.35) / res));
        const bi1 = Math.min(nx - 1, Math.ceil((mx - lo[0] + half + 0.35) / res));
        const bj0 = Math.max(0, Math.floor((my - lo[1] - half - 0.35) / res));
        const bj1 = Math.min(ny - 1, Math.ceil((my - lo[1] + half + 0.35) / res));
        for (let k = kLo; k <= kHi; ++k)
          for (let j = bj0; j <= bj1; ++j)
            for (let i = bi0; i <= bi1; ++i) {
              if (!walkable(i, j, k)) continue;
              const ox = lo[0] + (i+0.5)*res - mx, oy = lo[1] + (j+0.5)*res - my;
              if (Math.abs(ox*ux + oy*uy) > length/2 + 0.35) continue;
              const perp = -ox*uy + oy*ux;
              if (Math.abs(perp) > half + 0.20) continue;
              const at = k - kLo;
              mn[at] = Math.min(mn[at], perp); mx2[at] = Math.max(mx2[at], perp);
              const list = band.get(at) || (band.set(at, []), band.get(at));
              list.push(i + j*nx);
            }
        let last = NaN;
        for (let at = 0; at < nK; ++at) {
          if (mn[at] > mx2[at]) { spine[at] = last; continue; }
          spine[at] = (mn[at] + mx2[at]) / 2;
          halfK[at] = Math.max((mx2[at] - mn[at]) / 2, 0.10);
          last = spine[at];
          along[at] = alNum[at] ? alSum[at] / alNum[at] : 0;
        }
        for (let at = nK - 1; at >= 0; --at)
          if (spine[at] !== spine[at] && at + 1 < nK) spine[at] = spine[at+1];

        // A cluster of stepped columns is not a flight unless its tread centre
        // advances along the run as it rises: otherwise it is a step-edged slab
        // or a landing rim, and charging it would tilt the plan where it is flat.
        let nH = 0, sA = 0, sL = 0, sAA = 0, sLL = 0, sAL = 0;
        for (let at = 0; at < nK; ++at) {
          if (alNum[at] === 0) continue;
          ++nH; sA += at; sL += alSum[at] / alNum[at];
          sAA += at*at; sLL += (alSum[at] / alNum[at])**2; sAL += at * (alSum[at] / alNum[at]);
        }
        if (nH < MIN_HEIGHTS) continue;
        const vA = sAA - sA*sA/nH, vL = sLL - sL*sL/nH, cov = sAL - sA*sL/nH;
        const r2 = (vA > 1e-9 && vL > 1e-9) ? (cov*cov)/(vA*vL) : 0;
        const travel = Math.abs(cov) / Math.max(1e-9, vA) * (nH - 1);
        if (travel < MIN_TRAVEL_RISE * rise || r2 < MIN_R2) continue;

        // Per-height averages zig-zag where a riser edge shares a step's height;
        // a flight's centre is linear in height, so keep the fit, clamped to the
        // run so the first and last step do not overshoot.
        const slopeA = cov / vA, intercept = (sL - slopeA * sA) / nH;
        for (let at = 0; at < nK; ++at)
          along[at] = Math.max(-length/2, Math.min(length/2, intercept + slopeA * at));

        // Claim the tread: every walkable cell inside the per-height band. The
        // stepped columns alone cover maybe two thirds of a tread, which is why
        // the drawn patch looked detached from the stair it described.
        const cells = [];
        for (let at = 0; at < nK; ++at) {
          if (mn[at] > mx2[at]) continue;
          const k = kLo + at;
          const lo2 = mn[at] - 0.05, hi2 = mx2[at] + 0.05;
          for (const c of (band.get(at) || [])) {
            // the column's own top surface only: `walkable` is also true for
            // the free space above a floor, and claiming that turns the tread
            // into a solid block stacked over the whole footprint.
            if (tread[c] !== k) continue;
            const ii = c % nx, jj = (c / nx) | 0;
            const ox = lo[0] + (ii+0.5)*res - mx, oy = lo[1] + (jj+0.5)*res - my;
            const perp = -ox*uy + oy*ux;
            if (perp < lo2 || perp > hi2) continue;
            if (Math.abs(ox*ux + oy*uy) > length/2 + 0.15) continue;
            cells.push(c + k*nx*ny);
          }
        }
        for (const c of mem) cells.push(c + stepped[c]*nx*ny);   // plus the columns
        const uniq = Array.from(new Set(cells));

        regions.push({
          kind: 'stairs',
          centre: [mx, my], axis: [ux, uy],
          half, length, rise,
          z: [lo[2] + rkMin*res - 0.15, lo[2] + rkMax*res + 0.35],
          cells: uniq,
          steps: {kLo, kHi, spine, half: halfK, along},
          metrics: {columns: mem.length, cells: uniq.length, heights: nH,
                    travel: +travel.toFixed(2), r2: +r2.toFixed(3),
                    slope: +slope.toFixed(2)},
        });
      }
    return regions;
  }

  window.TerrainSemantics.register({
    id: 'stairs',
    title: '楼梯段',
    version: 3,
    detects: 'walkable stair flights with per-step tread centres',
    detect,
  });
})();
