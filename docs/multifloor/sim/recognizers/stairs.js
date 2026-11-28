// Stair flights: stepped tread columns, clustered and fitted to a run.
//
// The grid facade carries every standable tread height per column (treadKs),
// not just the topmost one: a switchback tower stacks flights in the same XY
// columns and a grandstand run can pass under a stair body, so a single height
// per column hides every flight but the top one. Clustering therefore runs on
// (column, height) nodes - two nodes join when their columns touch and their
// heights stay within one riser, while two heights in the SAME column never
// join, which is what keeps stacked flights apart.
//
// Filters exist because "stepped columns" alone is not a stair:
//   - a slab edge or a landing rim steps too, but its centre does not travel
//     as it rises, so the straightness fit rejects it;
//   - a handrail post steps against the tread beside it, but a lone column
//     cluster is under the minimum size;
//   - what a flight claims is the walkable top surface at each height, not the
//     columns that happened to step. Two thirds of a tread is why the overlay
//     looked detached, and leftovers of a bad axis read as scattered debris.
(function () {
  'use strict';

  const MIN_CELLS = 20;            // a short flight is still a flight
  const MIN_RISE = 4;              // cells; flatter than this is a floor
  const MIN_HALF = 0.18, MAX_HALF = 1.10;
  const MIN_SLOPE = 0.12, MAX_SLOPE = 2.2;
  const MAX_LEN = 6.5;             // storey scale, not one flight
  const MIN_HEIGHTS = 3;           // three steps and a rise is a stair
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
    const area = nx*ny;

    // Ascending standable heights per column; a facade without treadKs (an
    // older exporter) falls back to the single top surface.
    const ks = new Array(area);
    for (let f = 0; f < area; ++f)
      ks[f] = g.treadKs ? g.treadKs[f] : (tread[f] >= 1 ? [tread[f]] : null);
    const near = (f, k, slack) => {
      const list = ks[f];
      if (!list) return false;
      for (const k2 of list) if (Math.abs(k2 - k) <= slack) return true;
      return false;
    };

    // A node steps when a neighbouring column carries a height at least one
    // cell and at most one riser away. A sloping slab does not step; a wall
    // face has no walkable top at the same height on both sides.
    const stepped = new Uint8Array(area*nz);
    for (let j = 0; j < ny; ++j)
      for (let i = 0; i < nx; ++i) {
        const f = i + j*nx, list = ks[f];
        if (!list) continue;
        for (const k of list) {
          let step = 0;
          for (const [di, dj] of [[1,0],[-1,0],[0,1],[0,-1]]) {
            const i2 = i + di, j2 = j + dj;
            if (i2 < 0 || j2 < 0 || i2 >= nx || j2 >= ny) continue;
            const f2 = i2 + j2*nx, list2 = ks[f2];
            if (!list2) continue;
            for (const k2 of list2) {
              const d = Math.abs(k2 - k);
              if (d >= 1 && d <= maxStep) { step = 1; break; }
            }
            if (step) break;
          }
          if (step) stepped[f + k*area] = 1;
        }
      }

    const seen = new Uint8Array(area*nz), stack = [];
    const regions = [];
    const dbg = (typeof window !== 'undefined' && window.__TERRAIN_DBG) ? [] : null;
    if (dbg) window.__TERRAIN_REJECTS = dbg;
    const drop = (why, info) => { if (dbg) dbg.push(Object.assign({why}, info)); };

    // Fit one cluster of nodes; a switchback joined through its landing fails
    // the straight-line test, so split it across the minor axis and fit the
    // two runs again instead of throwing the whole tower away.
    function fit(mem, depth) {
      if (mem.length < MIN_CELLS) {
        let sx = 0, sy = 0;
        for (const q of mem){ const f = q % area, jj = (f / nx) | 0; sx += f - jj*nx; sy += jj; }
        drop('columns', {n: mem.length,
          cx: +(lo[0] + (sx/mem.length + 0.5)*res).toFixed(2),
          cy: +(lo[1] + (sy/mem.length + 0.5)*res).toFixed(2)});
        return [];
      }
      const cols = [];
      let rkMin = 1e9, rkMax = -1e9;
      for (const q of mem) {
        const f = q % area, k = (q / area) | 0;
        const jj = (f / nx) | 0, ii = f - jj*nx;
        cols.push([lo[0] + (ii + 0.5)*res, lo[1] + (jj + 0.5)*res]);
        rkMin = Math.min(rkMin, k); rkMax = Math.max(rkMax, k);
      }
      if (rkMax - rkMin < MIN_RISE) {
        let sx = 0, sy = 0;
        for (const p of cols){ sx += p[0]; sy += p[1]; }
        drop('rise', {n: mem.length, rise: rkMax - rkMin,
          cx: +(sx/cols.length).toFixed(2), cy: +(sy/cols.length).toFixed(2)});
        return [];
      }

      // Frame at the centroid of all columns.
      let mx = 0, my = 0;
      for (const p of cols) { mx += p[0]; my += p[1]; }
      mx /= cols.length; my /= cols.length;

      const kLo = Math.max(0, Math.round(rkMin) - 2);
      const kHi = Math.min(nz - 1, Math.round(rkMax) + 2);
      const nK = kHi - kLo + 1;

      // A flight's centre travels along the run as it rises. The footprint's
      // principal axis is only a guess: a stair wider than it is long (a podium
      // step, a landing mouth) has its WIDTH as principal axis, and measuring
      // along it reads travel zero and throws the flight away. Score both
      // candidate axes by how far the tread centre actually travels per height
      // and keep the better one.
      const heightStats = (ax, ay, mask) => {
        const alSum = new Float64Array(nK), alNum = new Int32Array(nK);
        for (const q of mem) {
          const f = q % area, kk = (q / area) | 0;
          if (kk < kLo || kk > kHi) continue;
          const jj = (f / nx) | 0, ii = f - jj*nx;
          const ox = lo[0] + (ii+0.5)*res - mx, oy = lo[1] + (jj+0.5)*res - my;
          alSum[kk - kLo] += ox*ax + oy*ay; alNum[kk - kLo] += 1;
        }
        let nH = 0, sA = 0, sL = 0, sAA = 0, sLL = 0, sAL = 0;
        for (let at = 0; at < nK; ++at) {
          if (alNum[at] === 0 || (mask && !mask[at])) continue;
          const L = alSum[at] / alNum[at];
          ++nH; sA += at; sL += L; sAA += at*at; sLL += L*L; sAL += at*L;
        }
        const vA = sAA - sA*sA/nH, vL = sLL - sL*sL/nH, cov = sAL - sA*sL/nH;
        const r2 = (vA > 1e-9 && vL > 1e-9) ? (cov*cov)/(vA*vL) : 0;
        const travel = nH > 1 ? Math.abs(cov) / Math.max(1e-9, vA) * (nH - 1) : 0;
        return {alSum, alNum, nH, sA, sL, sAA, sLL, sAL, vA, vL, cov, r2, travel};
      };
      // A riser's top edge stamps a strip of intermediate heights between the
      // treads - a quarter of a step up, a few cells wide. Those bleed rows
      // zig-zag the per-height centre and tilt the fit. A tread carries the
      // flight's full width; keep only heights whose occupancy reaches two
      // fifths of the busiest one. The thin rows still bridge the BFS, they
      // just do not steer the fit.
      const raw = heightStats(1, 0);
      let busy = 0;
      for (let at = 0; at < nK; ++at) busy = Math.max(busy, raw.alNum[at]);
      const hMask = new Uint8Array(nK);
      for (let at = 0; at < nK; ++at) hMask[at] = raw.alNum[at] >= 0.4*busy ? 1 : 0;
      const pca = axisOf(cols);
      const cand = [pca, [-pca[1], pca[0]]];
      const stats = cand.map(a => heightStats(a[0], a[1], hMask));
      const pick = (stats[1].travel > stats[0].travel + 1e-9 ||
                    (Math.abs(stats[1].travel - stats[0].travel) <= 1e-9 &&
                     stats[1].r2 > stats[0].r2)) ? 1 : 0;
      let ux = cand[pick][0], uy = cand[pick][1];
      const {alSum, alNum, nH, sA, sL, sAA, sLL, sAL, vA, vL, cov, r2, travel} = stats[pick];
      // the tread centre line: where the run sits at each height. The band
      // and the claim accept cells near this line at their own height, not
      // anywhere inside the run's whole span - a floor plate under the foot
      // or a landing beside the head is a step off the line and not tread.
      const regSlope = vA > 1e-9 ? cov / vA : 0;
      const regInt = nH ? (sL - regSlope * sA) / nH : 0;
      const regAt = at => regInt + regSlope * at;

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
      let length = Math.max(alongMax - alongMin, res);
      const rise = (rkMax - rkMin) * res;
      const slope = rise / length;
      // The width cap belongs to a single run. A switchback tower clusters
      // side-by-side flights whose combined span exceeds it; those must reach
      // the straightness test, which splits them along the minor axis. Only
      // the leaf fits enforce the cap.
      if (half < MIN_HALF) { drop('half', {n: mem.length, cx: +mx.toFixed(2), cy: +my.toFixed(2), half: +half.toFixed(2)}); return []; }
      if (slope < MIN_SLOPE || slope > MAX_SLOPE) { drop('slope', {n: mem.length, cx: +mx.toFixed(2), cy: +my.toFixed(2), slope: +slope.toFixed(2), len: +length.toFixed(2)}); return []; }
      // The length cap too is a single-run property: a tower's merged runs
      // span more than one flight, and the split below has to see them.

      // One centre for the whole run leaves the first steps out: at the foot
      // they merge into the floor. Scan the walkable section at each height.
      const spine = new Float32Array(nK).fill(NaN);
      const halfK = new Float32Array(nK).fill(NaN);
      const along = new Float32Array(nK).fill(NaN);
      const mn = new Float32Array(nK).fill(Infinity);
      const mx2 = new Float32Array(nK).fill(-Infinity);

      // The tread's own section at each height: walkable, on this flight's
      // footprint, inside the run. This is what gets claimed, so the band has
      // to be measured on the surface, not on the sampled columns.
      // the band box is the AABB of the oriented run rectangle: half across,
      // length/2 along - a square box of half clipped the end steps off
      const bex = Math.abs(ux)*(length/2 + 0.30) + Math.abs(uy)*(half + 0.30);
      const bey = Math.abs(uy)*(length/2 + 0.30) + Math.abs(ux)*(half + 0.30);
      const bi0 = Math.max(0, Math.floor((mx - lo[0] - bex) / res));
      const bi1 = Math.min(nx - 1, Math.ceil((mx - lo[0] + bex) / res));
      const bj0 = Math.max(0, Math.floor((my - lo[1] - bey) / res));
      const bj1 = Math.min(ny - 1, Math.ceil((my - lo[1] + bey) / res));
      let bandCells = 0;
      for (let k = kLo; k <= kHi; ++k) {
        const at = k - kLo, list = [];
        if (!hMask[at]) continue;      // a bleed strip is not a tread section
        for (let j = bj0; j <= bj1; ++j)
          for (let i = bi0; i <= bi1; ++i) {
            const f = i + j*nx;
            // the tread's own surface at this height: with every height per
            // column available, a slack of one would count the hovering cell
            // above each tread as band too and halve the cover ratio
            if (!near(f, k, 0)) continue;
            if (!walkable(i, j, k)) continue;
            const ox = lo[0] + (i+0.5)*res - mx, oy = lo[1] + (j+0.5)*res - my;
            if (Math.abs(ox*ux + oy*uy - regAt(at)) > 0.45) continue;
            const perp = -ox*uy + oy*ux;
            if (Math.abs(perp) > half + 0.20) continue;
            list.push(f);
            mn[at] = Math.min(mn[at], perp); mx2[at] = Math.max(mx2[at], perp);
          }
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
      if (nH < MIN_HEIGHTS) { drop('heights', {n: mem.length, cx: +mx.toFixed(2), cy: +my.toFixed(2), nH}); return []; }
      if (travel < MIN_TRAVEL_RISE * rise || r2 < MIN_R2 || half > MAX_HALF || length > MAX_LEN) {
        // A switchback tower stacks its runs at different heights in the same
        // XY columns, and a U-turn joins two runs side by side at one height.
        // First cut by height: where the tread centre's travel reverses
        // direction the run ends - that boundary is the landing between two
        // flights. Fit each monotone height run on its own.
        if (depth < 3) {
          const cents = [];
          for (let at = 0; at < nK; ++at) {
            if (!alNum[at] || !hMask[at]) continue;
            let cx = 0, cy = 0, n = 0;
            for (const q of mem) {
              if (((q / area) | 0) - kLo !== at) continue;
              const f = q % area, jj = (f / nx) | 0, ii = f - jj*nx;
              cx += lo[0] + (ii+0.5)*res; cy += lo[1] + (jj+0.5)*res; ++n;
            }
            cents.push({at, x: cx/n, y: cy/n});
          }
          const cuts = [0];
          for (let c = 2; c < cents.length; ++c) {
            const d1 = (cents[c-1].x - cents[c-2].x)*ux + (cents[c-1].y - cents[c-2].y)*uy;
            const d2 = (cents[c].x - cents[c-1].x)*ux + (cents[c].y - cents[c-1].y)*uy;
            // a landing joins the next flight sideways: the centre jumps
            // across the run even though the height keeps rising
            const p1 = -(cents[c-1].x - mx)*uy + (cents[c-1].y - my)*ux;
            const p2 = -(cents[c].x - mx)*uy + (cents[c].y - my)*ux;
            if (d1 * d2 < -1e-6 || Math.abs(p2 - p1) > 3*res) cuts.push(c - 1);
          }
          cuts.push(cents.length);
          if (cuts.length > 2) {
            const out = [];
            for (let s = 0; s + 1 < cuts.length; ++s) {
              const a0 = cents[cuts[s]].at, a1 = cents[cuts[s+1] - 1].at;
              const part = mem.filter(q => {
                const at = ((q / area) | 0) - kLo;
                return at >= a0 && at <= a1;
              });
              if (part.length >= MIN_CELLS) out.push(...fit(part, depth + 1));
            }
            if (out.length) return out;
          }
          // One height band can still hold a U: the two runs lie side by
          // side, so the minor axis separates them.
          const a = [], b = [];
          for (const q of mem) {
            const f = q % area, jj = (f / nx) | 0, ii = f - jj*nx;
            const ox = lo[0] + (ii+0.5)*res - mx, oy = lo[1] + (jj+0.5)*res - my;
            (-ox*uy + oy*ux >= 0 ? a : b).push(q);
          }
          if (a.length >= MIN_CELLS && b.length >= MIN_CELLS) {
            const out = fit(a, depth + 1).concat(fit(b, depth + 1));
            if (out.length) return out;
          }
        }
        drop(half > MAX_HALF ? 'half' : length > MAX_LEN ? 'length' : 'straightness',
             {n: mem.length, cx: +mx.toFixed(2), cy: +my.toFixed(2),
              half: +half.toFixed(2), len: +length.toFixed(2), travel: +travel.toFixed(2), r2: +r2.toFixed(2), rise: +rise.toFixed(2)});
        return [];
      }

      // Per-height averages zig-zag where a riser edge shares a step's height;
      // a flight's centre is linear in height, so keep the fit, clamped to the
      // run so the first and last step do not overshoot.
      const slopeA = cov / vA, intercept = (sL - slopeA * sA) / nH;
      for (let at = 0; at < nK; ++at)
        along[at] = Math.max(-length/2, Math.min(length/2, intercept + slopeA * at));

      // Claim the tread, one quad per column at each height the column carries
      // inside the run: a column listed at every height near it would stack
      // into a curtain, so only its own surfaces are claimed.
      const claimWith = (cmx, cmy, cux, cuy, cLen, cHalf, cMn, cMx2, cReg) => {
        const out = [];
        const cex = Math.abs(cux)*(cLen/2 + 0.30) + Math.abs(cuy)*(cHalf + 0.30);
        const cey = Math.abs(cuy)*(cLen/2 + 0.30) + Math.abs(cux)*(cHalf + 0.30);
        const ci0 = Math.max(0, Math.floor((cmx - lo[0] - cex) / res));
        const ci1 = Math.min(nx - 1, Math.ceil((cmx - lo[0] + cex) / res));
        const cj0 = Math.max(0, Math.floor((cmy - lo[1] - cey) / res));
        const cj1 = Math.min(ny - 1, Math.ceil((cmy - lo[1] + cey) / res));
        for (let j = cj0; j <= cj1; ++j)
          for (let i = ci0; i <= ci1; ++i) {
            const f = i + j*nx, list = ks[f];
            if (!list) continue;
            for (const k of list) {
              if (k < kLo || k > kHi) continue;
              if (!hMask[k - kLo]) continue;   // bleed strips stay unclaimed
              if (!walkable(i, j, k)) continue;
              const ox = lo[0] + (i+0.5)*res - cmx, oy = lo[1] + (j+0.5)*res - cmy;
              const at = k - kLo;
              if (Math.abs(ox*cux + oy*cuy - cReg(at)) > 0.45) continue;
              // only heights where the band scan found the tread's own
              // section: the full-width fallback let the floor and landing
              // plates, a step off the tread line, be claimed as tread
              if (!(cMn[at] <= cMx2[at])) continue;
              const perp = -ox*cuy + oy*cux;
              if (perp < cMn[at] - 0.20 || perp > cMx2[at] + 0.20) continue;
              let same = 0;
              for (const [di, dj] of [[1,0],[-1,0],[0,1],[0,-1]]) {
                const i2 = i + di, j2 = j + dj;
                if (i2 < 0 || j2 < 0 || i2 >= nx || j2 >= ny) continue;
                if (near(i2 + j2*nx, k, 1)) ++same;
              }
              // a lone top - a handrail post, a stray box - is not tread
              if (same === 0) continue;
              out.push(f + k*area);
            }
          }
        return out;
      };
      let claimed = claimWith(mx, my, ux, uy, length, half, mn, mx2, regAt);
      if (claimed.length < MIN_COVER * bandCells) {
        drop('cover', {n: mem.length, cx: +mx.toFixed(2), cy: +my.toFixed(2),
                       claimed: claimed.length, band: bandCells});
        return [];
      }

      // The first frame is fitted on every stepped node, bleed rows included,
      // and its centre line can sit off the tread line enough to clip the end
      // steps out of the claim window. The claimed cells are the clean tread:
      // re-frame on them and claim once more, keeping the wider result.
      {
        const cc = [];
        let cmx = 0, cmy = 0;
        for (const q of claimed) {
          const f = q % area, jj = (f / nx) | 0, ii = f - jj*nx;
          const x = lo[0] + (ii+0.5)*res, y = lo[1] + (jj+0.5)*res;
          cc.push([x, y]); cmx += x; cmy += y;
        }
        cmx /= cc.length; cmy /= cc.length;
        const cax = axisOf(cc);
        // per-height along sums on the clean frame, centred on the claimed cells
        const cAlSum = new Float64Array(nK), cAlNum = new Int32Array(nK);
        for (const q of claimed) {
          const f = q % area, kk = (q / area) | 0, at = kk - kLo;
          if (at < 0 || at >= nK || !hMask[at]) continue;
          const jj = (f / nx) | 0, ii = f - jj*nx;
          const ox = lo[0] + (ii+0.5)*res - cmx, oy = lo[1] + (jj+0.5)*res - cmy;
          cAlSum[at] += ox*cax[0] + oy*cax[1]; cAlNum[at] += 1;
        }
        let cnH = 0, csA = 0, csL = 0, csAA = 0, csLL = 0, csAL = 0;
        for (let at = 0; at < nK; ++at) {
          if (!cAlNum[at]) continue;
          const L = cAlSum[at] / cAlNum[at];
          ++cnH; csA += at; csL += L; csAA += at*at; csLL += L*L; csAL += at*L;
        }
        if (cnH >= MIN_HEIGHTS) {
          const cvA = csAA - csA*csA/cnH, cvL = csLL - csL*csL/cnH, ccov = csAL - csA*csL/cnH;
          const cr2 = (cvA > 1e-9 && cvL > 1e-9) ? (ccov*ccov)/(cvA*cvL) : 0;
          const cTravel = cnH > 1 ? Math.abs(ccov)/Math.max(1e-9,cvA)*(cnH-1) : 0;
          if (cTravel >= MIN_TRAVEL_RISE*rise && cr2 >= MIN_R2) {
            // The claim window must reach the end steps the first frame
            // clipped: extrapolate the clean centre line over the whole
            // occupied height range, not just the claimed span.
            const cSlope = ccov / cvA, cInt = (csL - cSlope*csA) / cnH;
            // The window reaches three cells past the claimed heights - far
            // enough to recover the end steps the first frame clipped, near
            // enough that the line does not land on the floor or the next
            // landing (a step away from the tread line) and claim that.
            let a0 = -1, a1 = -1;
            for (let at = 0; at < nK; ++at)
              if (cAlNum[at]) { if (a0 < 0) a0 = at; a1 = at; }
            a0 = Math.max(0, a0 - 3); a1 = Math.min(nK - 1, a1 + 3);
            const pLo = cInt + cSlope*a0, pHi = cInt + cSlope*a1;
            let cAlongMin = 1e9, cAlongMax = -1e9;
            for (const p of cc) {
              const al = (p[0]-cmx)*cax[0] + (p[1]-cmy)*cax[1];
              cAlongMin = Math.min(cAlongMin, al); cAlongMax = Math.max(cAlongMax, al);
            }
            const aLo = Math.min(pLo, pHi, cAlongMin) - 0.30;
            const aHi = Math.max(pLo, pHi, cAlongMax) + 0.30;
            // recentre on the window midpoint so the frame stays symmetric
            const mid = (aLo + aHi) / 2;
            cmx += cax[0]*mid; cmy += cax[1]*mid;
            const cLen = Math.max(aHi - aLo, res);
            // section scan on the clean frame
            const cMn = new Float32Array(nK).fill(Infinity);
            const cMx2 = new Float32Array(nK).fill(-Infinity);
            const cbex = Math.abs(cax[0])*(cLen/2 + 0.30) + Math.abs(cax[1])*(half + 0.30);
            const cbey = Math.abs(cax[1])*(cLen/2 + 0.30) + Math.abs(cax[0])*(half + 0.30);
            const cbi0 = Math.max(0, Math.floor((cmx - lo[0] - cbex) / res));
            const cbi1 = Math.min(nx - 1, Math.ceil((cmx - lo[0] + cbex) / res));
            const cbj0 = Math.max(0, Math.floor((cmy - lo[1] - cbey) / res));
            const cbj1 = Math.min(ny - 1, Math.ceil((cmy - lo[1] + cbey) / res));
            for (let k = kLo; k <= kHi; ++k) {
              const at = k - kLo;
              if (!hMask[at]) continue;
              for (let j = cbj0; j <= cbj1; ++j)
                for (let i = cbi0; i <= cbi1; ++i) {
                  const f = i + j*nx;
                  if (!near(f, k, 0) || !walkable(i, j, k)) continue;
                  const ox = lo[0] + (i+0.5)*res - cmx, oy = lo[1] + (j+0.5)*res - cmy;
                  if (Math.abs(ox*cax[0] + oy*cax[1] - (cInt + cSlope*at - mid)) > 0.45) continue;
                  const perp = -ox*cax[1] + oy*cax[0];
                  if (Math.abs(perp) > half + 0.20) continue;
                  cMn[at] = Math.min(cMn[at], perp); cMx2[at] = Math.max(cMx2[at], perp);
                }
            }
            const re = claimWith(cmx, cmy, cax[0], cax[1], cLen, half, cMn, cMx2,
                                 at => cInt + cSlope*at - mid);
            if (re.length > claimed.length) {
              claimed = re;
              mx = cmx; my = cmy; ux = cax[0]; uy = cax[1]; length = cLen;
              mn.set(cMn); mx2.set(cMx2);
              for (let at = 0; at < nK; ++at) {
                along[at] = Math.max(-length/2, Math.min(length/2, cInt + cSlope*at - mid));
                if (cMn[at] <= cMx2[at]) {
                  spine[at] = (cMn[at] + cMx2[at]) / 2;
                  halfK[at] = Math.max((cMx2[at] - cMn[at]) / 2, 0.10);
                }
              }
            }
          }
        }
      }

      return [{
        kind: 'stairs',
        centre: [mx, my], axis: [ux, uy],
        half, length, rise,
        z: [lo[2] + rkMin*res - 0.15, lo[2] + rkMax*res + 0.35],
        cells: claimed,
        steps: {kLo, kHi, spine, half: halfK, along},
        metrics: {columns: mem.length, cells: claimed.length, heights: nH,
                  travel: +travel.toFixed(2), r2: +r2.toFixed(3),
                  slope: +slope.toFixed(2)},
      }];
    }

    for (let j0 = 0; j0 < ny; ++j0)
      for (let i0 = 0; i0 < nx; ++i0) {
        const f0 = i0 + j0*nx, list0 = ks[f0];
        if (!list0) continue;
        for (const k0 of list0) {
          const id0 = f0 + k0*area;
          if (!stepped[id0] || seen[id0]) continue;
          stack.length = 0; stack.push(id0); seen[id0] = 1;
          const mem = [];
          while (stack.length) {
            const q = stack.pop(); mem.push(q);
            const f = q % area, k = (q / area) | 0;
            const qj = (f / nx) | 0, qi = f - qj*nx;
            for (const [di, dj] of [[1,0],[-1,0],[0,1],[0,-1]]) {
              // a riser line can raster as one or two columns with no tread of
              // their own; bridge them so a flight stays one cluster
              for (const dist of [1, 2, 3]) {
                const i2 = qi + di*dist, j2 = qj + dj*dist;
                if (i2 < 0 || j2 < 0 || i2 >= nx || j2 >= ny) continue;
                const f2 = i2 + j2*nx, list2 = ks[f2];
                if (!list2) continue;
                let joined = 0;
                for (const k2 of list2) {
                  if (Math.abs(k2 - k) > maxStep) continue;
                  const nq = f2 + k2*area;
                  if (!stepped[nq] || seen[nq]) continue;
                  seen[nq] = 1; stack.push(nq); joined = 1;
                }
                if (joined) break;
              }
            }
          }
          for (const r of fit(mem, 0)) regions.push(r);
        }
      }
    return regions;
  }

  window.TerrainSemantics.register({
    id: 'stairs',
    title: '楼梯段',
    version: 5,
    detects: 'walkable stair flights with per-step tread centres',
    detect,
  });
})();