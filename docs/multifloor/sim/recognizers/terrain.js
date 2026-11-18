// Terrain semantics: a registry the page and the exporter both talk to.
//
// The mesh page used to know exactly one kind of terrain - a stair - and the
// fit lived inside buildMap, mixed with the rasterisation. That makes a second
// kind (a ramp, a door threshold, a landing edge) a rewrite of buildMap, and
// makes the fit untestable on its own. Here the grid is handed over once and
// every recogniser is an independent function:
//
//   TerrainSemantics.register({
//     id: 'stairs',                     // stable, used in exports
//     title: '楼梯段',                  // shown in the UI
//     version: 2,
//     detects: 'walkable stair flights with per-step tread centres',
//     detect(grid, opts) -> region[]   // or [] when it finds nothing
//   });
//
// A region is the unit everything downstream agrees on:
//
//   {
//     kind, id,                        // kind defaults to the recogniser id
//     cells:  [packed (i,j,k)],        // every cell this region claims,
//                                      //   exactly what gets drawn and charged
//     centre: [cx, cy], axis: [ux, uy],
//     half, length, rise,              // metres in the grid's own frame
//     z: [zmin, zmax],
//     steps: {kLo, kHi, spine, half, along},   // optional per-step detail
//     metrics: {...}                   // whatever the recogniser wants to prove
//   }
//
// The grid passed in is read-only and deliberately small:
//   {res, lo, nx, ny, nz, walkable(i,j,k), tread(Int32Array nx*ny), topFace?}
// so a recogniser cannot reach into the page, and can be run from a test.
(function () {
  'use strict';

  const registry = [];

  function register(recogniser) {
    if (!recogniser || !recogniser.id || typeof recogniser.detect !== 'function')
      throw new Error('terrain recogniser needs id + detect');
    const at = registry.findIndex(r => r.id === recogniser.id);
    if (at >= 0) registry[at] = recogniser;      // hot reload / re-register
    else registry.push(recogniser);
  }

  function run(grid, opts) {
    const regions = [];
    const stats = [];
    for (const recogniser of registry) {
      const t0 = performance.now();
      let found = [];
      try {
        found = recogniser.detect(grid, opts || {}) || [];
      } catch (e) {
        // One recogniser failing must not take the map down with it: the rest
        // still describe what they can, and the page shows the failure.
        stats.push({id: recogniser.id, title: recogniser.title, error: String(e && e.message || e)});
        continue;
      }
      for (let i = 0; i < found.length; ++i) {
        if (!found[i].kind) found[i].kind = recogniser.id;
        found[i].id = recogniser.id + '-' + i;
        regions.push(found[i]);
      }
      stats.push({id: recogniser.id, title: recogniser.title, count: found.length,
                  ms: +(performance.now() - t0).toFixed(1)});
    }
    return {regions, stats};
  }

  // The planner's A* was written against one flight shape; keep that conversion
  // here instead of teaching the planner about regions.
  function stairsToFlight(region) {
    return {
      cx: region.centre[0], cy: region.centre[1],
      ux: region.axis[0], uy: region.axis[1],
      half: region.half, len: region.length, rise: region.rise,
      kLo: region.steps.kLo, kHi: region.steps.kHi,
      spine: region.steps.spine, halfK: region.steps.half, alongAt: region.steps.along,
      zmin: region.z[0], zmax: region.z[1],
      weight: 1.0, n: region.cells.length, cells: region.cells,
    };
  }

  window.TerrainSemantics = {register, run, stairsToFlight, kinds: () => registry.map(r => r.id)};
})();
