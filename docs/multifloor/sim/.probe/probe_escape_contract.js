// Escape route-rotation contract, JS side.
//
// Slices the pure functions between ESC_CONTRACT_BEGIN / ESC_CONTRACT_END out
// of mesh_nav.template.html (the same technique probe_plan.js uses for the
// planner) and runs escape_contract.fixture.json through them. The C++ twin is
// ros_ws/src/mission_manager/test/test_escape_route_gate.cpp on the same file:
// if the two implementations drift, one of them stops matching the fixture.
//
//   node .probe/probe_escape_contract.js          # template (source of truth)
//   node .probe/probe_escape_contract.js --built  # generated mesh_nav.html
//
// --built additionally checks that build.py injected the profile it claims to,
// so "the sim reads the robot's config" is verified rather than assumed.
const fs = require('fs');
const path = require('path');
const base = path.resolve(__dirname, '..');
const built = process.argv.indexOf('--built') >= 0;
const src = fs.readFileSync(path.join(base, built ? 'mesh_nav.html' : 'mesh_nav.template.html'), 'utf8');
const fix = JSON.parse(fs.readFileSync(path.join(base, 'escape_contract.fixture.json'), 'utf8'));

const cut = (a, b) => {
  const i = src.indexOf(a), j = src.indexOf(b, i);
  if (i < 0 || j < 0) throw new Error('slice not found: ' + a);
  return src.slice(i, j);
};
// The block minus the ESCP definition: the probe feeds its own params in, so a
// case can override one key the way a profile would.
const block = cut('// ESC_CONTRACT_BEGIN', '// ESC_CONTRACT_END');
const after = block.split('const ESCP = Object.assign({')[1];
if (!after) throw new Error('const ESCP = Object.assign({ not found in the contract block');
const body = after.slice(0, after.indexOf('}, (typeof window'));   // the default literals
const fns = after.slice(after.indexOf('\nfunction '));             // every pure function
global.window = {};
const api = (0, eval)(
  'var ESCP_DEFAULTS = {' + body + '};\n' +
  // escTrace reads the module-level ESCP the page defines; give the slice one so
  // it stays self-contained even though the probe never calls it.
  'var ESCP = Object.assign({}, ESCP_DEFAULTS);\n' +
  fns +
  '\n({ESCP_DEFAULTS: ESCP_DEFAULTS, escRouteSignature: escRouteSignature, escRound: escRound,' +
  ' escAcceptCandidate: escAcceptCandidate, escPickCandidate: escPickCandidate,' +
  ' escTerrainGate: escTerrainGate, escDwellMs: escDwellMs})');

let pass = 0, fail = 0;
const ok = (cond, what) => {
  if (cond) { ++pass; } else { ++fail; console.log('FAIL ' + what); }
};
const paramsOf = (over) => Object.assign({}, api.ESCP_DEFAULTS, over || {});

// 1. defaults: the contract block and the fixture must agree key by key, or a
//    new parameter can be honoured by one runtime and silently missed by the
//    other (and by the four robots/<id>/neupan/config.yaml files).
const dflt = fix.defaults;
ok(Object.keys(api.ESCP_DEFAULTS).length === Object.keys(dflt).length,
   'default key count: sim ' + Object.keys(api.ESCP_DEFAULTS).length +
   ' vs fixture ' + Object.keys(dflt).length);
for (const k of Object.keys(dflt)) {
  const a = JSON.stringify(api.ESCP_DEFAULTS[k]), b = JSON.stringify(dflt[k]);
  ok(a === b, 'default ' + k + ': sim ' + a + ' vs fixture ' + b);
}
if (built) {
  const inj = /window\.__ESCAPE_CFG = (\{.*?\});/.exec(src);
  ok(!!inj, 'built page has no window.__ESCAPE_CFG injection');
  if (inj) {
    const cfg = JSON.parse(inj[1]);
    for (const k of Object.keys(cfg))
      ok(JSON.stringify(cfg[k]) === JSON.stringify(dflt[k]),
         'injected profile value ' + k + ' = ' + cfg[k] + ' differs from the shipped default ' + dflt[k]);
  }
}

// 2. signatures
for (const s of fix.signatures) {
  const got = api.escRouteSignature(s.kind, s.pts, s.grid_m);
  ok(got === s.expect_sig, 'sig "' + s.name + '": got ' + got + ' want ' + s.expect_sig);
}

// 3. cases
for (const c of fix.cases) {
  const p = paramsOf(c.params);
  const round = c.attempt !== undefined ? api.escRound(c.attempt, p) : c.round;
  if (c.expect.round !== undefined)
    ok(round === c.expect.round, c.name + ': round ' + round + ' want ' + c.expect.round);
  const action = api.escTerrainGate(!!c.on_flight, !!c.at_landing, !!c.reversal, p);
  ok(action === c.expect.action, c.name + ': action ' + action + ' want ' + c.expect.action);
  if (c.expect.dwell_ms !== undefined) {
    const ms = api.escDwellMs(!!c.floor_transition, p);
    ok(ms === c.expect.dwell_ms, c.name + ': dwell ' + ms + ' want ' + c.expect.dwell_ms);
  }
  // Episode-level verdict: a hold or a retreat never reaches the picker. The
  // terrain cases still carry candidates on purpose - they prove it is the gate
  // that stops the install, not the acceptance math happening to refuse.
  let pick = {index: -1, fallback: false};
  if (action === 'allow') {
    const cands = (c.candidates || []).map((x) => ({
      pts: x.pts, clearance: x.clearance, length: x.length, kind: x.kind,
      terrainPenalty: x.terrain_penalty || 0,
    }));
    pick = api.escPickCandidate(cands, c.current, round, c.tried || [], p);
  }
  const want = c.expect.pick_index === 'none' ? -1 : c.expect.pick_index;
  ok(pick.index === want, c.name + ': pick ' + pick.index + ' want ' + want);
  ok(!!pick.fallback === !!c.expect.fallback, c.name + ': fallback ' + pick.fallback +
     ' want ' + c.expect.fallback);
}

console.log((fail ? 'FAILED ' : 'ok ') + pass + ' passed, ' + fail + ' failed  [' +
            (built ? 'mesh_nav.html' : 'template') + ', ' + fix.cases.length + ' cases, ' +
            fix.signatures.length + ' signatures]');
process.exit(fail ? 1 : 0);
