# Stuck-Escape Exploration — two layers, one contract

This branch exists to make the *route-exploration* half of the stuck-escape
reach the same sim ↔ robot parity the *local-maneuver* half already had, and to
make the escape **terrain aware** (it must not swap the route out from under a
dog that is mid-way up a staircase). `NeuPAN/neupan/escape_policies/stuck_escape.py`
records why this matters: before it, the benchmark carried a hand-written
simplification "so simulation results about escape tuning did not transfer to
the robot at all". The plan here is that the *other* half obeys the same rule.

The escape is deliberately split into two layers that live in two places:

| | local maneuver | route exploration |
|---|---|---|
| question | "I am stuck right now — back up and turn" | "this corridor is a dead end — pick a different way around" |
| robot | `NeuPAN/neupan/escape_policies/stuck_escape.py` (`StuckEscapePolicy`) | `ros_ws/src/mission_manager` escape episode |
| simulator | the tick's `state.blocked` / `state.backOut` | `replanGlobal()` best-of-N + per-episode rotation |
| decision library | (in the policy) | `mission_manager/include/mission_manager/escape_route_gate.h` |
| trigger | dwell counting (`trigger_dwell_ms`) | a `StuckEscapePolicy.request_replan` / `/neupan/replan` |

`escape_route_gate.h` is header-only and **ROS-free**, shared byte-for-byte by
the `ros_ws` and `ros2_ws` trees. Every decision the robot makes about *which*
route to try is a call into it, so there is exactly one implementation of the
acceptance, ranking, rotation and terrain math.

## How parity is proven, not asserted

A single table-driven fixture drives the simulator probe and the C++ test:

- `docs/multifloor/sim/escape_contract.fixture.json`
- `docs/multifloor/sim/.probe/probe_escape_contract.js` (node, slices the JS
  decision functions straight out of `mesh_nav.template.html`)
- `ros_ws/src/mission_manager/test/test_escape_route_gate.cpp` (and its
  `ros2_ws` mirror) runs the *same* file through the header.

If the two runtimes disagree on any case, one of the two probes fails. That is
the parity gate. Change a rule and you must change it on both sides, then re-run
both.

Terrain behaviour is checked separately by
`docs/multifloor/sim/.probe/probe_escape_terrain.py` (4 cases: mid-flight hold,
one-tread-from-portal install, retreat-to-landing target, and the floor-handover
dwell sentinel).

## Episode state machine (route layer)

The route layer is memory of one stuck spell. It is `EscapeEpisode` on the
robot, `state.escape {try, sigs, at, ep}` in the simulator, and the shape is the
same:

```
        (new goal / arrived / cancel / reset)
                        │  resetEpisode()
                        ▼
   idle ──request──► evaluate ──terrainGate()──► hold / retreat   (mid-flight:
  (round 0)            │   ▲                         (no route swap)  keep the
                       │   └── episodeMaybeReset ──► (rolled >1.2 m    route,
                       ▼                              or stale >20 s)   finish
                   pick / accept ──► install through the ordinary       to a
                       │              replan gate, noteEpisodeInstall    landing)
                       ▼
                   tried_sigs grows; round advances (saturates at max_rounds)
```

- **Round 0** installs the argmax route, exactly as before this branch.
- **Round ≥ 1** deliberately spends its turn on the route the scorer ranked
  second, third, …, skipping signatures already tried — the ranking is only a
  proxy, and re-installing the same top pick that just re-blocked is the
  livelock this layer exists to break.
- The episode resets (back to a fresh argmax) once the dog is genuinely rolling
  again or the spell goes stale, so an unrelated later stuck spell is not
  penalised by old memory.
- **The trigger is progress-anchored, not blocked-flag-edge-triggered.** The stall
  timer is only reset when the robot actually gains ground past where the block
  began (`+0.5 m` of route in the simulator, a tick-local constant), not on the
  first frame the blocked verdict happens to clear. A dog shuffling against a post
  flips `state.blocked` every couple of seconds without moving; if each flip wiped
  the clock the escape would never fire on a genuinely stuck-but-oscillating robot.

## The four terrain rules

Keyed off `stair_commit_enable`; with it off the escape is terrain-blind (the
pre-branch behaviour). `terrainGate()` returns `allow | hold | retreat_to_landing`.

1. **Mid-flight: no route swap.** While the robot is between a flight's portals
   and not within `landing_snap_radius_m` of one, the escape does not generate
   or install a route — the dog finishes to the nearest landing. On the robot
   `runEscapeEpisode()` returns having *consumed* the replan request without
   changing the path.
2. **Retreat aims at the landing.** When a reversal is genuinely required
   mid-flight, the target is the nearest portal cell, not "back along the path"
   (an unrecognised tread edge is not a standable surface).
3. **Committed-flight candidates are rejected, not de-ranked.** A candidate that
   re-enters the flight the episode is already committed to gets
   `terrain_penalty > 0` and is filtered out *before* acceptance scoring.
4. **A floor handover is not a block.** The longer `transition_dwell_ms` (2× the
   ordinary `trigger_dwell_ms`) is keyed off the live stair-on signal
   (`state.onStairNow`) — the dog is between a flight's portals or in the gait
   handover — so a slow climb is not mistaken for a stuck dog, while a block on
   open floor keeps the ordinary `trigger_dwell_ms`. Rotation never drops the
   centre-line stair leg. (It is deliberately *not* keyed off the flat-floor crawl
   window `floorPass`: that is true on open floor and would wrongly lengthen an
   ordinary block there.)

On the robot side the local policy has a matching opt-in gate
(`stuck_escape.py`: `note_terrain()` + `stair_suppress_enable`, default **false**,
fed from `/stair_gait/status`) so a back-up-plus-rotate does not start mid-flight.

## The detour is ephemeral — the plan is not

The route an escape installs is a **tactical, single-pass** maneuver, not a
revision of the strategic mission. The obstacle that stopped the dog is very often
*temporary* — a person who walked past, a chair that was moved, a door that was
shut and will open — so baking the long way around into the global plan would
leave the robot detouring for a hazard that is already gone.

The simulator models this with two polylines:

- `state.path` — the route the dog is actually riding. An escape installs its
  detour here and here only, via `installPath(best.pts, /*ephemeral*/ true)`.
- `state.basePath` — the **committed mission**, written when the user clicks
  「规划」 (or a hash-boot plans a demo) and never touched by `replanGlobal`.

When a looping pass reaches the goal the tick restores `state.basePath` as the
active route and clears the episode memory, so the next pass runs the pristine
mission from its true start again: it re-escapes if the block is still there, and
simply walks the direct line if the block has cleared. This mirrors the robot,
where there is no loop but `mission_manager` keeps re-planning from the static
prior — a tactical detour never persists into the strategic route there either.

## The attempt pool — how candidates are generated

`replanGlobal()` does not ask the planner for a single alternate. It builds a
**best-of-N pool** of genuinely different routes, scores each with `routeSafety`
(the tighter of physical-OCC clearance and dynamic-post clearance ahead, read
the way the local layer reads them), and installs the safest one that clears the
acceptance gate. Pool sources, cheapest first:

| kind (`cls`) | how it is generated | planned on → scored on | origin |
|---|---|---|---|
| `绕开立柱` | re-plan the stops with the dynamic posts merged into the search (`globalMergePosts`) | live → live | core |
| `贴墙绕行` | re-plan on the banded sibling `state._bandMap` so a wall-hugging route is pulled off the wall | band → band | core |
| `全局×N` | **inflation fan-out** — one plan per `candidate_inflation_scales` entry above 1.0, built lazily by `ensureInflMaps()` | wide band → **live** | sim-only |
| `绕行试探` | detour via-points from `detourSeeds()`, planned on the band | band → band | core |
| `绕行·现场` | the same via-points re-planned over the **live** map (round ≥ 1) — two more genuinely new routes | live → live | sim-only |
| `绕行·随机` | **randomised via-points** spread along the look-ahead window, re-rolled every burst | live → live | sim-only |

### `detourSeeds()` — the via-point generator

It locates the tightest point `c` on the next `detour_lookahead` metres of the
route; if `escDetourWanted(worst, round)` judges the corridor genuinely tight it
emits free cells around it, escalating ambition with the round number:

1. **Perpendicular berths** (round 0+): free cells at ±`detour_berths_m` (widened
   to `detour_berths_widened_m` on later rounds) across the local tangent.
2. **Angled swings** (round 2+): free cells rotated ±`detour_swing_rad` off the
   tangent at `detour_swing_range_m`. From the third attempt on these **reserve**
   `2 × len(detour_swing_rad)` of the `detour_max_seeds` slots so the perpendicular
   berths can no longer claim every slot and starve the swings out. The reservation
   is mirrored byte-for-byte in the C++ `detourSeedPoints()`.
3. **Randomised tail** (round 1+, sim only): a handful of via-points at a random
   arc position across the whole window, offset to a random side by a random
   1.2–2.4 m range, re-rolled by `Math.random` on every call. This is the
   local-optimum breaker — the grid above is deterministic and clustered on the
   single pinch `c`, so without it every throttled burst of one stuck spell
   replans the *identical* pool. `detourSeeds()` returns `{seeds, nGrid}` so
   `replanGlobal()` plans the deterministic head on band+live but the random tail
   on the live map only (a wide loop reads worse on the fatter band, and planning
   each twice doubled the per-burst A\* cost). The C++ side stays deterministic:
   a robot episode must be reproducible for field debugging.

> Randomness and fan-out widen the *attempt set*; they cannot manufacture
> clearance the map does not contain. See the caveat under
> [What is still sim-only](#what-is-still-sim-only-honest-list).

## Parameters

`stuck_escape.route_rotation` in `robots/<id>/neupan/config.yaml` (the simulator
reads the same block through `build.py --profile`; `mission_manager` reads it
from its own `route_rotation/*` params, opt-in via `escape_route_enable`).

| key | default | meaning |
|---|---|---|
| `enable` | true | explore alternative routes during a stuck spell |
| `candidate_inflation_scales` | `[1.0, 1.5, 2.0]` | one global plan per inflation multiplier; the sim re-plans the same stops on wider-band sibling maps from the second attempt on (sim-only fan-out, see below) |
| `detour_berths_m` | `[0.5, 0.95]` | lateral bypass offsets, first attempt |
| `detour_berths_widened_m` | `[0.5, 0.95, 1.45]` | ... from the second attempt on |
| `detour_lookahead_m` / `_widened_m` | 3.0 / 6.0 | forward distance searched for the tightest spot |
| `detour_max_seeds` | 6 | cap on bypass via-points per round; from the third attempt on the angled swings reserve `2 × len(detour_swing_rad)` of these slots, so the perpendicular berths cannot starve them out |
| `detour_swing_rad` | `[-0.6, 0.6]` | angled swings off the perpendicular, third attempt on (each angle × each side gets a reserved seed slot) |
| `detour_swing_range_m` | 1.3 | ... at this range |
| `install_gain_m` | 0.05 | clearance gain a candidate needs to replace the route (also the ranking band) |
| `rescue_clearance_m` | 0.225 | body half-width + 5 cm; below this the route wedges the robot |
| `min_candidate_clearance_m` | 0.175 | body half-width; exploration never installs below this |
| `max_rounds` | 3 | attempt index saturates here |
| `route_sig_grid_m` | 0.5 | quantisation of the "already tried" route signature |
| `max_tried_sigs` | 8 | route signatures remembered per episode |
| `episode_reset_progress_m` | 1.2 | rolling again by this much ends the spell |
| `episode_reset_timeout_sec` | 20 | ... or this long since the last install |
| `stair_commit_enable` | true | terrain rules on (false = terrain-blind escape) |
| `escape_only_at_landing` | true | no route swap mid-flight, only on a landing |
| `landing_snap_radius_m` | 0.45 | distance to a portal that counts as "on the landing" |
| `trigger_dwell_ms` | 3000 | stalled before the escape fires |
| `transition_dwell_ms` | 6000 | ... across a floor transition |
| `replan_throttle_ms` | 1200 | minimum gap between escape re-plan bursts |
| `trace_enable` | true | emit one compact JSON line per escape decision |

The defaults are the constants the simulator used before they were configurable,
so a rebuilt page is behaviour-identical until a profile changes them.

## Sim → real symbol map

| simulator (`mesh_nav.template.html`) | production |
|---|---|
| `replanGlobal()` (escape block) | `MissionManager::runEscapeEpisode()` |
| `routeSafety()` / `staticClearanceAhead()` | `ValidatePath`-style clearance; scan corridor via `escapeCorridorClearance()` (`corridorBlocked`) |
| `sigOf()` | `routeSignature()` |
| `cands.filter(...)` acceptance | `acceptCandidate()` |
| sort + rotation walk | `pickCandidate()` |
| `detourSeeds()` | `detourSeedPoints()` |
| `state._bandMap` (banded sibling) | inflation-scale candidates (see sim-only) |
| tick stair rules / `state.onStairNow` | `terrainGate()` fed from `/stair_gait/status` |
| `escDwellMs()` | `escapeDwellMs()` |
| `state.basePath` + `installPath(pts, ephemeral)` | the static-prior global route `mission_manager` re-plans from; escape installs are not committed to it |
| `state.escape {try, sigs, at}` | `EscapeEpisode` |
| `window.__ESCAPE_TRACE` (array) | `/neupan/mission_escape_trace` (JSON line, `formatEscapeTrace`) |

Both telemetry paths emit the same record shape —
`{stamp, episode_id, round, kind, clearance, current, sig, action, on_flight}` —
which is what makes a simulator tuning claim transferable to the robot.

## What is still sim-only (honest list)

- **Multi-candidate inflation fan-out (implemented in the sim only).** From the
  second attempt on, `replanGlobal()` re-plans the same stops once per
  `candidate_inflation_scales` entry above 1.0 (via `ensureInflMaps()`), each on a
  sibling map with a wider inflation band but scored against the *live* map, so
  the clearance numbers stay honest. In tight rooms these wider-band routes often
  score *lower* than the nominal plan (a bigger keep-out band shoves A\* into
  harder physical-OCC pinches), so the fan-out widens the attempt set without
  changing which candidate wins — it is a coverage improvement, not a clearance
  boost. The robot side cannot mirror it: `ros_ws`/`ros2_ws`
  `nav_interfaces/PlanPath.srv` carries only `{goal, shadow_only}` — there is
  **no `inflation_radius` field** — and the plan forbids adding new service
  fields, so the robot escape re-plans the single nominal route to the goal.
  Acceptance, rotation, terrain gating, install-gating and telemetry all still
  run; only the *width* of the candidate set differs. A detour via-point could be
  issued as a temporary single-waypoint goal, but that two-phase maneuver is not
  hardware-validated and is left disabled.
- **Randomised detour seeds — local-optimum breakers (sim only).** The
  berth/swing grid in `detourSeeds()` is deterministic and clustered on the
  single tightest spot ahead, so a stalled episode re-plans the *same* pool every
  throttled burst and can sit in a local optimum of the **candidate space**: a
  genuinely safer route that dodges the pinch region off to one side stays hidden
  because no grid point samples the corridor that reaches it. From the second
  attempt on, `detourSeeds()` also returns a small tail of randomised via-points
  (`绕行·随机`): an arc position drawn across the whole look-ahead window, offset
  to a random side by a random 1.2–2.4 m range, re-rolled via `Math.random` on
  each call, so the ~10 bursts of one stuck spell sweep the reachable open space
  the grid misses. They are planned on the **live map only** (a wide loop reads
  worse on the banded sibling, and planning each twice doubled the per-burst A\*
  cost for no acceptance benefit). `freeCellNear` drops the ones that land in a
  wall, `planStops` drops the unreachable ones, and the acceptance gate still has
  to clear the body budget, so this only ever *adds* attempts — never removes a
  good candidate or lowers the bar. `detourSeedPoints()` (C++) stays
  deterministic: a robot episode must be reproducible for field debugging.
  **Caveat — randomness cannot manufacture clearance.** An exhaustive via-space
  sweep of the `示例 单层阶梯` start→first-waypoint corridor (the reported stuck
  case) found *no* route above the 0.175 m body budget — its global maximum is
  ~0.15 m, capped by a hard downstream bottleneck that `routeSafety` scores as
  the min over the whole remaining chain, so every candidate ties. There the
  escape correctly declines to install (an equal-or-worse swap), and neither the
  fan-out nor the randomised seeds change that; the lever for such a segment is
  the acceptance parameters / scoring horizon, not more exploration.
- **`ValidatePath` clearance ranking.** ROS1 has no `ValidatePath` service and
  ROS2's returns no clearance metric, so the robot ranks candidates by the scan
  corridor clearance when a fresh 360 scan is available, else by polyline length.
  The simulator's `routeSafety` proxy is geometric and richer.
- **Terrain suppression on the robot is ROS1-only.** `/stair_gait/status` is a
  ROS1 topic; the `ros2_ws` escape runs terrain-blind (the multi-floor/stair
  scope of this branch is ROS1/Go2).
- **The local maneuver is not simulated in full** (corridor / limit-cycle /
  rotation-gate dynamics); only the escape's dwell and terrain verdict are.
- **The ephemeral-detour / committed-plan split is driven by the loop.** The
  simulator restores `state.basePath` when a pass ends — its only signal that "the
  hazard may be gone". The robot reaches the same effect through
  `mission_manager`'s periodic re-plan off the static prior rather than a loop
  boundary.

The fixture and the trace shape are designed so each of these can be lifted to
the robot later without renaming anything.

## Enabling it on the robot

`escape_route_enable` (mission_manager param) defaults **false**, so existing
deployments are byte-identical until it is turned on. To enable: set
`escape_route_enable:=true` and map the `route_rotation/*` keys (from the robot's
`neupan/config.yaml`) into the mission_manager parameter namespace. Recommended
together with `install_gate_enable`/`install_gate_shadow` so the escape installs
through the same safety gate and the corridor-clearance ranking has data.
