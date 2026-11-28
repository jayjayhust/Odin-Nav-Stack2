/*
 * Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Pure-logic stuck-escape ROUTE-ROTATION contract (ROS-independent).
//
// The escape has two layers, and this file owns the second one:
//   * local maneuver   -> NeuPAN/neupan/util/stuck_escape.py (back up + turn)
//   * route exploration-> what is declared here (which alternative route to
//                         install when the local maneuver cannot recover)
//
// It is the production counterpart of the simulator escape block in
// docs/multifloor/sim/mesh_nav.template.html, and the mapping is 1:1:
//   routeSignature  <->  sigOf()          (when is a route "already tried")
//   acceptCandidate <->  cands.filter()   (may it replace the stuck route)
//   pickCandidate   <->  sort + rotation  (which candidate, in which order)
//   detourSeedPoints<->  detourSeeds()    (where to force a bypass via-point)
//   terrainGate     <->  the stair rules in replanGlobal()/the tick
//   EscapeEpisode   <->  state.escape {try, sigs, at}
// Parity is not asserted, it is checked: docs/multifloor/sim/
// escape_contract.fixture.json drives BOTH test_escape_route_gate.cpp here and
// docs/multifloor/sim/.probe/probe_escape_contract.js on the simulator side.
// Change a rule here and you must change it there, then re-run both.
//
// Decision pipeline (evaluated in order):
//   1) Terrain gate: while the robot is mid-flight on a recognised stair it
//      must finish onto a landing instead of having its route swapped under it,
//      and a required reversal aims at the nearest portal rather than "back
//      along the path" (an unrecognised tread edge is not a standable surface).
//   2) Acceptance: a candidate replaces the stuck route only if it beats it by
//      install_gain_m, or lifts the body budget up to rescue_clearance_m, or -
//      from round 1 on, when the safety proxy has demonstrably missed - is an
//      equally safe but *different* peer above min_candidate_clearance_m.
//      Candidates with terrain_penalty > 0 are rejected outright, never ranked.
//   3) Rotation: the survivors are ranked by clearance then length and the
//      attempt index walks down that ranking skipping signatures the episode
//      has already installed. The ranking is a proxy (clearance is not the
//      outcome), so a retry must not re-install the same argmax that just
//      re-blocked - that is the livelock this whole layer exists to break.
//
// Note on the candidate point type: the plan sketch used geometry_msgs::Point,
// but this library is deliberately ROS-free (same rule as
// replan_install_gate.h, which it reuses for Pt / pointAtArc / pathLength), so
// candidates carry std::vector<Pt>. Callers convert to/from ROS messages.

#ifndef MISSION_MANAGER_ESCAPE_ROUTE_GATE_H
#define MISSION_MANAGER_ESCAPE_ROUTE_GATE_H

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "mission_manager/replan_install_gate.h"

namespace mission_manager_gate {

// ---------------------------------------------------------------------------
// Parameters (mirrors stuck_escape.route_rotation in robots/<id>/neupan/
// config.yaml one key at a time; defaults are the constants the simulator
// used before they were configurable, so a rebuilt page behaves identically).
// ---------------------------------------------------------------------------
struct EscapeParams {
  bool enable{true};
  // Global-plan candidates: one re-plan per inflation multiplier (1.0 = nominal).
  std::vector<double> candidate_inflation_scales{1.0, 1.5, 2.0};
  // Lateral berths (m) snapped off the tightest spot ahead, round 0 ...
  std::vector<double> detour_berths_m{0.5, 0.95};
  // ... and round >= 1, where the first ranking already proved unreliable.
  std::vector<double> detour_berths_widened_m{0.5, 0.95, 1.45};
  double detour_lookahead_m{3.0};
  double detour_lookahead_widened_m{6.0};
  int detour_max_seeds{6};
  // Angled swings off the corridor tangent, added from round 2 on (rad, m).
  std::vector<double> detour_swing_rad{-0.6, 0.6};
  double detour_swing_range_m{1.3};
  // Minimum clearance gain a candidate needs to replace the active route, and
  // the tolerance equal-safety routes are ranked within.
  double install_gain_m{0.05};
  // Body budget: HALF_W + 0.05 (rescue) and HALF_W (exploration floor).
  double rescue_clearance_m{0.225};
  double min_candidate_clearance_m{0.175};
  int max_rounds{3};
  // routeSignature quantisation (m); signatures are emitted as grid indices.
  double route_sig_grid_m{0.5};
  // Episode reset: the dog rolled (progress) or the spell went stale (time).
  double episode_reset_progress_m{1.2};
  double episode_reset_timeout_sec{20.0};
  int max_tried_sigs{8};
  // Terrain rules (see terrainGate). Off restores the terrain-blind escape.
  bool stair_commit_enable{true};
  bool escape_only_at_landing{true};
  double landing_snap_radius_m{0.45};
  // Trigger timing: dwell before a stall counts as stuck, longer across a
  // floor transition so a slow gait handover is not read as a block, and a
  // throttle between re-plan bursts.
  int trigger_dwell_ms{3000};
  int transition_dwell_ms{6000};
  int replan_throttle_ms{1200};
  bool trace_enable{true};
};

// ---------------------------------------------------------------------------
// Candidates and episode memory
// ---------------------------------------------------------------------------
struct EscapeCandidate {
  std::vector<Pt> pts;
  double clearance{0.0};  // routeSafety / validatePath minimum clearance (m)
  double length{0.0};     // polyline length (m); tie-breaker inside a band
  std::string kind;       // human label, also the signature prefix
  std::string sig;        // empty -> derived from pts/kind by candidateSignature
  int terrain_penalty{0};  // > 0: re-enters a flight already committed to
};

struct EscapeEpisode {
  int round{0};                      // attempt index (monotonic, clamped)
  std::vector<std::string> tried_sigs;
  double installed_at{0.0};          // monotonic seconds of the last install
  double reset_progress_m{0.0};      // path arc gained since that install
};

// Attempt index -> round, clamped so the escalation saturates at max_rounds.
inline int escapeRound(int attempt, const EscapeParams &p) {
  const int m = p.max_rounds > 0 ? p.max_rounds : 0;
  return attempt < 0 ? 0 : (attempt > m ? m : attempt);
}

// ---------------------------------------------------------------------------
// routeSignature: "is this the route we already tried?"
//
// Two samples at 33% and 67% of the arc length, quantised onto the grid and
// emitted as *grid indices* (integers). Indices rather than formatted floats
// are deliberate: printf("%.2f") and Number.toFixed() disagree on exact ties
// (glibc rounds to even, ECMAScript rounds away from zero), so a decimal
// rendering would make the C++/JS parity claim grid-value dependent. With
// integers the two sides agree for any grid.
//
// Consequence: changing route_sig_grid_m invalidates existing signatures. That
// is safe because episode memory lives only as long as one stuck spell.
// ---------------------------------------------------------------------------
inline long long sigBin(double v, double grid_m) {
  const double g = grid_m > 0.0 ? grid_m : 0.5;
  // JS Math.round semantics (floor(x + 0.5)), NOT std::round (half away from
  // zero), which differs for negative half-steps.
  return static_cast<long long>(std::floor(v / g + 0.5));
}

// Polyline is treated as planar for arc-length purposes (the fixture keeps
// every candidate at z = 0 so the 2D gate maths and the simulator's 3D
// arc-length sampling agree).
inline std::string routeSignature(const std::vector<Pt> &pts, double grid_m,
                                 const std::string &kind) {
  const double len = pathLength(pts);
  Pt a{0.0, 0.0};
  Pt b{0.0, 0.0};
  pointAtArc(pts, len * 0.33, a);
  pointAtArc(pts, len * 0.67, b);
  char buf[96];
  std::snprintf(buf, sizeof(buf), "|%lld,%lld,%lld,%lld", sigBin(a.x, grid_m),
                sigBin(a.y, grid_m), sigBin(b.x, grid_m), sigBin(b.y, grid_m));
  return kind + buf;
}

inline std::string candidateSignature(const EscapeCandidate &c, double grid_m) {
  if (!c.sig.empty()) return c.sig;
  return routeSignature(c.pts, grid_m, c.kind);
}

// ---------------------------------------------------------------------------
// Candidate generation geometry
// ---------------------------------------------------------------------------
inline double detourLookahead(int round, const EscapeParams &p) {
  return round >= 1 ? p.detour_lookahead_widened_m : p.detour_lookahead_m;
}

inline const std::vector<double> &detourBerths(int round, const EscapeParams &p) {
  return round >= 1 ? p.detour_berths_widened_m : p.detour_berths_m;
}

// True when a detour is worth generating at all: round 0 only bypasses a
// genuinely tight spot, later rounds always try (the route is known-bad).
inline bool detourWanted(double worst_clearance, int round, const EscapeParams &p) {
  if (round >= 1) return true;
  return worst_clearance < p.rescue_clearance_m;
}

// Bypass via-points around `at`, perpendicular to the local tangent
// (tangent_x, tangent_y are expected normalised; degenerate tangents are
// normalised here). Both sides of the corridor, every berth, then - from round
// 2 on - angled swings off the perpendicular, because "go around" and "go
// around at an angle" are different routes to the planner. Returns at most
// detour_max_seeds points in the sim's emission order.
inline std::vector<Pt> detourSeedPoints(const Pt &at, double tangent_x,
                                       double tangent_y, int round,
                                       const EscapeParams &p) {
  std::vector<Pt> out;
  const double tl = std::hypot(tangent_x, tangent_y);
  if (tl < 1e-9) return out;
  const double tx = tangent_x / tl;
  const double ty = tangent_y / tl;
  const double px = -ty;
  const double py = tx;
  const int cap = p.detour_max_seeds > 0 ? p.detour_max_seeds : 6;
  const std::vector<double> &offs = detourBerths(round, p);
  // Angled swings must not starve: the perpendicular loop used to claim every
  // seed slot (widened berths 3 x 2 sides = detour_max_seeds = 6 by default),
  // so the round-2 swing block below never emitted a point. Reserve one slot
  // per swing angle per side from the same cap. Mirrored byte-for-byte in
  // detourSeeds (mesh_nav.template.html).
  const int reserve = round >= 2 ? 2 * static_cast<int>(p.detour_swing_rad.size()) : 0;
  const int berth_max = std::max(0, std::min(2 * static_cast<int>(offs.size()), cap - reserve));
  const double sides[2] = {1.0, -1.0};
  int emitted = 0;
  for (int si = 0; si < 2 && emitted < berth_max; ++si) {
    for (std::size_t oi = 0; oi < offs.size() && emitted < berth_max; ++oi) {
      out.push_back(Pt{at.x + px * sides[si] * offs[oi], at.y + py * sides[si] * offs[oi]});
      ++emitted;
    }
  }
  if (round >= 2) {
    for (int si = 0; si < 2 && static_cast<int>(out.size()) < cap; ++si) {
      for (std::size_t ai = 0; ai < p.detour_swing_rad.size() && static_cast<int>(out.size()) < cap; ++ai) {
        const double x0 = px * sides[si];
        const double y0 = py * sides[si];
        const double ca = std::cos(p.detour_swing_rad[ai]);
        const double sa = std::sin(p.detour_swing_rad[ai]);
        out.push_back(Pt{at.x + (x0 * ca - y0 * sa) * p.detour_swing_range_m,
                         at.y + (x0 * sa + y0 * ca) * p.detour_swing_range_m});
      }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Acceptance
// ---------------------------------------------------------------------------
// May `cand_d` replace the stuck route scoring `cur_d`? Exactly the three
// clauses the simulator applies, in the same order:
//   - a meaningful safety gain;
//   - a rescue, i.e. the active route is inside the body budget and the
//     candidate restores it (the "I am wedged" case);
//   - from round 1 on, an equal-safety peer - trying a route the proxy ranked
//     second costs one dwell period and the proxy is not the ground truth.
// The min_candidate_clearance_m floor on the third clause is what keeps
// exploration from ever installing a route the body cannot fit through.
inline bool acceptCandidate(double cur_d, double cand_d, int round,
                           const EscapeParams &p) {
  if (cand_d > cur_d + p.install_gain_m) return true;
  if (cur_d < p.rescue_clearance_m && cand_d >= p.rescue_clearance_m) return true;
  if (round >= 1 && cand_d >= p.min_candidate_clearance_m &&
      cand_d >= cur_d - p.install_gain_m) {
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Terrain gate
// ---------------------------------------------------------------------------
enum class TerrainAction {
  kAllow,             // route rotation may run
  kHold,              // mid-flight: keep the current route, finish the climb
  kRetreatToLanding   // a reversal is required: aim at the nearest portal cell
};

inline const char *terrainActionName(TerrainAction a) {
  switch (a) {
    case TerrainAction::kHold: return "hold";
    case TerrainAction::kRetreatToLanding: return "retreat_to_landing";
    case TerrainAction::kAllow: break;
  }
  return "allow";
}

struct TerrainGateInput {
  bool on_flight{false};          // robot is between the portals of a flight
  bool at_landing{false};         // ... and within landing_snap_radius_m of one
  bool reversal_requested{false}; // local layer wants to back up / flip
  bool floor_transition{false};   // active leg is a transition/floor-crossing
};

// Is `p` standing on (within snap radius of) one of the flight's landing
// cells? Portals are the foot/head cells the recogniser publishes.
inline bool nearLanding(const Pt &p, const std::vector<Pt> &portals,
                       const EscapeParams &params) {
  for (std::size_t i = 0; i < portals.size(); ++i) {
    if (std::hypot(p.x - portals[i].x, p.y - portals[i].y) <= params.landing_snap_radius_m) {
      return true;
    }
  }
  return false;
}

// The four stair rules as one pure decision:
//   * stair_commit_enable off      -> terrain-blind escape (legacy behaviour).
//   * not on a flight, or on a
//     landing                      -> allow; a landing is a standable platform.
//   * on a flight, reversal needed -> retreat to the landing instead of
//     reversing along the path, which would run the dog off a tread edge.
//   * on a flight, no reversal      -> hold while escape_only_at_landing is set
//     (finish the climb; a route swap mid-flight would re-plan from a pose the
//     recogniser has no per-cell support for), otherwise allow.
// floor_transition only lengthens the dwell (escapeDwellMs); it must never
// drop the stair leg, which is why it is not a separate outcome here.
inline TerrainAction terrainGate(const TerrainGateInput &in, const EscapeParams &p) {
  if (!p.stair_commit_enable) return TerrainAction::kAllow;
  if (!in.on_flight || in.at_landing) return TerrainAction::kAllow;
  if (in.reversal_requested) return TerrainAction::kRetreatToLanding;
  if (!p.escape_only_at_landing) return TerrainAction::kAllow;
  return TerrainAction::kHold;
}

// Convenience form matching the (onFlight, atLanding, params) shape.
inline TerrainAction terrainGate(bool on_flight, bool at_landing,
                                const EscapeParams &p) {
  TerrainGateInput in;
  in.on_flight = on_flight;
  in.at_landing = at_landing;
  return terrainGate(in, p);
}

// Trigger dwell before a stall counts as an escape, longer across a floor
// transition so a slow gait handover is not mistaken for a block.
inline int escapeDwellMs(bool floor_transition, const EscapeParams &p) {
  return floor_transition ? p.transition_dwell_ms : p.trigger_dwell_ms;
}

// ---------------------------------------------------------------------------
// Selection (filter -> rank -> rotate)
// ---------------------------------------------------------------------------
struct EscapePick {
  int index{-1};        // index into the candidate vector, -1 = install nothing
  bool fallback{false}; // picked by the "everything already tried" rotation
};

// Terrain rejection happens before acceptance scoring (rule 3 of the terrain
// set): a candidate that re-enters the flight the episode is committed to is
// not de-ranked, it is not a candidate.
//
// Ranking: clearance binned by install_gain_m, then shorter first, then
// generation order. Bins rather than the simulator's original pairwise
// tolerance because a tolerance comparator is not a strict weak ordering
// (d1~d2, d2~d3, d1!~d3), so its result depends on the sort implementation -
// precisely the kind of "one true optimum the scorer cannot see" ambiguity the
// rotation layer exists to tolerate. The bin grid makes both sides agree.
inline EscapePick pickCandidate(const std::vector<EscapeCandidate> &cands,
                               double cur_d, int round,
                               const EscapeEpisode &episode,
                               const EscapeParams &p) {
  EscapePick pick;
  std::vector<std::size_t> order;
  order.reserve(cands.size());
  for (std::size_t i = 0; i < cands.size(); ++i) {
    if (cands[i].terrain_penalty > 0) continue;
    if (cands[i].pts.size() < 2) continue;
    if (!acceptCandidate(cur_d, cands[i].clearance, round, p)) continue;
    order.push_back(i);
  }
  if (order.empty()) return pick;

  const double tol = p.install_gain_m > 0.0 ? p.install_gain_m : 1e-6;
  std::stable_sort(order.begin(), order.end(),
                   [&](std::size_t a, std::size_t b) {
                     const double da = std::floor(cands[a].clearance / tol);
                     const double db = std::floor(cands[b].clearance / tol);
                     if (da != db) return da > db;
                     if (cands[a].length != cands[b].length) return cands[a].length < cands[b].length;
                     return a < b;
                   });

  const std::size_t n = order.size();
  const std::size_t start = static_cast<std::size_t>(
      round >= 0 ? (round % static_cast<int>(n)) : 0);
  for (std::size_t q = 0; q < n; ++q) {
    const std::size_t idx = order[(start + q) % n];
    const std::string sig = candidateSignature(cands[idx], p.route_sig_grid_m);
    bool tried = false;
    for (std::size_t t = 0; t < episode.tried_sigs.size() && !tried; ++t) {
      tried = episode.tried_sigs[t] == sig;
    }
    if (!tried) {
      pick.index = static_cast<int>(idx);
      return pick;
    }
  }
  // Everything the ranking offers has already been installed. From round 1 on
  // the episode is out of alternatives, so reuse the rotated pick rather than
  // give up (the map may have changed under the old signature).
  if (round > 0) {
    pick.index = static_cast<int>(order[start]);
    pick.fallback = true;
  }
  return pick;
}

// ---------------------------------------------------------------------------
// Episode bookkeeping
// ---------------------------------------------------------------------------
inline void resetEpisode(EscapeEpisode &e) {
  e.round = 0;
  e.tried_sigs.clear();
  e.installed_at = 0.0;
  e.reset_progress_m = 0.0;
}

// Record an install: advance the attempt index (saturating), remember the
// signature (bounded FIFO) and restart the progress/time watches.
inline void noteEpisodeInstall(EscapeEpisode &e, const std::string &sig,
                              double now_sec, const EscapeParams &p) {
  e.round = escapeRound(e.round + 1, p);
  e.tried_sigs.push_back(sig);
  const int cap = p.max_tried_sigs > 0 ? p.max_tried_sigs : 8;
  while (static_cast<int>(e.tried_sigs.size()) > cap) e.tried_sigs.erase(e.tried_sigs.begin());
  e.installed_at = now_sec;
  e.reset_progress_m = 0.0;
}

// True when the episode was cleared, i.e. this stuck spell is over: the dog is
// rolling again (progress past episode_reset_progress_m) or the spell went
// stale (episode_reset_timeout_sec). Only while not currently blocked, so a
// fresh escalation keeps its memory of what was already tried.
inline bool episodeMaybeReset(EscapeEpisode &e, double progress_since_install_m,
                             double now_sec, bool blocked, const EscapeParams &p) {
  if (e.tried_sigs.empty() || blocked) return false;
  const bool rolled = progress_since_install_m > p.episode_reset_progress_m;
  const bool stale = e.installed_at > 0.0 &&
                     (now_sec - e.installed_at) > p.episode_reset_timeout_sec;
  if (!rolled && !stale) return false;
  resetEpisode(e);
  return true;
}

// ---------------------------------------------------------------------------
// Trace record (same shape from mission_manager, stuck_escape.py and the
// simulator, which is what makes a simulator tuning claim transferable)
// ---------------------------------------------------------------------------
struct EscapeTraceRecord {
  double stamp{0.0};
  std::string episode_id;
  int round{0};
  std::string kind;
  double clearance{0.0};
  double current{0.0};
  std::string sig;
  std::string action;      // terrainActionName() or "installed"/"no_candidate"
  bool on_flight{false};
};

inline void appendJsonString(std::string &out, const std::string &s) {
  out.push_back('"');
  for (std::size_t i = 0; i < s.size(); ++i) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    if (c == '"' || c == '\\') {
      out.push_back('\\');
      out.push_back(static_cast<char>(c));
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if (c == '\t') {
      out += "\\t";
    } else if (c < 0x20) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "\\u%04x", c);
      out += buf;
    } else {
      out.push_back(static_cast<char>(c));  // UTF-8 passes through, like Python's ensure_ascii=False
    }
  }
  out.push_back('"');
}

// Compact single-line JSON, key order fixed so diffs across the two runtimes
// read as the same record.
inline std::string formatEscapeTrace(const EscapeTraceRecord &r) {
  std::string out = "{";
  out += "\"stamp\":";
  char num[32];
  std::snprintf(num, sizeof(num), "%.3f", r.stamp);
  out += num;
  out += ",\"episode_id\":";
  appendJsonString(out, r.episode_id);
  out += ",\"round\":" + std::to_string(r.round);
  out += ",\"kind\":";
  appendJsonString(out, r.kind);
  out += ",\"clearance\":";
  std::snprintf(num, sizeof(num), "%.3f", r.clearance);
  out += num;
  out += ",\"current\":";
  std::snprintf(num, sizeof(num), "%.3f", r.current);
  out += num;
  out += ",\"sig\":";
  appendJsonString(out, r.sig);
  out += ",\"action\":";
  appendJsonString(out, r.action);
  out += ",\"on_flight\":";
  out += (r.on_flight ? "true" : "false");
  out += "}";
  return out;
}

}  // namespace mission_manager_gate

#endif  // MISSION_MANAGER_ESCAPE_ROUTE_GATE_H
