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

// Stuck-escape ROUTE-ROTATION contract, C++ side.
//
// This is the parity gate the header promises: it runs the SAME
// docs/multifloor/sim/escape_contract.fixture.json through escape_route_gate.h
// that docs/multifloor/sim/.probe/probe_escape_contract.js runs through the
// simulator's ESC_CONTRACT block. The fixture is the only place the expected
// behaviour is written down, so "the simulator tests what the robot does" is
// checked rather than asserted - change a rule in one implementation and not
// the other and this binary stops matching the fixture.
//
// It mirrors the same package's replan_install_gate_test.cpp: standalone, no
// ROS, no gtest, because the gate library is deliberately dependency-free and
// the parity check has to be runnable without a full catkin build.
//
//   g++ -std=c++14 -Wall -Wextra -I include
//       -DESCALE_FIXTURE_PATH='"/abs/path/escape_contract.fixture.json"'
//       test/test_escape_route_gate.cpp -o /tmp/escape_gate_test && /tmp/escape_gate_test
//
// The fixture path may also be given as argv[1].

#include "mission_manager/escape_route_gate.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace mission_manager_gate;

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (cond) {                                                                \
      ++g_pass;                                                                \
    } else {                                                                   \
      ++g_fail;                                                                \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);              \
    }                                                                          \
  } while (0)

#define CHECK_MSG(cond, msg)                                                   \
  do {                                                                         \
    if (cond) {                                                                \
      ++g_pass;                                                                \
    } else {                                                                   \
      ++g_fail;                                                                \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, (msg));              \
    }                                                                          \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
  do {                                                                         \
    const double va = (a);                                                     \
    const double vb = (b);                                                     \
    if (std::fabs(va - vb) <= (tol)) {                                         \
      ++g_pass;                                                                \
    } else {                                                                   \
      ++g_fail;                                                                \
      std::printf("FAIL %s:%d  |%.6f - %.6f| > %.6f\n", __FILE__, __LINE__,    \
                  va, vb, static_cast<double>(tol));                          \
    }                                                                          \
  } while (0)

// ---------------------------------------------------------------------------
// Minimal JSON reader.
//
// The fixture is the input, so the test needs to parse it; but pulling in a
// JSON dependency would break the "buildable with a bare g++" property this
// gate shares with replan_install_gate_test.cpp. What follows is a compact
// recursive-descent reader that understands exactly what the fixture uses:
// objects, arrays, strings (with \uXXXX), numbers, true/false/null. Unknown
// members are preserved rather than dropped so a future key still shows up in
// the default-parity count.
// ---------------------------------------------------------------------------
namespace mini_json {

struct Value {
  enum Type { Null, Bool, Num, Str, Arr, Obj } type = Null;
  bool boolean = false;
  double num = 0.0;
  std::string str;
  std::vector<Value> arr;
  std::vector<std::pair<std::string, Value> > obj;

  bool has(const std::string &k) const { return find(k) != NULL; }
  const Value *find(const std::string &k) const {
    for (std::size_t i = 0; i < obj.size(); ++i)
      if (obj[i].first == k) return &obj[i].second;
    return NULL;
  }
  // Accessors with defaults, mirroring how the ROS side reads config.
  bool asBool(bool def = false) const { return type == Bool ? boolean : def; }
  double asNum(double def = 0.0) const { return type == Num ? num : def; }
  int asInt(int def = 0) const { return type == Num ? static_cast<int>(num) : def; }
  std::string asStr(const std::string &def = "") const { return type == Str ? str : def; }
};

struct Parser {
  const std::string &s;
  std::size_t i = 0;
  bool ok = true;
  explicit Parser(const std::string &src) : s(src) {}

  void ws() {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) ++i;
  }
  Value parse() {
    ws();
    Value v;
    if (i >= s.size()) { ok = false; return v; }
    const char c = s[i];
    if (c == '{') return object();
    if (c == '[') return array();
    if (c == '"') { v.type = Value::Str; v.str = string(); return v; }
    if (c == 't' || c == 'f') return literal_bool();
    if (c == 'n') { i += 4; v.type = Value::Null; return v; }
    return number();
  }
  Value object() {
    Value v; v.type = Value::Obj; ++i;  // consume '{'
    ws();
    if (i < s.size() && s[i] == '}') { ++i; return v; }
    while (i < s.size()) {
      ws();
      std::string key = string();
      ws();
      if (i < s.size() && s[i] == ':') ++i;
      Value val = parse();
      v.obj.push_back(std::make_pair(key, val));
      ws();
      if (i < s.size() && s[i] == ',') { ++i; continue; }
      if (i < s.size() && s[i] == '}') { ++i; break; }
      ok = false; break;
    }
    return v;
  }
  Value array() {
    Value v; v.type = Value::Arr; ++i;  // consume '['
    ws();
    if (i < s.size() && s[i] == ']') { ++i; return v; }
    while (i < s.size()) {
      v.arr.push_back(parse());
      ws();
      if (i < s.size() && s[i] == ',') { ++i; continue; }
      if (i < s.size() && s[i] == ']') { ++i; break; }
      ok = false; break;
    }
    return v;
  }
  std::string string() {
    std::string out;
    if (i >= s.size() || s[i] != '"') { ok = false; return out; }
    ++i;
    while (i < s.size()) {
      const char c = s[i++];
      if (c == '"') return out;
      if (c == '\\') {
        if (i >= s.size()) break;
        const char e = s[i++];
        switch (e) {
          case 'n': out.push_back('\n'); break;
          case 't': out.push_back('\t'); break;
          case 'r': out.push_back('\r'); break;
          case 'b': out.push_back('\b'); break;
          case 'f': out.push_back('\f'); break;
          case '/': out.push_back('/'); break;
          case '"': out.push_back('"'); break;
          case '\\': out.push_back('\\'); break;
          case 'u': {
            unsigned cp = 0;
            for (int k = 0; k < 4 && i < s.size(); ++k) {
              const char h = s[i++];
              cp <<= 4;
              if (h >= '0' && h <= '9') cp |= (h - '0');
              else if (h >= 'a' && h <= 'f') cp |= (h - 'a' + 10);
              else if (h >= 'A' && h <= 'F') cp |= (h - 'A' + 10);
            }
            // Encode the code point as UTF-8 (the fixture is ASCII, this keeps
            // the reader correct for anything else).
            if (cp < 0x80) out.push_back(static_cast<char>(cp));
            else if (cp < 0x800) {
              out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
              out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
            } else {
              out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
              out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
              out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
            }
            break;
          }
          default: out.push_back(e); break;
        }
      } else {
        out.push_back(c);
      }
    }
    ok = false;
    return out;
  }
  Value literal_bool() {
    Value v; v.type = Value::Bool;
    if (s.compare(i, 4, "true") == 0) { v.boolean = true; i += 4; }
    else if (s.compare(i, 5, "false") == 0) { v.boolean = false; i += 5; }
    else ok = false;
    return v;
  }
  Value number() {
    Value v; v.type = Value::Num;
    const std::size_t start = i;
    if (i < s.size() && (s[i] == '-' || s[i] == '+')) ++i;
    while (i < s.size() && ((s[i] >= '0' && s[i] <= '9') || s[i] == '.' ||
                            s[i] == 'e' || s[i] == 'E' || s[i] == '-' || s[i] == '+')) ++i;
    v.num = std::strtod(s.substr(start, i - start).c_str(), NULL);
    return v;
  }
};

}  // namespace mini_json

// ---------------------------------------------------------------------------
// Fixture -> gate plumbing
// ---------------------------------------------------------------------------

// EscapeParams with a single key overridden, exactly the way a profile or a
// case's "params" block overrides one field on the JS side.
static void applyOverride(EscapeParams &p, const std::string &k,
                          const mini_json::Value &v) {
  if (k == "enable") p.enable = v.asBool();
  else if (k == "candidate_inflation_scales") { p.candidate_inflation_scales.clear(); for (std::size_t i = 0; i < v.arr.size(); ++i) p.candidate_inflation_scales.push_back(v.arr[i].asNum()); }
  else if (k == "detour_berths_m") { p.detour_berths_m.clear(); for (std::size_t i = 0; i < v.arr.size(); ++i) p.detour_berths_m.push_back(v.arr[i].asNum()); }
  else if (k == "detour_berths_widened_m") { p.detour_berths_widened_m.clear(); for (std::size_t i = 0; i < v.arr.size(); ++i) p.detour_berths_widened_m.push_back(v.arr[i].asNum()); }
  else if (k == "detour_lookahead_m") p.detour_lookahead_m = v.asNum();
  else if (k == "detour_lookahead_widened_m") p.detour_lookahead_widened_m = v.asNum();
  else if (k == "detour_max_seeds") p.detour_max_seeds = v.asInt();
  else if (k == "detour_swing_rad") { p.detour_swing_rad.clear(); for (std::size_t i = 0; i < v.arr.size(); ++i) p.detour_swing_rad.push_back(v.arr[i].asNum()); }
  else if (k == "detour_swing_range_m") p.detour_swing_range_m = v.asNum();
  else if (k == "install_gain_m") p.install_gain_m = v.asNum();
  else if (k == "rescue_clearance_m") p.rescue_clearance_m = v.asNum();
  else if (k == "min_candidate_clearance_m") p.min_candidate_clearance_m = v.asNum();
  else if (k == "max_rounds") p.max_rounds = v.asInt();
  else if (k == "route_sig_grid_m") p.route_sig_grid_m = v.asNum();
  else if (k == "max_tried_sigs") p.max_tried_sigs = v.asInt();
  else if (k == "episode_reset_progress_m") p.episode_reset_progress_m = v.asNum();
  else if (k == "episode_reset_timeout_sec") p.episode_reset_timeout_sec = v.asNum();
  else if (k == "stair_commit_enable") p.stair_commit_enable = v.asBool();
  else if (k == "escape_only_at_landing") p.escape_only_at_landing = v.asBool();
  else if (k == "landing_snap_radius_m") p.landing_snap_radius_m = v.asNum();
  else if (k == "trigger_dwell_ms") p.trigger_dwell_ms = v.asInt();
  else if (k == "transition_dwell_ms") p.transition_dwell_ms = v.asInt();
  else if (k == "replan_throttle_ms") p.replan_throttle_ms = v.asInt();
  else if (k == "trace_enable") p.trace_enable = v.asBool();
  else std::printf("NOTE case overrode an unknown key '%s'\n", k.c_str());
}

static std::vector<Pt> ptsFrom(const mini_json::Value &v) {
  std::vector<Pt> out;
  for (std::size_t i = 0; i < v.arr.size(); ++i) {
    const mini_json::Value &p = v.arr[i];
    out.push_back(Pt{p.arr.size() > 0 ? p.arr[0].asNum() : 0.0,
                     p.arr.size() > 1 ? p.arr[1].asNum() : 0.0});
  }
  return out;
}

// The C++ default-constructed EscapeParams must agree key-for-key with the
// fixture's `defaults`, which is the same agreement the JS probe checks between
// its ESCP block and the fixture. A parameter added on one side only, or with a
// different shipped value, fails here rather than drifting silently.
static void checkDefaults(const mini_json::Value &dflt) {
  const EscapeParams p;  // shipped defaults
  CHECK_MSG(p.enable == dflt.find("enable")->asBool(), "default enable");
  CHECK_MSG(p.candidate_inflation_scales.size() == dflt.find("candidate_inflation_scales")->arr.size() &&
                p.candidate_inflation_scales[0] == 1.0 && p.candidate_inflation_scales[2] == 2.0,
            "default candidate_inflation_scales");
  CHECK_MSG(p.detour_berths_m.size() == 2 && p.detour_berths_m[0] == 0.5 && p.detour_berths_m[1] == 0.95,
            "default detour_berths_m");
  CHECK_MSG(p.detour_berths_widened_m.size() == 3 && p.detour_berths_widened_m[2] == 1.45,
            "default detour_berths_widened_m");
  CHECK_MSG(p.detour_lookahead_m == dflt.find("detour_lookahead_m")->asNum(), "default detour_lookahead_m");
  CHECK_MSG(p.detour_lookahead_widened_m == dflt.find("detour_lookahead_widened_m")->asNum(),
            "default detour_lookahead_widened_m");
  CHECK_MSG(p.detour_max_seeds == dflt.find("detour_max_seeds")->asInt(), "default detour_max_seeds");
  CHECK_MSG(p.detour_swing_rad.size() == 2 && p.detour_swing_rad[0] == -0.6 && p.detour_swing_rad[1] == 0.6,
            "default detour_swing_rad");
  CHECK_MSG(p.detour_swing_range_m == dflt.find("detour_swing_range_m")->asNum(), "default detour_swing_range_m");
  CHECK_MSG(p.install_gain_m == dflt.find("install_gain_m")->asNum(), "default install_gain_m");
  CHECK_MSG(p.rescue_clearance_m == dflt.find("rescue_clearance_m")->asNum(), "default rescue_clearance_m");
  CHECK_MSG(p.min_candidate_clearance_m == dflt.find("min_candidate_clearance_m")->asNum(),
            "default min_candidate_clearance_m");
  CHECK_MSG(p.max_rounds == dflt.find("max_rounds")->asInt(), "default max_rounds");
  CHECK_MSG(p.route_sig_grid_m == dflt.find("route_sig_grid_m")->asNum(), "default route_sig_grid_m");
  CHECK_MSG(p.max_tried_sigs == dflt.find("max_tried_sigs")->asInt(), "default max_tried_sigs");
  CHECK_MSG(p.episode_reset_progress_m == dflt.find("episode_reset_progress_m")->asNum(),
            "default episode_reset_progress_m");
  CHECK_MSG(p.episode_reset_timeout_sec == dflt.find("episode_reset_timeout_sec")->asNum(),
            "default episode_reset_timeout_sec");
  CHECK_MSG(p.stair_commit_enable == dflt.find("stair_commit_enable")->asBool(), "default stair_commit_enable");
  CHECK_MSG(p.escape_only_at_landing == dflt.find("escape_only_at_landing")->asBool(),
            "default escape_only_at_landing");
  CHECK_MSG(p.landing_snap_radius_m == dflt.find("landing_snap_radius_m")->asNum(),
            "default landing_snap_radius_m");
  CHECK_MSG(p.trigger_dwell_ms == dflt.find("trigger_dwell_ms")->asInt(), "default trigger_dwell_ms");
  CHECK_MSG(p.transition_dwell_ms == dflt.find("transition_dwell_ms")->asInt(), "default transition_dwell_ms");
  CHECK_MSG(p.replan_throttle_ms == dflt.find("replan_throttle_ms")->asInt(), "default replan_throttle_ms");
  CHECK_MSG(p.trace_enable == dflt.find("trace_enable")->asBool(), "default trace_enable");
  // Count guard: every fixture default must be a known struct field and vice
  // versa. 24 is the current surface; bump it together with the struct, the JS
  // block and the four config.yaml files.
  CHECK_MSG(dflt.obj.size() == 24, "fixture default key count is 24");
}

static void checkSignatures(const mini_json::Value &sigs) {
  for (std::size_t i = 0; i < sigs.arr.size(); ++i) {
    const mini_json::Value &s = sigs.arr[i];
    const std::string got =
        routeSignature(ptsFrom(*s.find("pts")), s.find("grid_m")->asNum(), s.find("kind")->asStr());
    const std::string want = s.find("expect_sig")->asStr();
    CHECK_MSG(got == want, ("signature '" + s.find("name")->asStr() + "': got " + got + " want " + want).c_str());
  }
}

static void checkCases(const mini_json::Value &cases) {
  for (std::size_t i = 0; i < cases.arr.size(); ++i) {
    const mini_json::Value &c = cases.arr[i];
    const std::string name = c.find("name")->asStr();

    EscapeParams p;
    const mini_json::Value *over = c.find("params");
    if (over) {
      for (std::size_t k = 0; k < over->obj.size(); ++k)
        applyOverride(p, over->obj[k].first, over->obj[k].second);
    }

    const int round = c.has("attempt") ? escapeRound(c.find("attempt")->asInt(), p)
                                       : c.find("round")->asInt();
    const mini_json::Value &expect = *c.find("expect");
    if (expect.has("round"))
      CHECK_MSG(round == expect.find("round")->asInt(), (name + ": round").c_str());

    // Terrain gate -> action. The fixture carries on_flight / at_landing /
    // reversal / floor_transition booleans.
    TerrainGateInput tin;
    tin.on_flight = c.has("on_flight") && c.find("on_flight")->asBool();
    tin.at_landing = c.has("at_landing") && c.find("at_landing")->asBool();
    tin.reversal_requested = c.has("reversal") && c.find("reversal")->asBool();
    tin.floor_transition = c.has("floor_transition") && c.find("floor_transition")->asBool();
    const std::string action = terrainActionName(terrainGate(tin, p));
    CHECK_MSG(action == expect.find("action")->asStr(),
              (name + ": action " + action + " want " + expect.find("action")->asStr()).c_str());

    if (expect.has("dwell_ms")) {
      const int ms = escapeDwellMs(tin.floor_transition, p);
      CHECK_MSG(ms == expect.find("dwell_ms")->asInt(), (name + ": dwell_ms").c_str());
    }

    // Episode verdict: a hold or a retreat never reaches the picker, mirroring
    // replanGlobal. Terrain cases still carry candidates so the empty pick is
    // attributed to the gate, not to the acceptance math.
    EscapePick pick;
    const mini_json::Value *candsV = c.find("candidates");
    if (action == "allow" && candsV) {
      std::vector<EscapeCandidate> cands;
      for (std::size_t j = 0; j < candsV->arr.size(); ++j) {
        const mini_json::Value &cj = candsV->arr[j];
        EscapeCandidate e;
        e.pts = ptsFrom(*cj.find("pts"));
        e.clearance = cj.find("clearance")->asNum();
        e.length = cj.find("length")->asNum();
        e.kind = cj.find("kind")->asStr();
        e.terrain_penalty = cj.has("terrain_penalty") ? cj.find("terrain_penalty")->asInt() : 0;
        cands.push_back(e);
      }
      EscapeEpisode ep;
      const mini_json::Value *trV = c.find("tried");
      if (trV) {
        for (std::size_t j = 0; j < trV->arr.size(); ++j) ep.tried_sigs.push_back(trV->arr[j].asStr());
      }
      pick = pickCandidate(cands, c.find("current")->asNum(), round, ep, p);
    }
    const int want = (expect.find("pick_index")->type == mini_json::Value::Str)
                         ? -1
                         : expect.find("pick_index")->asInt();
    CHECK_MSG(pick.index == want, (name + ": pick " + std::to_string(pick.index) + " want " +
                                   std::to_string(want)).c_str());
    CHECK_MSG(pick.fallback == (expect.has("fallback") && expect.find("fallback")->asBool()),
              (name + ": fallback").c_str());
  }
}

// A handful of assertions the fixture cannot express but the header must keep:
// episode bookkeeping saturates and bounded-resets. These guard the state
// machine the escape episode in mission_manager drives (P3a).
static void checkEpisodeBookkeeping() {
  EscapeParams p;
  p.max_rounds = 2;
  EscapeEpisode e;
  // Three installs in a row: round saturates at max_rounds, tried_sigs grows.
  noteEpisodeInstall(e, "A|0,0,1,1", 1.0, p);
  CHECK(e.round == 1);
  noteEpisodeInstall(e, "B|0,0,2,2", 2.0, p);
  CHECK(e.round == 2);
  noteEpisodeInstall(e, "C|0,0,3,3", 3.0, p);
  CHECK(e.round == 2);  // clamped
  CHECK(e.tried_sigs.size() == 3);

  // A rolling dog past episode_reset_progress_m clears the episode, but only
  // while not blocked.
  CHECK(episodeMaybeReset(e, 0.5, 3.5, true, p) == false);   // blocked: hold memory
  CHECK(episodeMaybeReset(e, 1.3, 4.0, false, p) == true);   // rolled forward
  CHECK(e.tried_sigs.empty());
  CHECK(e.round == 0);

  // Staleness clears it too.
  noteEpisodeInstall(e, "D|0,0,4,4", 10.0, p);
  CHECK(episodeMaybeReset(e, 0.0, 10.0 + p.episode_reset_timeout_sec + 0.1, false, p) == true);

  // tried_sigs FIFO bound.
  p.max_tried_sigs = 2;
  EscapeEpisode f;
  noteEpisodeInstall(f, "s1", 0.0, p);
  noteEpisodeInstall(f, "s2", 0.0, p);
  noteEpisodeInstall(f, "s3", 0.0, p);
  CHECK(f.tried_sigs.size() == 2);
  CHECK(f.tried_sigs[0] == "s2" && f.tried_sigs[1] == "s3");
}

// detourSeedPoints: two sides x berths, capped, plus angled swings from round 2.
static void checkDetourSeeds() {
  EscapeParams p;
  const Pt at{0.0, 0.0};
  // Along +x, so the perpendicular is +y / -y.
  const std::vector<Pt> r0 = detourSeedPoints(at, 1.0, 0.0, 0, p);
  // round 0: 2 sides * 2 berths = 4, no swings.
  CHECK(r0.size() == 4);
  CHECK_NEAR(r0[0].y, 0.5, 1e-9);
  CHECK_NEAR(r0[0].x, 0.0, 1e-9);
  const std::vector<Pt> r2 = detourSeedPoints(at, 1.0, 0.0, 2, p);
  // widened berths (3) * 2 sides would be 6, but detour_max_seeds caps it.
  CHECK(static_cast<int>(r2.size()) <= p.detour_max_seeds);
  // The cap must reserve room for the angled swings: 2 angles * 2 sides = 4,
  // so berth_max = cap - reserve = 6 - 4 = 2. Total = 2 + 4 = 6.
  // Before the reservation the perpendicular loop claimed all 6 slots and no
  // swing was ever emitted.
  const int swing_count = 2 * static_cast<int>(p.detour_swing_rad.size());  // 2 sides
  const int expected = (p.detour_max_seeds - swing_count) + swing_count;    // cap
  CHECK(static_cast<int>(r2.size()) == expected);
  bool saw_swing = false;
  for (std::size_t i = 0; i < r2.size(); ++i) {
    // A swing sits off the perpendicular: |x| > 1e-6 at |y| ~ 1.16 (range 1.3
    // rotated by +/-0.6 rad), while a berth has x == 0 exactly.
    if (std::fabs(r2[i].x) > 1e-6) saw_swing = true;
    if (i < 2) CHECK_NEAR(r2[i].x, 0.0, 1e-9);  // first two are berths
  }
  CHECK(saw_swing);
  // Degenerate tangent yields no seeds rather than a divide-by-zero.
  CHECK(detourSeedPoints(at, 0.0, 0.0, 2, p).empty());
}

static std::string slurp(const std::string &path, bool &ok) {
  std::ifstream in(path.c_str());
  ok = static_cast<bool>(in);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

int main(int argc, char **argv) {
  std::string fixture =
#ifdef ESCAPE_FIXTURE_PATH
      ESCAPE_FIXTURE_PATH;
#else
      "";
#endif
  if (argc > 1) fixture = argv[1];
  if (fixture.empty()) {
    std::printf("test_escape_route_gate: no fixture path (set ESCAPE_FIXTURE_PATH or pass argv[1])\n");
    return 2;
  }
  bool rok = false;
  const std::string text = slurp(fixture, rok);
  if (!rok) {
    std::printf("test_escape_route_gate: cannot read fixture '%s'\n", fixture.c_str());
    return 2;
  }
  mini_json::Parser parser(text);
  const mini_json::Value root = parser.parse();
  if (!parser.ok || root.type != mini_json::Value::Obj || !root.has("defaults") ||
      !root.has("signatures") || !root.has("cases")) {
    std::printf("test_escape_route_gate: fixture did not parse as expected\n");
    return 2;
  }

  checkDefaults(*root.find("defaults"));
  checkSignatures(*root.find("signatures"));
  checkCases(*root.find("cases"));
  checkEpisodeBookkeeping();
  checkDetourSeeds();

  std::printf("test_escape_route_gate: %d passed, %d failed  [%s, %d cases, %d signatures]\n",
              g_pass, g_fail, fixture.c_str(),
              static_cast<int>(root.find("cases")->arr.size()),
              static_cast<int>(root.find("signatures")->arr.size()));
  return g_fail == 0 ? 0 : 1;
}
