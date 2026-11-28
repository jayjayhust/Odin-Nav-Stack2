#!/usr/bin/env python3
# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the path-block grace (`note_path_block`) and the terrain-aware
stuck-escape suppression (`note_terrain`) state machines."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from neupan.util.stuck_escape import StuckEscapePolicy

BASE_CFG = {
    "enable": False,
    "path_block_grace_enable": True,
    "path_block_dist_m": 0.45,
    "path_block_frames": 5,
    "path_block_clear_frames": 10,
}


def new_policy(**over):
    return StuckEscapePolicy(dict(BASE_CFG, **over))


def feed(p, seq):
    """Feed a sequence of boolean blocked states and return the per-frame active flags."""
    return [p.note_path_block(b) for b in seq]


fails = 0


def run_case(name, fn):
    """Run one assertion-style case, tallying pass/fail without aborting the run."""
    global fails
    try:
        fn()
        print(name + " PASS")
    except AssertionError as e:
        fails += 1
        print(str(e) + " FAIL")


# T1: disabled switch never activates
p = new_policy(path_block_grace_enable=False)
r = feed(p, [True] * 20)
assert not any(r), "T1 failed: activated when enable=false"
print("T1 enable=false stays inactive PASS")

# T2: activates on 5 consecutive blocked frames (not 4)
p = new_policy()
r = feed(p, [True] * 4)
assert not any(r), "T2 failed: activated on 4 frames"
r += feed(p, [True])
assert r[-1] is True and p.pb_active, "T2 failed: did not activate on 5th frame"
print("T2 activates on 5 consecutive frames PASS")

# T3: interruption resets counter (3 + 1 false + 3 does not activate)
p = new_policy()
r = feed(p, [True] * 3 + [False] + [True] * 3)
assert not p.pb_active, "T3 failed: counter not reset on interruption"
print("T3 interruption resets counter PASS")

# T4: deactivates after 10 consecutive clear frames (holds on 9)
p = new_policy()
feed(p, [True] * 5)
assert p.pb_active
r = feed(p, [False] * 9)
assert p.pb_active, "T4 failed: deactivated early on 9 frames"
r += feed(p, [False])
assert not p.pb_active, "T4 failed: did not deactivate on 10th frame"
print("T4 deactivates after 10 clear frames PASS")

# T5: can re-activate after deactivation
p = new_policy()
feed(p, [True] * 5 + [False] * 10)
assert not p.pb_active
r = feed(p, [True] * 5)
assert r[-1] is True, "T5 failed: could not re-activate"
print("T5 re-activates after deactivation PASS")

# T6: shares grace scale factors with escape grace
p = new_policy()
assert abs(p.grace_q_s_scale - 0.5) < 1e-9 and abs(p.grace_q_theta_scale - 0.25) < 1e-9
print("T6 shares escape grace scales (q_s*0.5 / q_theta*0.25) PASS")


# ---------------------------------------------------------------------------
# Terrain-aware escape suppression (note_terrain). The whole point of this
# branch is that the escape no longer reverses off a staircase: while a flight
# is being traversed the stall counter is held instead of escalating, and the
# default (stair_suppress_enable=false) leaves the policy byte-identical to what
# it was before terrain existed.
# ---------------------------------------------------------------------------
STUCK_CFG = {
    "enable": True,
    "criterion": "displacement",
    "trigger_frames": 3,
    "front_obs_threshold": 0.6,
    "early_replan_enable": False,
}


def stuck_state():
    return np.array([[0.0], [0.0], [0.0]])


def close_points():
    # one obstacle dead ahead at 0.30 m: inside the cone, inside the trigger
    # threshold, and asymmetric enough that the corridor guard stays asleep.
    return np.array([[0.30], [0.0]])


def drive(p, terrain=None, frames=6, t0=10.0, dt=0.2):
    """Advance decide() `frames` times with a stalled command and an obstacle
    ahead. Re-notes the terrain each frame so freshness never races the clock.
    Returns (entered, final_count)."""
    entered = False
    for i in range(frames):
        t = t0 + i * dt
        p.note_pose(t, 0.0, 0.0)
        p.note_cmd(0.0, 0.0)
        if terrain is not None:
            p.note_terrain(t, terrain, 4)
        d = p.decide(t, stuck_state(), close_points())
        if d.entered:
            entered = True
    return entered, p.count


def t7_default_off_unchanged():
    # The opt-out has to be total: a robot with no stair detection (or the
    # feature simply off) must escape exactly as before, terrain fed or not.
    p = StuckEscapePolicy(dict(STUCK_CFG))
    assert p.stair_suppress_enable is False
    entered, _ = drive(p, terrain="up")
    assert entered, "T7: escape suppressed even though stair_suppress_enable is off"


def t8_on_flight_holds():
    p = StuckEscapePolicy(dict(STUCK_CFG, stair_suppress_enable=True))
    entered, cnt = drive(p, terrain="up")
    assert not entered, "T8: entered escape while a flight was being traversed"
    assert cnt == 0, "T8: stall count escalated instead of held (%d)" % cnt
    assert p.stair_suppressed is True


def t9_landing_releases():
    p = StuckEscapePolicy(dict(STUCK_CFG, stair_suppress_enable=True))
    drive(p, terrain="up", frames=4)          # hold, count stays at 0
    entered, _ = drive(p, terrain="flat", frames=6, t0=20.0)  # reach a landing
    assert entered, "T9: escape still held after the flight turned to flat floor"


def t10_grace_ends_at_landing():
    p = StuckEscapePolicy(dict(STUCK_CFG, stair_suppress_enable=True,
                               post_escape_grace_max_sec=5.0))
    p.grace_active = True
    p.grace_start_t = 0.0
    p.grace_start_xy = (0.0, 0.0)
    p.grace_clear_since = None
    p.note_terrain(6.0, "up", 4)              # past grace_max but still climbing
    assert p.update_grace(6.0, stuck_state(), close_points()) is None, \
        "T10: grace weights restored mid-flight"
    p.note_terrain(6.1, "flat", 0)            # reached the landing
    assert p.update_grace(6.1, stuck_state(), close_points()) == (1.0, 1.0), \
        "T10: grace window did not close on the landing"


run_case("T7 default-off leaves the escape unchanged (terrain ignored)", t7_default_off_unchanged)
run_case("T8 opt-in: mid-flight holds the stall count, no escape", t8_on_flight_holds)
run_case("T9 opt-in: a landing releases the hold, escape resumes", t9_landing_releases)
run_case("T10 grace window runs on to the landing", t10_grace_ends_at_landing)

if __name__ == "__main__":
    total = 10
    print("\ntest_path_block_grace: %d/%d PASS" % (total - fails, total))
    sys.exit(1 if fails else 0)
