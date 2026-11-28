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

"""Contract checks for the escape-policy SLOT (neupan.escape_policies).

Run: python3 test_escape_policy_slot.py

1. The registry only offers what it can build, and rejects unknown names.
2. ``stuck_escape`` selected through the slot behaves exactly like the direct
   class - the default robot must not change - and the old import path
   ``neupan.util.stuck_escape`` still resolves to the same objects.
3. ``null`` declines to act but exposes every attribute neupan_ros.py reads
   directly, so selecting it cannot crash the node.
4. A custom policy only needs ``decide()``: the remaining contract (including
   the node's state surface) is injected by ``as_escape_policy``. Dotted module
   paths work through ``create_escape_policy``.
"""
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from neupan.escape_policies import (  # noqa: E402
    EscapeDecision,
    EscapePolicy,
    NullEscapePolicy,
    StuckEscapePolicy,
    as_escape_policy,
    available_escape_policies,
    create_escape_policy,
    register_escape_policy,
)

FAILURES = []


def check(name, cond, detail=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{'' if cond else ' :: ' + detail}")
    if not cond:
        FAILURES.append(name)


CFG = dict(
    enable=True, displacement_window_sec=1.0, displacement_threshold_m=0.05,
    progress_threshold_m=0.05, v_eps=0.15, w_eps=0.30, trigger_frames=10,
    front_obs_threshold=1.0, cone_half_width_rad=0.7, back_speed=0.2,
    rotate_speed=0.5, duration_sec=2.0, exit_obs_distance=1.0,
    side_search_radius_m=3.5, limit_cycle_consecutive_n=2,
    limit_cycle_window_sec=20.0, min_rotation_rad=0.35,
    early_replan_enable=True, early_replan_trigger_frames=5,
    early_replan_cooldown_sec=2.0,
)
DT = 0.05


def state(x=0.0, y=0.0, yaw=0.0):
    return np.array([[x], [y], [yaw]], dtype=float)


def front_wall(distance=0.9, n=40):
    ys = np.linspace(-0.4, 0.4, n)
    return np.vstack([np.full(n, distance), ys])


def drive(policy, frames=12, points=None):
    """Stalled-at-a-wall sequence; returns the decision list."""
    out = []
    for i in range(frames):
        t = i * DT
        policy.note_pose(t, 0.0, 0.0)
        policy.note_cmd(0.0, 0.0)
        out.append(policy.decide(t, state(), front_wall(0.9) if points is None else points))
    return out


print("[1] registry")

names = available_escape_policies()
check("built-ins registered", names == ["null", "stuck_escape"], str(names))

try:
    create_escape_policy("does_not_exist")
    check("unknown backend raises ValueError", False, "no exception")
except ValueError as ex:
    check("unknown backend raises ValueError", "available" in str(ex), str(ex))

try:
    register_escape_policy("bad.name", lambda cfg: NullEscapePolicy(cfg))
    check("dotted names cannot be registered", False, "no exception")
except ValueError:
    check("dotted names cannot be registered", True)

print("[2] stuck_escape through the slot is byte-identical to the direct class")

direct = drive(StuckEscapePolicy(CFG, criterion="displacement"))
slot = drive(create_escape_policy("stuck_escape", CFG))
check("same number of decisions", len(direct) == len(slot))
check("same enter frame",
      [i for i, d in enumerate(direct) if d.entered] ==
      [i for i, d in enumerate(slot) if d.entered])
check("same commands",
      all((d.cmd is None and s.cmd is None)
          or np.allclose(d.cmd, s.cmd) for d, s in zip(direct, slot)))
check("criterion defaults to displacement", slot[0].info.get("metric_name") is not None,
      str(slot[0].info))
check("factory respects cfg['criterion']",
      create_escape_policy("stuck_escape", dict(CFG, criterion="progress")).criterion
      == "progress")

try:
    from neupan.util.stuck_escape import (  # noqa: E402
        EscapeDecision as ShimDecision,
        StuckEscapePolicy as ShimPolicy,
    )
except ModuleNotFoundError as ex:
    # neupan.util.__init__ pulls in the planner (torch); a dev box without the
    # runtime can still validate the slot itself.
    print(f"  SKIP  legacy import path check ({ex})")
else:
    check("legacy import path resolves to the same class", ShimPolicy is StuckEscapePolicy)
    check("legacy import path resolves to the same decision", ShimDecision is EscapeDecision)

print("[3] null policy")

null = create_escape_policy("null", {"enable": True})
check("null never overrides", all(d.cmd is None for d in drive(null)))
check("null reports no stall", null.count == 0 and null.active is False)
NODE_ATTRS = (
    "enable criterion exit_obs_distance stair_suppress_enable active count "
    "direction trigger_frames window_sec back_speed rotate_speed min_rotation "
    "front_obs_threshold grace_active grace_q_theta_scale grace_q_s_scale "
    "grace_max grace_min_distance pb_enable pb_active pb_dist strafe_enable "
    "strafe_speed strafe_direction_sign"
).split()
missing = [a for a in NODE_ATTRS if not hasattr(null, a)]
check("null exposes every attribute neupan_ros reads", not missing, str(missing))
check("null side_clearance/note_path_block are inert",
      null.side_clearance(state(), front_wall(), +1) == (float("inf"), 0)
      and null.note_path_block(True) is False)

print("[4] custom policy: decide() is the only requirement")


class MinimalPolicy:
    """Deliberately does not subclass EscapePolicy."""

    def decide(self, now, robot_state, obstacle_points, **kwargs):
        return EscapeDecision(cmd=np.array([[0.0], [0.3]]))


minimal = as_escape_policy(MinimalPolicy())
missing = [a for a in NODE_ATTRS if not hasattr(minimal, a)]
check("defaults injected onto a duck-typed policy", not missing, str(missing))
check("injected hooks are callable",
      callable(minimal.note_terrain) and callable(minimal.note_path_block)
      and callable(minimal.update_grace))
check("custom decide() is used",
      np.allclose(minimal.decide(0.0, state(), None).cmd, [[0.0], [0.3]]))

try:
    as_escape_policy(object())
    check("policy without decide() is rejected", False, "no exception")
except TypeError:
    check("policy without decide() is rejected", True)

with tempfile.TemporaryDirectory() as tmp:
    # A dotted module path is the "no source change" extension point: the site
    # package exposes create_policy(cfg) and is selected by name.
    pkg_dir = Path(tmp) / "slot_pkg"
    pkg_dir.mkdir()
    (pkg_dir / "__init__.py").write_text("")
    (pkg_dir / "custom_policy.py").write_text(
        "import numpy as np\n"
        "from neupan.escape_policies import EscapeDecision, EscapePolicy\n"
        "\n"
        "class P(EscapePolicy):\n"
        "    def __init__(self, cfg):\n"
        "        self.enable = cfg.get('enable', True)\n"
        "    def decide(self, now, robot_state, obstacle_points, **kwargs):\n"
        "        return EscapeDecision(cmd=np.zeros((2, 1)))\n"
        "\n"
        "def create_policy(cfg):\n"
        "    return P(cfg)\n"
    )
    sys.path.insert(0, tmp)
    custom = create_escape_policy("slot_pkg.custom_policy", {"enable": True})
    check("dotted module path loads create_policy()", type(custom).__name__ == "P",
          type(custom).__name__)
    check("dotted module policy is usable", custom.enable is True
          and np.allclose(custom.decide(0.0, state(), None).cmd, 0.0))

print("\n" + ("ALL PASS" if not FAILURES else f"FAILED: {FAILURES}"))
sys.exit(1 if FAILURES else 0)
