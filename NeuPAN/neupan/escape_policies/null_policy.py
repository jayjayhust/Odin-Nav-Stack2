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

"""Null escape policy: never overrides the MPC output.

Useful as a slot smoke test and for benchmark runs that measure the MPC
unsupervised (``local_planner.escape_policy.backend: null``). It keeps the
same contract as every other policy but always declines to act, so the node
publishes the planner command untouched.
"""

from .base import EscapeDecision, EscapePolicy


class NullEscapePolicy(EscapePolicy):
    """Inert policy: ``enable`` stays false, ``decide()`` is never a trigger."""

    def __init__(self, cfg=None):
        cfg = cfg or {}
        self.enable = bool(cfg.get("enable", False))
        self.criterion = "none"
        self.exit_obs_distance = float(cfg.get("exit_obs_distance", 1.0))
        self.stair_suppress_enable = False

    def decide(self, now, robot_state, obstacle_points,
               preferred_direction=0, require_preferred_direction=False,
               preferred_min_clearance=0.57):
        return EscapeDecision()
