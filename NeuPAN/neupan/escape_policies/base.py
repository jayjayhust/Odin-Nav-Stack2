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

"""Escape-policy slot contract.

The local avoidance loop (NeuPAN PAN/NRMP/DUNE) is one thing; what to do when
that loop stalls is another. This module declares the second half as a slot so
a site can replace the maneuver without touching the MPC: the ROS node keeps
its side effects (logging, ``/neupan/replan``, MPC weight reset) and every
policy only has to produce a decision.

A policy is any object that follows :class:`EscapePolicy`. Only ``decide()``
is abstract; the remaining hooks degrade to no-ops so a minimal custom policy
does not have to implement progress criteria, terrain gating or post-escape
grace it does not use.

Contract (consumed by ``neupan/ros/neupan_ros.py``):

``enable``                  bool, False disables the override entirely
``criterion``               str, informational (logged at startup)
``exit_obs_distance``       float, front clearance that counts as "cleared"
``stair_suppress_enable``   bool, opt-in terrain suppression
``decide(...)``             one control frame -> :class:`EscapeDecision`
``update_grace(...)``       returns ``(q_theta_scale, q_s_scale)`` or None
``set_reference_path(...)`` reference path for progress-based stall metrics
``note_terrain(...)``       terrain verdict from the recognizer (``up``/``down``)
``min_obs_distance_in_cone(...)`` nearest obstacle in the front cone

``neupan_ros.py`` also reads the state-machine attributes below (``active``,
``count``, ``direction``, ``grace_*``, ...) for its trace, recovery shadow and
log lines. They are declared here with inert defaults so a policy that only
implements ``decide()`` is safe to plug into the node.

Registered backends live in :mod:`neupan.escape_policies`; select one with
``local_planner.escape_policy.backend`` in ``robots/<robot>/robot.yaml``
(exported as ``NEUPAN_ESCAPE_POLICY``) or with ``escape_policy.backend`` in
this robot's ``neupan/config.yaml``.
"""

from __future__ import annotations

import numpy as np


class EscapeDecision:
    """Result of one policy step.

    Attributes:
        cmd: ``(2, 1)`` array to publish instead of the MPC output, or ``None``
            to let the caller use its own command.
        entered / exited: state-machine edges, for logging by the caller.
        request_replan: the caller should trigger a fresh global plan.
        grace_scales: ``(q_theta_scale, q_s_scale)`` to apply, ``None`` when
            unchanged. ``(1.0, 1.0)`` means "restore the nominal values".
        info: diagnostic values (front_obs, stall metric, count, ...).
    """

    __slots__ = ("cmd", "entered", "exited", "request_replan", "grace_scales", "info")

    def __init__(self, cmd=None, entered=False, exited=False,
                 request_replan=False, grace_scales=None, info=None):
        self.cmd = cmd
        self.entered = entered
        self.exited = exited
        self.request_replan = request_replan
        self.grace_scales = grace_scales
        self.info = info or {}


class EscapePolicy:
    """Base class of the escape-policy slot (see the module docstring)."""

    #: Disables the override entirely; the MPC output is used untouched.
    enable = False
    #: Stall criterion name, informational.
    criterion = "none"
    #: Front clearance (m) above which "still blocked" is reported as cleared.
    exit_obs_distance = 1.0
    #: Opt-in: suppress the maneuver while the robot is on a staircase.
    stair_suppress_enable = False

    # --- state surface read by neupan_ros.py (trace, recovery shadow, logs) ---
    #: True while the policy is overriding the MPC command.
    active = False
    #: Consecutive stall frames, as reported in ``EscapeDecision.info``.
    count = 0
    #: Turn direction of the active maneuver (+1 left, -1 right, 0 idle).
    direction = 0.0
    #: Frames of continuous stall required before the maneuver starts.
    trigger_frames = 0
    #: Length of the stall-measurement window, seconds.
    window_sec = 1.0
    #: Reversing / rotating speeds of the maneuver, used for the trace only.
    back_speed = 0.2
    rotate_speed = 0.5
    min_rotation = 0.0
    #: Front clearance below which a blocked robot is "watched".
    front_obs_threshold = 1.0
    #: Post-escape grace window (relaxed MPC weights).
    grace_active = False
    grace_q_theta_scale = 1.0
    grace_q_s_scale = 1.0
    grace_max = 0.0
    grace_min_distance = 0.0
    #: Path-block grace (obstacle sitting on the reference path).
    pb_enable = False
    pb_active = False
    pb_dist = 0.0
    #: Sidestep (strafe) support of the chassis, if the policy uses it.
    strafe_enable = False
    strafe_speed = 0.0
    strafe_direction_sign = 0

    def decide(self, now, robot_state, obstacle_points,
               preferred_direction=0, require_preferred_direction=False,
               preferred_min_clearance=0.57) -> EscapeDecision:
        """Return the escape command for one control frame.

        Args:
            now: wall-clock timestamp (``time.time()``), same clock as
                :meth:`note_terrain`.
            robot_state: ``(3, 1)`` ``[x, y, theta]`` in the map frame.
            obstacle_points: ``(2, n)`` obstacle points in the map frame, or
                None before the first scan.
            preferred_direction: side (+1 / -1 / 0) the recovery contract
                would like the robot to turn towards, when required.
            require_preferred_direction: True while a recovery episode is
                being committed; the policy must honour ``preferred_direction``.
            preferred_min_clearance: clearance the preferred side must offer
                before it may be used.
        """
        raise NotImplementedError

    def update_grace(self, now, robot_state, obstacle_points):
        """Post-escape grace window; return ``(q_theta_scale, q_s_scale)``
        to relax, or None to restore the nominal weights."""
        return None

    def set_reference_path(self, path):
        """Install the reference path used by progress-based stall metrics."""

    def note_terrain(self, now, state, steps=0):
        """Feed the terrain-recognizer verdict (``flat`` / ``up`` / ``down``)."""

    def min_obs_distance_in_cone(self, robot_state, obstacle_points):
        """Nearest obstacle inside the front cone, ``inf`` when unknown."""
        if robot_state is None or obstacle_points is None:
            return float("inf")
        return float("inf")

    def side_clearance(self, robot_state, obstacle_points, side=0):
        """``(min_distance, count)`` in the forward half-cone on one ``side``."""
        return float("inf"), 0

    def note_pose(self, now, x, y):
        """Record the current pose (stall criteria built on displacement)."""

    def note_progress(self, now, x, y):
        """Record the current pose against the reference path."""

    def note_cmd(self, v, w):
        """Record the command the MPC asked for, before any override."""

    def note_rotation(self, now, yaw_rate):
        """Record measured yaw rate (limit-cycle / rotation guards)."""

    def note_motion(self, now, forward_v):
        """Record measured forward speed."""

    def note_path_block(self, blocked):
        """Feed the reference-path-blocked flag; return the policy's own state.

        Returning False keeps the MPC weights nominal, i.e. a policy that does
        not implement path-block grace leaves ``q_theta``/``q_s`` untouched.
        """
        return False


def as_escape_policy(obj) -> EscapePolicy:
    """Validate that ``obj`` satisfies the slot contract (duck-typed).

    ``decide`` is the only hard requirement: everything else has a safe
    default on :class:`EscapePolicy`, which is injected onto instances that do
    not define their own (custom policies may subclass the base or not).
    """
    if not callable(getattr(obj, "decide", None)):
        raise TypeError(
            "escape policy must implement decide(now, robot_state, "
            "obstacle_points, ...) -> EscapeDecision"
        )
    for name, default in (
        ("enable", EscapePolicy.enable),
        ("criterion", EscapePolicy.criterion),
        ("exit_obs_distance", EscapePolicy.exit_obs_distance),
        ("stair_suppress_enable", EscapePolicy.stair_suppress_enable),
        ("active", EscapePolicy.active),
        ("count", EscapePolicy.count),
        ("direction", EscapePolicy.direction),
        ("trigger_frames", EscapePolicy.trigger_frames),
        ("window_sec", EscapePolicy.window_sec),
        ("back_speed", EscapePolicy.back_speed),
        ("rotate_speed", EscapePolicy.rotate_speed),
        ("min_rotation", EscapePolicy.min_rotation),
        ("front_obs_threshold", EscapePolicy.front_obs_threshold),
        ("grace_active", EscapePolicy.grace_active),
        ("grace_q_theta_scale", EscapePolicy.grace_q_theta_scale),
        ("grace_q_s_scale", EscapePolicy.grace_q_s_scale),
        ("grace_max", EscapePolicy.grace_max),
        ("grace_min_distance", EscapePolicy.grace_min_distance),
        ("pb_enable", EscapePolicy.pb_enable),
        ("pb_active", EscapePolicy.pb_active),
        ("pb_dist", EscapePolicy.pb_dist),
        ("strafe_enable", EscapePolicy.strafe_enable),
        ("strafe_speed", EscapePolicy.strafe_speed),
        ("strafe_direction_sign", EscapePolicy.strafe_direction_sign),
    ):
        if not hasattr(obj, name):
            setattr(obj, name, default)
    for name in ("update_grace", "set_reference_path", "note_terrain",
                 "min_obs_distance_in_cone", "side_clearance", "note_pose",
                 "note_progress", "note_cmd", "note_rotation", "note_motion",
                 "note_path_block"):
        if not callable(getattr(obj, name, None)):
            setattr(obj, name, getattr(EscapePolicy, name).__get__(obj))
    return obj


def zeros_cmd():
    """Convenience zero command in the ``(2, 1)`` shape the node expects."""
    return np.zeros((2, 1), dtype=float)
