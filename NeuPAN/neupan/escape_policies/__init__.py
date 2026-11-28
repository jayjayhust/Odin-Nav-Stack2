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

"""Escape-policy slot registry.

The stuck-escape slot replaces only the *maneuver* half of local avoidance;
the NeuPAN MPC (PAN/NRMP/DUNE) is untouched. Selection precedence in
``neupan_ros.py``:

1. ``NEUPAN_ESCAPE_POLICY`` (exported by the launcher from
   ``local_planner.escape_policy.backend`` in ``robots/<robot>/robot.yaml``)
2. ``escape_policy.backend`` in this robot's ``neupan/config.yaml``
3. ``stuck_escape`` (default, behaviour unchanged from previous releases)

Built-in backends:

``stuck_escape``
    Reference "back up + turn + replan" state machine. All of its previous
    parameters stay under the ``stuck_escape:`` key of ``neupan/config.yaml``,
    so an untouched robot behaves exactly as before.
``null``
    Inert policy, keeps the MPC output untouched (benchmark / slot test).

Custom backends: either ``register_escape_policy(name, factory)`` at import
time, or select a dotted module path (e.g. ``my_pkg.my_policy``) whose module
exposes ``create_policy(cfg)`` returning an object with ``decide()``.
"""

from __future__ import annotations

import importlib

from .base import EscapeDecision, EscapePolicy, as_escape_policy, zeros_cmd
from .null_policy import NullEscapePolicy
from .stuck_escape import StuckEscapePolicy

__all__ = [
    "EscapeDecision",
    "EscapePolicy",
    "NullEscapePolicy",
    "StuckEscapePolicy",
    "as_escape_policy",
    "available_escape_policies",
    "create_escape_policy",
    "register_escape_policy",
    "zeros_cmd",
]


def _make_stuck_escape(cfg):
    # criterion defaults to "displacement" when the key is absent, matching the
    # historical StuckEscapePolicy(cfg, criterion=cfg.get(...)) call site.
    return StuckEscapePolicy(cfg, criterion=cfg.get("criterion", "displacement"))


_BUILTIN = {
    "stuck_escape": _make_stuck_escape,
    "null": lambda cfg: NullEscapePolicy(cfg),
}


def register_escape_policy(name, factory):
    """Register a custom escape policy factory under ``name``.

    ``factory`` receives the policy config dict and must return an object that
    implements ``decide(...)`` (see :class:`neupan.escape_policies.base.EscapePolicy`).
    """
    key = str(name).strip()
    if not key or "." in key:
        raise ValueError(f"invalid escape policy name: {name!r}")
    if not callable(factory):
        raise TypeError("escape policy factory must be callable")
    _BUILTIN[key] = factory


def available_escape_policies():
    return sorted(_BUILTIN)


def _factory_from_module(module_path):
    module = importlib.import_module(module_path)
    factory = getattr(module, "create_policy", None)
    if not callable(factory):
        raise TypeError(
            f"escape policy module {module_path!r} must expose create_policy(cfg)"
        )
    return factory


def create_escape_policy(backend, cfg=None):
    """Instantiate the escape policy selected by ``backend``.

    Args:
        backend: registered name (``stuck_escape`` / ``null`` / custom) or a
            dotted module path exposing ``create_policy(cfg)``.
        cfg: policy configuration dict (may be empty).
    """
    cfg = dict(cfg or {})
    name = str(backend or "stuck_escape").strip() or "stuck_escape"
    factory = _BUILTIN.get(name)
    if factory is None:
        if "." not in name:
            raise ValueError(
                f"unknown escape policy {name!r}; available: {available_escape_policies()} "
                "(or use a dotted module path exposing create_policy)"
            )
        factory = _factory_from_module(name)
    return as_escape_policy(factory(cfg))
