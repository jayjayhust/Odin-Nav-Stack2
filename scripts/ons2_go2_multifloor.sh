#!/usr/bin/env bash
# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Multi-floor wrapper around ons2_go2.sh.
#
# Differences from the single-floor launcher:
#   * map_server is disabled; floor_manager publishes /map for the active floor
#   * floor_manager is started next to mission_manager
#   * mission_manager gets the floor list and lift-lobby poses
#
# Usage:
#   bash scripts/ons2_go2_multifloor.sh --map=floor1
#   INITIAL_FLOOR=2F bash scripts/ons2_go2_multifloor.sh --map=floor2

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export MULTIFLOOR=1
export ENABLE_MAP_SERVER=0
export INITIAL_FLOOR="${INITIAL_FLOOR:-1F}"
export FLOORS_CONFIG="${FLOORS_CONFIG:-$(cd "${SCRIPT_DIR}/.." && pwd)/robots/go2/floors.yaml}"

exec bash "${SCRIPT_DIR}/ons2_go2.sh" "$@"
