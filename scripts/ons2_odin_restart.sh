#!/usr/bin/env bash
# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Restart the Odin1 ROS1 driver so it re-reads control_command.yaml.
# floor_manager rewrites relocalization_map_abs_path before calling this.
#
# Usage:
#   ons2_odin_restart.sh [--mode process|systemd] [--unit odin1-driver] [--dry-run]
#
# process (default): SIGINT the running roslaunch, then relaunch it detached.
# systemd          : systemctl restart <unit> (recommended on production robots).

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${GO2_LOG_DIR:-${HOME}/go2_logs}"
MODE="process"
UNIT="odin1-driver"
DRY_RUN=0
LAUNCH_PKG="odin_ros_driver"
LAUNCH_FILE="odin1_ros1.launch"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="${2:-process}"; shift 2 ;;
    --unit) UNIT="${2:-odin1-driver}"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help)
      sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "[odin_restart] unknown argument: $1" >&2; exit 2 ;;
  esac
done

mkdir -p "${LOG_DIR}"

if [[ "${MODE}" == "systemd" ]]; then
  echo "[odin_restart] systemctl restart ${UNIT}"
  if [[ "${DRY_RUN}" -eq 1 ]]; then exit 0; fi
  systemctl restart "${UNIT}"
  rc=$?
  if [[ ${rc} -ne 0 ]]; then
    echo "[odin_restart] FAILED: systemctl restart ${UNIT} (rc=${rc})" >&2
  fi
  exit ${rc}
fi

if [[ "${MODE}" != "process" ]]; then
  echo "[odin_restart] unknown mode '${MODE}' (expected process|systemd)" >&2
  exit 2
fi

if [[ "${DRY_RUN}" -eq 1 ]]; then
  echo "[odin_restart] DRY-RUN: would stop '${LAUNCH_FILE}' / 'host_sdk_sample' and relaunch"
  exit 0
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export GO2_ROOT="${REPO_ROOT}"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash 2>/dev/null || true
# shellcheck disable=SC1091
source "${REPO_ROOT}/ros_ws/devel/setup.bash" 2>/dev/null || true

echo "[odin_restart] stopping Odin1 driver"
pkill -INT -f "${LAUNCH_FILE}" 2>/dev/null || true
pkill -INT -f "host_sdk_sample" 2>/dev/null || true
for _ in $(seq 1 20); do
  if ! pgrep -f "${LAUNCH_FILE}" >/dev/null && ! pgrep -f "host_sdk_sample" >/dev/null; then
    break
  fi
  sleep 0.5
done
pkill -9 -f "${LAUNCH_FILE}" 2>/dev/null || true
pkill -9 -f "host_sdk_sample" 2>/dev/null || true

STAMP="$(date +%Y%m%d_%H%M%S)"
ODIN_LOG="${LOG_DIR}/odin_restart_${STAMP}.log"
echo "[odin_restart] starting ${LAUNCH_PKG}/${LAUNCH_FILE} -> ${ODIN_LOG}"
setsid nohup roslaunch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
  > "${ODIN_LOG}" 2>&1 < /dev/null &

for _ in $(seq 1 30); do
  if pgrep -f "${LAUNCH_FILE}" >/dev/null && pgrep -f "host_sdk_sample" >/dev/null; then
    echo "[odin_restart] driver process is up"
    exit 0
  fi
  sleep 1
done

echo "[odin_restart] FAILED: driver did not come back within 30s (see ${ODIN_LOG})" >&2
tail -n 20 "${ODIN_LOG}" >&2 || true
exit 1
