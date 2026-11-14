#!/usr/bin/env bash
# Stair transition hook for mission_manager (FLOOR_TRANSIT, type=stairs,
# mode=autonomous).
#
# Sequence, and why it is open loop:
#   1. stop the base      - NeuPAN must not keep publishing /cmd_vel while the
#                           gait layer owns the legs
#   2. select the gait    - forward climbing (up) or reverse climbing (down)
#   3. drive straight     - fixed speed for a fixed time; the staircase is
#                           deliberately NOT planned. Odin1 degrades on treads
#                           (grazing incidence) and both planners are 2D
#   4. stop + restore gait
#   5. declare arrival    - floor_manager then restarts the driver, relocalises
#                           and verifies the endpoint pose before mission
#                           resumes
#
# Everything the hook needs comes from mission_manager as environment variables:
#   FLOOR_TRANSITION_FROM / _TO / _TYPE / _MODE
#   REPO_ROOT (expanded by mission_manager)
#
# Extra tuning knobs (export before starting the stack):
#   STAIR_DIR            up|down   (default: derived from FROM/TO, else up)
#   STAIR_SPEED          m/s       (default 0.25)
#   STAIR_CLIMB_SECONDS  s         (default 12)
#   STAIR_WARMUP_SECONDS s         (default 1.5, gait settles before driving)
set -uo pipefail

TO="${FLOOR_TRANSITION_TO:?FLOOR_TRANSITION_TO is required}"
FROM="${FLOOR_TRANSITION_FROM:-unknown}"
DIR="${STAIR_DIR:-}"
SPEED="${STAIR_SPEED:-0.25}"
CLIMB_SECONDS="${STAIR_CLIMB_SECONDS:-12}"
WARMUP_SECONDS="${STAIR_WARMUP_SECONDS:-1.5}"

log() { echo "[stairs_gait_hook] $*"; }

# Floor ordering is a local convention; keep it here so the yaml stays data.
floor_index() {
  case "$1" in
    1F|B1) echo 1 ;;
    2F) echo 2 ;;
    3F) echo 3 ;;
    4F) echo 4 ;;
    5F) echo 5 ;;
    *) echo 0 ;;
  esac
}

if [[ -z "$DIR" ]]; then
  from_i=$(floor_index "$FROM")
  to_i=$(floor_index "$TO")
  if [[ "$from_i" -gt 0 && "$to_i" -gt 0 && "$to_i" -lt "$from_i" ]]; then
    DIR="down"
  else
    DIR="up"
  fi
fi

stop_motion() {
  for _ in 1 2 3 4 5; do
    rostopic pub -1 /cmd_vel geometry_msgs/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
      >/dev/null 2>&1 || true
  done
}

cleanup() {
  log "cleanup: stop motion, restore trot"
  stop_motion
  rosservice call /stair_gait/leave >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

log "transition $FROM -> $TO dir=$DIR speed=$SPEED climb=${CLIMB_SECONDS}s"

# 1. stop the base
stop_motion
rosservice call /neupan/stop std_msgs/Empty >/dev/null 2>&1 || \
  log "warn: /neupan/stop unavailable"

# 2. select the climbing gait
if [[ "$DIR" == "down" ]]; then
  rosservice call /stair_gait/enter_descend || {
    log "ERROR: enter_descend failed"; exit 2; }
else
  rosservice call /stair_gait/enter_climb || {
    log "ERROR: enter_climb failed"; exit 2; }
fi

sleep "$WARMUP_SECONDS"

# 3. drive straight, open loop
log "driving straight for ${CLIMB_SECONDS}s"
END=$(( $(date +%s) + CLIMB_SECONDS ))
while [[ $(date +%s) -lt $END ]]; do
  rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    "{linear: {x: $SPEED, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
  sleep 0.1
done

# 4. stop and restore the walking gait
stop_motion
rosservice call /stair_gait/leave >/dev/null 2>&1 || log "warn: leave failed"

# 5. declare arrival; floor_manager does the rest
rosservice call /floor_manager/arrived_at_floor "floor: '$TO'" || {
  log "ERROR: arrived_at_floor failed"; exit 3; }

log "done $FROM -> $TO"
