#!/usr/bin/env bash
# 模板：电梯 API 联调（api）
#
# 由 mission_manager 异步调用，环境变量：
#   FLOOR_TRANSITION_FROM / FLOOR_TRANSITION_TO / FLOOR_TRANSITION_TYPE / FLOOR_TRANSITION_MODE
#
# 约定：退出码 0 = 机器人已到目标层并已调用 arrived_at_floor
#
# 现实约束（务必先读）：
#   * 绝大多数办公楼电梯不对外开放 API，常见三种可落地形式：
#       a) 厂商 HTTP/MQTT 接口（需物业授权）
#       b) 干接点/继电器板（按楼层按钮 + 读到位信号），走串口或 Modbus TCP
#       c) 无接口 —— 只能人工按电梯，走 mode: manual
#   * 电梯内 Odin1 无法重定位（金属轿厢封闭视野），无论哪种方式，
#     跨层都要经历一次"重定位 -> 换图"，这不是 API 能消除的。
#
# 本模板用 LIFT_API_* 环境变量抽象协议，方便替换成 Modbus/HTTP/私有 SDK。

set -uo pipefail

FROM="${FLOOR_TRANSITION_FROM:-}"
TO="${FLOOR_TRANSITION_TO:-}"
ARRIVE_SERVICE="${ARRIVE_SERVICE:-/floor_manager/arrived_at_floor}"
LOG_TAG="[lift]"

# --- 协议适配点：按现场改成 HTTP / Modbus / 干接点 ---
LIFT_API_BASE="${LIFT_API_BASE:-}"          # e.g. http://10.0.0.20/api
LIFT_API_TOKEN="${LIFT_API_TOKEN:-}"
LIFT_CALL_TIMEOUT="${LIFT_CALL_TIMEOUT:-60}"
LIFT_POLL_INTERVAL="${LIFT_POLL_INTERVAL:-2}"

stop_robot() {
  rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
}

fail() {
  echo "${LOG_TAG} $*" >&2
  stop_robot
  exit 1
}

[ -n "${TO}" ] || fail "FLOOR_TRANSITION_TO is empty"
[ -n "${LIFT_API_BASE}" ] || fail "LIFT_API_BASE is not set (no lift API available on this site)"

echo "${LOG_TAG} ${FROM:-?} -> ${TO} via ${LIFT_API_BASE}"

# 1) 停住，确认机器人不在电梯门运行范围内
stop_robot
rosservice call /neupan/stop >/dev/null 2>&1 || true

# 2) 叫梯到当前层（按现场协议替换）
#    示例：curl -fsS -X POST "${LIFT_API_BASE}/call" -H "Authorization: Bearer ${LIFT_API_TOKEN}" -d "{\"floor\":\"${FROM}\"}"
echo "${LOG_TAG} TODO: call lift to ${FROM}"

# 3) 等门开、机器人进轿厢（这一步通常需要人，或者需要门状态信号）
echo "${LOG_TAG} TODO: wait for door open / robot boarded"

# 4) 按目标层
echo "${LOG_TAG} TODO: send destination ${TO}"

# 5) 轮询到位：电梯到达 + 门开
deadline=$((SECONDS + LIFT_CALL_TIMEOUT))
while [ "${SECONDS}" -lt "${deadline}" ]; do
  # 示例：state=$(curl -fsS "${LIFT_API_BASE}/state" | jq -r .floor)
  state=""
  if [ "${state}" = "${TO}" ]; then
    break
  fi
  sleep "${LIFT_POLL_INTERVAL}"
done
[ "${state}" = "${TO}" ] || fail "lift did not reach ${TO} within ${LIFT_CALL_TIMEOUT}s"

# 6) 机器人出轿厢 -> 目标层电梯口 -> 让 floor_manager 换图重定位
echo "${LOG_TAG} TODO: wait for robot to leave the car"
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
rosservice call "${ARRIVE_SERVICE}" "floor: '${TO}'" || fail "arrived_at_floor call failed"

echo "${LOG_TAG} done"
exit 0
