#!/usr/bin/env bash
# 模板：楼梯自主攀爬（autonomous）
#
# 由 mission_manager 以如下形式异步调用，不要直接手工依赖它的参数格式：
#   FLOOR_TRANSITION_FROM=2F FLOOR_TRANSITION_TO=3F FLOOR_TRANSITION_TYPE=stairs \
#   FLOOR_TRANSITION_MODE=autonomous \
#   timeout --signal=INT 180 bash scripts/transition/stairs_auto_example.sh
#
# 约定（必须遵守）：
#   1. 退出码 0 = 机器人已站在目标层、且已调用 /floor_manager/arrived_at_floor
#   2. 退出码非 0 = 未完成；mission_manager 会记录失败并在等待超时后跳过该航点
#   3. 脚本结束前必须保证机器人零速 —— 爬楼过程中一旦丢定位，没有人能替它刹车
#
# 本模板只做流程编排，真正的爬楼动作由 STAIR_CLIMB_CMD 提供，因为：
#   * 宇树 Go2 的爬楼步态切换属于运动控制层接口，各版本 SDK 命名不一致；
#   * 办公楼梯踏步深度/踢面高度必须现场实测后写进动作序列，不能通用。
# 在没做完"单层安全实测"之前，floors.yaml 请保持 mode: manual。

set -uo pipefail

FROM="${FLOOR_TRANSITION_FROM:-}"
TO="${FLOOR_TRANSITION_TO:-}"
CLIMB_CMD="${STAIR_CLIMB_CMD:-}"
ARRIVE_SERVICE="${ARRIVE_SERVICE:-/floor_manager/arrived_at_floor}"
LOG_TAG="[stairs]"

fail() {
  echo "${LOG_TAG} $*" >&2
  # 任何失败都先兜底停车
  rostopic pub -1 /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
  rosservice call /neupan/stop >/dev/null 2>&1 || true
  exit 1
}

[ -n "${TO}" ] || fail "FLOOR_TRANSITION_TO is empty"
[ -n "${CLIMB_CMD}" ] || fail "STAIR_CLIMB_CMD is not set (no stair gait implementation wired yet)"

echo "${LOG_TAG} ${FROM:-?} -> ${TO}, climb command: ${CLIMB_CMD}"

# 1) 进入楼梯口前：停 NeuPAN、切爬楼步态（由 CLIMB_CMD 负责）
rosservice call /neupan/stop >/dev/null 2>&1 || true
bash -c "${CLIMB_CMD}" || fail "climb command returned non-zero"

# 2) 爬楼期间 Odin1 定位必然退化，此处不做任何 map 系规划。
#    若上层控制需要闭环，只允许使用里程计/IMU 的相对推进。

# 3) 到达目标层楼梯口：先停车，再让 floor_manager 换图并重定位
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true

echo "${LOG_TAG} calling ${ARRIVE_SERVICE} ${TO}"
rosservice call "${ARRIVE_SERVICE}" "floor: '${TO}'" || fail "arrived_at_floor call failed"

echo "${LOG_TAG} done"
exit 0
