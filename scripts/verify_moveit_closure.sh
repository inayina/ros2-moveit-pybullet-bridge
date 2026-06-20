#!/usr/bin/env bash
# Verify FR-MOV MoveIt planning/execution closure on iiwa7.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT="${1:-${ROOT}/docs/samples/moveit-closure-metrics.json}"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"

echo "==> Preflight: MoveIt closure packages"
verify_env_require_pkg moveit_config
verify_env_require_pkg pybullet_bridge
verify_env_require_pkg manipulation_actions

echo "==> Cleaning stale MoveIt/bridge processes"
pkill -f "ros2 launch moveit_config" 2>/dev/null || true
pkill -f "/moveit_ros_move_group/move_group" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
pkill -f "/pybullet_bridge/trajectory_controller_node" 2>/dev/null || true
sleep 1

echo "==> Launch m2_iiwa_demo (DIRECT, no RViz, ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
setsid ros2 launch moveit_config m2_iiwa_demo.launch.py \
  sim_mode:=DIRECT use_rviz:=false enable_dual_source:=false \
  >/tmp/moveit_closure_verify.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -- "-${LAUNCH_PID}" 2>/dev/null || kill "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "==> Measure MoveIt planning/execution closure"
if ! python3 "${SCRIPT_DIR}/check_moveit_closure.py" --output "${OUTPUT}"; then
  echo "==> MoveIt launch log tail"
  tail -80 /tmp/moveit_closure_verify.log || true
  exit 1
fi

cleanup
trap - EXIT
echo "[PASS] MoveIt closure verification complete: ${OUTPUT}"
