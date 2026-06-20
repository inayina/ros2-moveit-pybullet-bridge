#!/usr/bin/env bash
# Verify FR-BRG bridge communication timing and publish stability.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT="${1:-${ROOT}/docs/samples/bridge-comm-metrics.json}"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"

echo "==> Preflight: bridge package"
verify_env_require_pkg pybullet_bridge

echo "==> Cleaning stale bridge processes"
pkill -f "ros2 launch pybullet_bridge" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
pkill -f "/risk_engine/risk_node" 2>/dev/null || true
sleep 1

echo "==> Launch bridge (DIRECT, iiwa7, dual-source, ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
setsid ros2 launch pybullet_bridge bridge.launch.py \
  sim_mode:=DIRECT robot:=iiwa7 enable_dual_source:=true enable_camera:=false \
  >/tmp/bridge_comm_verify.log 2>&1 &
BRIDGE_PID=$!

cleanup() {
  kill -- "-${BRIDGE_PID}" 2>/dev/null || kill "${BRIDGE_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "==> Measure bridge communication"
if ! python3 "${SCRIPT_DIR}/check_bridge_comm.py" --output "${OUTPUT}"; then
  echo "==> Bridge launch log tail"
  tail -40 /tmp/bridge_comm_verify.log || true
  exit 1
fi

cleanup
trap - EXIT
echo "[PASS] Bridge communication verification complete: ${OUTPUT}"
