#!/usr/bin/env bash
# Verify FR-RSK risk aggregation, R3 e-stop, attribution, and acknowledge flow.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT="${1:-${ROOT}/docs/samples/risk-management-metrics.json}"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"

echo "==> Preflight: risk management packages"
verify_env_require_pkg pybullet_bridge
verify_env_require_pkg dist_monitor
verify_env_require_pkg risk_engine

echo "==> Cleaning stale risk/bridge processes"
pkill -f "ros2 launch pybullet_bridge" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
pkill -f "/dist_monitor/monitor_node" 2>/dev/null || true
pkill -f "/risk_engine/risk_node" 2>/dev/null || true
sleep 1

echo "==> Launch portfolio_demo (DIRECT, risk stack, ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
setsid ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic motion_source:=iiwa enable_camera:=false \
  >/tmp/fr_rsk_verify_launch.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -- "-${LAUNCH_PID}" 2>/dev/null || kill "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "==> Measure risk management behavior"
if ! python3 "${SCRIPT_DIR}/check_risk_management.py" --output "${OUTPUT}"; then
  echo "==> Risk launch log tail"
  tail -100 /tmp/fr_rsk_verify_launch.log || true
  exit 1
fi

cleanup
trap - EXIT
echo "[PASS] FR-RSK verification complete: ${OUTPUT}"
