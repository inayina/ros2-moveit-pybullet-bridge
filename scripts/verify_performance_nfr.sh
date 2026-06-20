#!/usr/bin/env bash
# Verify NFR-P performance criteria with live bridge / monitor measurements.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT="${1:-${ROOT}/docs/samples/performance-nfr-metrics.json}"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-93}"

echo "==> Preflight: performance packages"
verify_env_require_pkg pybullet_bridge
verify_env_require_pkg dist_monitor
verify_env_require_pkg risk_engine

echo "==> Cleaning stale performance processes"
pkill -f "ros2 launch pybullet_bridge" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
pkill -f "/dist_monitor/monitor_node" 2>/dev/null || true
pkill -f "/risk_engine/risk_node" 2>/dev/null || true
sleep 1

echo "==> Launch portfolio_demo for NFR-P (DIRECT, dual source, camera off)"
setsid ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic motion_source:=none enable_camera:=false \
  >/tmp/nfr_p_verify_portfolio.log 2>&1 &
PORT_PID=$!

cleanup() {
  kill -- "-${PORT_PID}" 2>/dev/null || kill "${PORT_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "==> Measure NFR-P performance"
if ! python3 "${SCRIPT_DIR}/check_performance_nfr.py" --output "${OUTPUT}"; then
  echo "==> portfolio log tail"
  tail -100 /tmp/nfr_p_verify_portfolio.log || true
  exit 1
fi

cleanup
trap - EXIT
echo "[PASS] NFR-P verification complete: ${OUTPUT}"
