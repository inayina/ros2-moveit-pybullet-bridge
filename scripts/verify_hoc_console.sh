#!/usr/bin/env bash
# Verify FR-HOC WebSocket dashboard backend, commands, and report export.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT="${1:-${ROOT}/docs/samples/hoc-console-metrics.json}"
REPORT_DIR="${2:-${ROOT}/docs/samples/hoc-verification-reports}"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-92}"

echo "==> Preflight: HOC packages"
verify_env_require_pkg pybullet_bridge
verify_env_require_pkg dist_monitor
verify_env_require_pkg risk_engine
verify_env_require_pkg hoc_console

echo "==> Cleaning stale HOC/bridge processes"
pkill -f "ros2 launch pybullet_bridge" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
pkill -f "/dist_monitor/monitor_node" 2>/dev/null || true
pkill -f "/risk_engine/risk_node" 2>/dev/null || true
pkill -f "/hoc_console/hoc_server" 2>/dev/null || true
sleep 1

echo "==> Launch portfolio_demo (DIRECT, HOC source data, ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
setsid ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic motion_source:=iiwa enable_camera:=false \
  >/tmp/fr_hoc_verify_portfolio.log 2>&1 &
PORT_PID=$!

echo "==> Launch hoc_server backend"
setsid ros2 run hoc_console hoc_server --ros-args \
  -p serve_frontend:=false \
  -p report_output_dir:="${REPORT_DIR}" \
  >/tmp/fr_hoc_verify_server.log 2>&1 &
HOC_PID=$!

cleanup() {
  kill -- "-${PORT_PID}" 2>/dev/null || kill "${PORT_PID}" 2>/dev/null || true
  kill -- "-${HOC_PID}" 2>/dev/null || kill "${HOC_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "==> Configure HOC verification risk response"
for _ in {1..30}; do
  if ros2 param set /risk_engine cancel_move_group_on_e_stop false >/dev/null 2>&1; then
    break
  fi
  sleep 0.2
done

echo "==> Measure HOC WebSocket and commands"
if ! python3 "${SCRIPT_DIR}/check_hoc_console.py" \
  --output "${OUTPUT}" \
  --report-dir "${REPORT_DIR}"; then
  echo "==> portfolio log tail"
  tail -80 /tmp/fr_hoc_verify_portfolio.log || true
  echo "==> hoc_server log tail"
  tail -80 /tmp/fr_hoc_verify_server.log || true
  exit 1
fi

cleanup
trap - EXIT
echo "[PASS] FR-HOC verification complete: ${OUTPUT}"
