#!/usr/bin/env bash
# Verify NFR-R reliability evidence: watchdog, recovery, smoke, and rosbag.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT="${1:-${ROOT}/docs/samples/reliability-nfr-metrics.json}"
BAG_DIR="${2:-${ROOT}/docs/samples/reliability-nfr-rosbags}"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-94}"

echo "==> Preflight: reliability packages"
verify_env_require_pkg pybullet_bridge
verify_env_require_pkg dist_monitor
verify_env_require_pkg risk_engine
verify_env_require_pkg hoc_console

echo "==> Cleaning stale reliability processes"
pkill -f "ros2 launch pybullet_bridge" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
pkill -f "/dist_monitor/monitor_node" 2>/dev/null || true
pkill -f "/risk_engine/risk_node" 2>/dev/null || true
pkill -f "/hoc_console/hoc_server" 2>/dev/null || true
sleep 1

rm -rf "${BAG_DIR}"
mkdir -p "${BAG_DIR}"

echo "==> Launch portfolio_demo for NFR-R (DIRECT, dual source, camera off)"
setsid ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic motion_source:=none enable_camera:=false \
  >/tmp/nfr_r_verify_portfolio.log 2>&1 &
PORT_PID=$!

echo "==> Launch hoc_server for rosbag recording"
setsid ros2 run hoc_console hoc_server --ros-args \
  -p serve_frontend:=false \
  -p rosbag_output_dir:="${BAG_DIR}" \
  >/tmp/nfr_r_verify_hoc.log 2>&1 &
HOC_PID=$!

cleanup() {
  kill -- "-${PORT_PID}" 2>/dev/null || kill "${PORT_PID}" 2>/dev/null || true
  kill -- "-${HOC_PID}" 2>/dev/null || kill "${HOC_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "==> Measure NFR-R reliability"
if ! python3 "${SCRIPT_DIR}/check_reliability_nfr.py" --output "${OUTPUT}"; then
  echo "==> portfolio log tail"
  tail -100 /tmp/nfr_r_verify_portfolio.log || true
  echo "==> hoc_server log tail"
  tail -100 /tmp/nfr_r_verify_hoc.log || true
  exit 1
fi

cleanup
trap - EXIT
echo "[PASS] NFR-R verification complete: ${OUTPUT}"
