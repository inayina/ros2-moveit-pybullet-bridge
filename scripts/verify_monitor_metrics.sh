#!/usr/bin/env bash
# Verify FR-MON distribution metrics, threshold reload, injection detection, and timeline evidence.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT="${1:-${ROOT}/docs/samples/monitor-metrics.json}"
TIMELINE="${2:-${ROOT}/docs/samples/monitor-metrics-timeline.csv}"
BAG_DIR="${3:-${ROOT}/docs/samples/monitor-metrics-rosbag}"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-90}"

echo "==> Preflight: monitor packages"
verify_env_require_pkg pybullet_bridge
verify_env_require_pkg dist_monitor
verify_env_require_pkg risk_engine

echo "==> Cleaning stale monitor/bridge processes"
pkill -f "ros2 launch pybullet_bridge" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
pkill -f "/dist_monitor/monitor_node" 2>/dev/null || true
pkill -f "/risk_engine/risk_node" 2>/dev/null || true
pkill -f "ros2 bag record.*monitor/distribution_metrics" 2>/dev/null || true
sleep 1

echo "==> Launch portfolio_demo (DIRECT, dual-source monitor, ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
setsid ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic motion_source:=iiwa enable_camera:=false \
  >/tmp/fr_mon_verify_launch.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -- "-${LAUNCH_PID}" 2>/dev/null || kill "${LAUNCH_PID}" 2>/dev/null || true
  if [ -n "${BAG_PID:-}" ]; then
    kill -- "-${BAG_PID}" 2>/dev/null || kill "${BAG_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

rm -rf "${BAG_DIR}"
echo "==> Record short rosbag2 evidence: ${BAG_DIR}"
setsid timeout 24s ros2 bag record \
  -o "${BAG_DIR}" /monitor/distribution_metrics \
  >/tmp/fr_mon_verify_bag.log 2>&1 &
BAG_PID=$!

echo "==> Measure distribution monitor metrics"
if ! python3 "${SCRIPT_DIR}/check_monitor_metrics.py" \
  --output "${OUTPUT}" \
  --timeline-csv "${TIMELINE}"; then
  echo "==> Portfolio launch log tail"
  tail -80 /tmp/fr_mon_verify_launch.log || true
  echo "==> rosbag log tail"
  tail -40 /tmp/fr_mon_verify_bag.log || true
  exit 1
fi

wait "${BAG_PID}" 2>/dev/null || true
if [ ! -f "${BAG_DIR}/metadata.yaml" ]; then
  echo "[FAIL] rosbag metadata not found: ${BAG_DIR}/metadata.yaml" >&2
  tail -40 /tmp/fr_mon_verify_bag.log || true
  exit 1
fi

cleanup
trap - EXIT
echo "[PASS] FR-MON verification complete: ${OUTPUT}"
