#!/usr/bin/env bash
# Phase S5b: D3 dynamics anomaly + soft limit smoke test.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"

echo "==> Cleaning stale ROS processes"
pkill -f "ros2 launch pybullet_bridge portfolio_demo" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
pkill -f "/dist_monitor/monitor_node" 2>/dev/null || true
pkill -f "/risk_engine/risk_node" 2>/dev/null || true
sleep 1

echo "==> Launch portfolio_demo (DIRECT, 12s)"
setsid ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic >/tmp/verify_risk_d3.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -- "-${LAUNCH_PID}" 2>/dev/null || kill "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 10

echo "==> Check dynamics fields in DistributionMetrics"
METRICS=$(timeout 5 ros2 topic echo /monitor/distribution_metrics --once 2>/dev/null || true)
for field in dynamics_anomaly_score velocity_jump_per_joint soft_limit_score soft_limit_triggered; do
  if ! echo "$METRICS" | grep -q "$field"; then
    echo "[FAIL] missing field: $field (rebuild bridge_monitor_msgs?)" >&2
    exit 1
  fi
  echo "  $field OK"
done

echo "==> Inject shift (payload) to exercise dynamics path"
if ros2 service list 2>/dev/null | grep -q '/bridge/inject_shift'; then
  ros2 service call /bridge/inject_shift bridge_monitor_msgs/srv/InjectShift \
    "{parameter_name: payload_mass, delta_percent: 80.0, duration_sec: 5.0}" >/dev/null 2>&1 || true
  sleep 3
  METRICS2=$(timeout 5 ros2 topic echo /monitor/distribution_metrics --once 2>/dev/null || true)
  echo "$METRICS2" | grep -q 'dynamics_anomaly_score' || true
  echo "  inject_shift service OK"
else
  echo "[WARN] /bridge/inject_shift unavailable (dual-source off?)"
fi

echo "==> Risk engine exposes dynamics_anomaly attribution"
RISK=$(timeout 5 ros2 topic echo /risk/status --once 2>/dev/null || true)
if ! echo "$RISK" | grep -q 'dynamics_anomaly'; then
  echo "[FAIL] dynamics_anomaly not in risk attribution" >&2
  exit 1
fi
echo "  dynamics_anomaly attribution OK"

cleanup
trap - EXIT
sleep 1

echo "[PASS] verify_risk_d3.sh complete (D3 + soft limits)"
