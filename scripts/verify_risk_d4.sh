#!/usr/bin/env bash
# Phase S5a: D4 comm health + R2 degraded mode smoke test.
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

echo "==> Launch portfolio_demo (DIRECT, 15s)"
ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic >/tmp/verify_risk_d4.log 2>&1 &
LAUNCH_PID=$!
sleep 10

echo "==> Check /monitor/comm_health"
if ! timeout 5 ros2 topic echo /monitor/comm_health --once >/tmp/comm_health_once.yaml 2>/dev/null; then
  echo "[FAIL] /monitor/comm_health not publishing" >&2
  kill "$LAUNCH_PID" 2>/dev/null || true
  exit 1
fi
grep -q 'comm_health_aggregate' /tmp/comm_health_once.yaml
echo "  comm_health topic OK"

echo "==> Check comm_health_score in DistributionMetrics"
METRICS=$(timeout 5 ros2 topic echo /monitor/distribution_metrics --once 2>/dev/null || true)
if ! echo "$METRICS" | grep -q 'comm_health_score'; then
  echo "[FAIL] comm_health_score missing from DistributionMetrics (rebuild bridge_monitor_msgs?)" >&2
  kill "$LAUNCH_PID" 2>/dev/null || true
  exit 1
fi
echo "  comm_health_score field OK"

echo "==> Check degraded_mode flag path (risk status publishes)"
RISK=$(timeout 5 ros2 topic echo /risk/status --once 2>/dev/null || true)
if ! echo "$RISK" | grep -q 'degraded_mode'; then
  echo "[FAIL] degraded_mode missing from /risk/status" >&2
  kill "$LAUNCH_PID" 2>/dev/null || true
  exit 1
fi
echo "  degraded_mode field OK"

kill "$LAUNCH_PID" 2>/dev/null || true
sleep 1

echo "[PASS] verify_risk_d4.sh complete (D4 comm health + R2 wiring)"
