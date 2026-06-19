#!/usr/bin/env bash
# Portfolio integration smoke test (Plan C — iiwa7 profile).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${ROS2_WS_ROOT:-${HOME}/ros2_ws}"

# shellcheck source=integration_paths.sh
source "${SCRIPT_DIR}/integration_paths.sh"
resolve_integration_paths

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"

echo "==> Cleaning stale ROS processes"
pkill -f "ros2 launch pybullet_bridge" 2>/dev/null || true
pkill -f "ros2 launch hoc_console" 2>/dev/null || true
pkill -f "/hoc_console/hoc_server" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
pkill -f "/dist_monitor/monitor_node" 2>/dev/null || true
pkill -f "/risk_engine/risk_node" 2>/dev/null || true
sleep 1

echo "==> Integration paths"
echo "    EPISODE_DATA_LAB_ROOT=${EPISODE_DATA_LAB_ROOT:-<unset>}"
echo "    LEROBOT_EXPORT=${LEROBOT_EXPORT:-<unset>}"

echo "==> Checking iiwa7 profile URDF"
verify_env_require_pkg pybullet_bridge
verify_env_require_pkg dist_monitor

python3 - <<'PY'
from pybullet_bridge.robot_profiles import resolve_profile_config
import os
cfg = resolve_profile_config('iiwa7')
assert os.path.isfile(cfg['urdf_path']), cfg['urdf_path']
print('URDF:', cfg['urdf_path'])
print('DOF:', len(cfg['home_positions']))
PY

if [ -d "${LEROBOT_EXPORT}" ]; then
  echo "==> Offline LeRobot compare (episode-data-lab export)"
  if ! ros2 pkg executables dist_monitor 2>/dev/null | grep -q offline_compare; then
    echo "[FAIL] dist_monitor offline_compare not installed. Rebuild workspace:" >&2
    echo "  cd ~/ros2_ws && colcon build --packages-select dist_monitor --symlink-install" >&2
    exit 1
  fi
  ros2 run dist_monitor offline_compare \
    --real-dataset "${LEROBOT_EXPORT}" \
    --sim-dataset "${LEROBOT_EXPORT}" \
    --min-samples 50 | head -20
else
  echo "[SKIP] LeRobot export not found at ${LEROBOT_EXPORT}"
fi

echo "==> Launch portfolio_demo (DIRECT, smoke + action servers)"
ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic >/tmp/portfolio_verify.log 2>&1 &
PORT_PID=$!
sleep 8
if ros2 action list 2>/dev/null | grep -q '/manipulation/pick'; then
  echo "  /manipulation/pick OK"
else
  echo "[FAIL] /manipulation/pick not available after portfolio_demo launch" >&2
  kill "$PORT_PID" 2>/dev/null || true
  exit 1
fi
if ros2 action list 2>/dev/null | grep -q '/manipulation/place'; then
  echo "  /manipulation/place OK"
else
  echo "[FAIL] /manipulation/place not available after portfolio_demo launch" >&2
  kill "$PORT_PID" 2>/dev/null || true
  exit 1
fi
tail -15 /tmp/portfolio_verify.log || true
kill "$PORT_PID" 2>/dev/null || true
sleep 1

echo "[PASS] Portfolio verification complete"
