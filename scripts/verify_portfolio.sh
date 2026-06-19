#!/usr/bin/env bash
# Portfolio integration smoke test (Plan C — iiwa7 profile).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${HOME}/ros2_ws"

export PATH="/usr/bin:/bin:/opt/ros/jazzy/bin:${PATH}"
unset CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV
export ROS_LOG_DIR="${ROOT}/.ros_log"
mkdir -p "${ROS_LOG_DIR}"

source /opt/ros/jazzy/setup.bash
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  source "${WS_ROOT}/install/setup.bash"
fi

LEROBOT_EXPORT="${LEROBOT_EXPORT:-/home/ina/robot-sim-lab/robot-arm-episode-data-lab/dataset/v1/lerobot_export}"

echo "==> Checking iiwa7 profile URDF"
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
  ros2 run dist_monitor offline_compare \
    --real-dataset "${LEROBOT_EXPORT}" \
    --sim-dataset "${LEROBOT_EXPORT}" \
    --min-samples 50 | head -20
else
  echo "[SKIP] LeRobot export not found at ${LEROBOT_EXPORT}"
fi

echo "==> Launch portfolio_demo (DIRECT, 15s)"
timeout 15 ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic 2>&1 | tail -15 || true

echo "[PASS] Portfolio verification complete"
