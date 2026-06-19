#!/usr/bin/env bash
# Run unit, node, and launch integration tests.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${HOME}/ros2_ws"

# ROS Jazzy rclpy + launch_testing require system Python 3.12 (not conda 3.13)
export PATH="/usr/bin:/bin:/opt/ros/jazzy/bin:${PATH}"
unset CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV
export ROS_LOG_DIR="${ROOT}/.ros_log"
mkdir -p "${ROS_LOG_DIR}"

source /opt/ros/jazzy/setup.bash
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  source "${WS_ROOT}/install/setup.bash"
fi

PYTHON="/usr/bin/python3"

if ! "${PYTHON}" -c "import pybullet" 2>/dev/null; then
  echo "==> Installing pybullet for ${PYTHON} (required by bridge tests)"
  "${PYTHON}" -m pip install pybullet --break-system-packages -q \
    || "${PYTHON}" -m pip install pybullet --user -q
fi

PACKAGES=(dist_monitor risk_engine pybullet_bridge hoc_console)
LAUNCH_IGNORE=(--ignore=test/test_m1_launch.py --ignore=test/test_full_system_launch.py)
FAILED=0

echo "==> Unit + node tests (${PYTHON})"
for pkg in "${PACKAGES[@]}"; do
  echo "=== ${pkg} ==="
  IGNORE_ARGS=()
  if [ "${pkg}" = "pybullet_bridge" ]; then
    IGNORE_ARGS=("${LAUNCH_IGNORE[@]}")
  fi
  if (cd "${ROOT}/${pkg}" && "${PYTHON}" -m pytest test/ -v "${IGNORE_ARGS[@]}"); then
    :
  else
    FAILED=1
  fi
done

echo "==> Launch integration tests (pybullet_bridge)"
if (cd "${ROOT}/pybullet_bridge" && "${PYTHON}" -m pytest \
    test/test_m1_launch.py test/test_full_system_launch.py -v); then
  :
else
  FAILED=1
fi

if [ "${FAILED}" -ne 0 ]; then
  echo "[FAIL] 部分测试未通过"
  exit 1
fi

echo "[PASS] 全部测试通过（单元 + 节点 + 集成）"
