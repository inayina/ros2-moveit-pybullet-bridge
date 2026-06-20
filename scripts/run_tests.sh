#!/usr/bin/env bash
# Run unit, node, and launch integration tests.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"

PYTHON="/usr/bin/python3"

echo "==> Preflight: ROS packages"
for pkg in bridge_monitor_msgs pybullet_bridge dist_monitor risk_engine hoc_console manipulation_actions; do
  verify_env_require_pkg "${pkg}" || exit 1
done

if ! "${PYTHON}" -c "import pybullet" 2>/dev/null; then
  echo "==> Installing pybullet for ${PYTHON} (required by bridge tests)"
  "${PYTHON}" -m pip install pybullet --break-system-packages -q \
    || "${PYTHON}" -m pip install pybullet --user -q
fi

PACKAGES=(dist_monitor risk_engine pybullet_bridge hoc_console manipulation_actions)
LAUNCH_IGNORE=(--ignore=test/test_m1_launch.py --ignore=test/test_full_system_launch.py)
LAUNCH_TEST_TIMEOUT_SEC="${LAUNCH_TEST_TIMEOUT_SEC:-120}"
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
if (cd "${ROOT}/pybullet_bridge" && timeout --kill-after=10s "${LAUNCH_TEST_TIMEOUT_SEC}s" \
    "${PYTHON}" -m pytest test/test_m1_launch.py test/test_full_system_launch.py -v); then
  :
else
  FAILED=1
fi

if [ "${FAILED}" -ne 0 ]; then
  echo "[FAIL] 部分测试未通过"
  echo "若报 package 'pybullet_bridge' not found，请清理后重建：" >&2
  echo "  rm -rf ~/ros2_ws/build/pybullet_bridge ~/ros2_ws/install/pybullet_bridge" >&2
  echo "  cd ~/ros2_ws && colcon build --packages-select pybullet_bridge --symlink-install" >&2
  exit 1
fi

echo "[PASS] 全部测试通过（单元 + 节点 + 集成）"
