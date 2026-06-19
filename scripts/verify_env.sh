#!/usr/bin/env bash
# Shared environment for milestone / integration verification scripts.
# Usage: source "$(dirname "$0")/verify_env.sh" && verify_env_init /path/to/bridge/repo

verify_env_init() {
  local repo_root="${1:-}"
  export PATH="/usr/bin:/bin:/opt/ros/jazzy/bin:${PATH}"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV

  if [ -n "${repo_root}" ]; then
    export ROS_LOG_DIR="${repo_root}/.ros_log"
    mkdir -p "${ROS_LOG_DIR}"
  fi

  if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "[FAIL] ROS 2 Jazzy not found: /opt/ros/jazzy/setup.bash" >&2
    return 1
  fi
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash

  local ws_root="${ROS2_WS_ROOT:-${HOME}/ros2_ws}"
  if [ -f "${ws_root}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${ws_root}/install/setup.bash"
  else
    echo "[WARN] ${ws_root}/install/setup.bash not found — run colcon build first" >&2
  fi

  if ! python3 -c "import pybullet" 2>/dev/null; then
    echo "[INFO] Installing pybullet for system python3 (verification)"
    python3 -m pip install pybullet --break-system-packages -q \
      || python3 -m pip install pybullet --user -q
  fi

  for pkg in websockets aiohttp pillow; do
    if ! python3 -c "import ${pkg}" 2>/dev/null; then
      echo "[INFO] Installing ${pkg} for HOC WebSocket/UI"
      python3 -m pip install "${pkg}" --break-system-packages -q \
        || python3 -m pip install "${pkg}" --user -q
    fi
  done
}

verify_env_require_pkg() {
  local pkg="$1"
  if ! ros2 pkg prefix "${pkg}" &>/dev/null; then
    echo "[FAIL] ROS package '${pkg}' not found. Build workspace first:" >&2
    echo "  cd ~/ros2_ws && colcon build --symlink-install" >&2
    return 1
  fi
}
