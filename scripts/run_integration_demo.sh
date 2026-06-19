#!/usr/bin/env bash
# One-click integration: episode-data-lab (data) + ros2-moveit-pybullet-bridge (monitor).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-${HOME}/ros2_ws}"

# shellcheck source=integration_paths.sh
source "${SCRIPT_DIR}/integration_paths.sh"
resolve_integration_paths

DO_COLLECT=0
USE_DOCKER=0

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Run from the ros2-moveit-pybullet-bridge repository.

Options:
  --collect   Run batch_collect + LeRobot export in episode-data-lab first
  --docker    Run bridge verify inside Docker
  -h, --help  Show this help

Environment:
  EPISODE_DATA_LAB_ROOT  episode-data-lab checkout (see scripts/integration_paths.sh)
  LEROBOT_EXPORT         LeRobot export directory
  BRIDGE_ROOT            defaults to this repo (${BRIDGE_ROOT})
  ROS2_WS_ROOT           defaults to ~/ros2_ws
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --collect) DO_COLLECT=1 ;;
    --docker) USE_DOCKER=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

export EPISODE_DATA_LAB_ROOT LEROBOT_EXPORT BRIDGE_ROOT ROS2_WS_ROOT

echo "==> Integration paths"
echo "    EPISODE_DATA_LAB_ROOT=${EPISODE_DATA_LAB_ROOT:-<unset>}"
echo "    LEROBOT_EXPORT=${LEROBOT_EXPORT:-<unset>}"
echo "    BRIDGE_ROOT=${BRIDGE_ROOT}"

if [ -z "${EPISODE_DATA_LAB_ROOT:-}" ] || [ ! -d "${EPISODE_DATA_LAB_ROOT}" ]; then
  echo "[ERROR] episode-data-lab not found. Set EPISODE_DATA_LAB_ROOT to your checkout." >&2
  exit 1
fi

if [ "${DO_COLLECT}" -eq 1 ]; then
  echo "==> Step A: batch collect + LeRobot export"
  (
    cd "${EPISODE_DATA_LAB_ROOT}"
    python3 -m pip install -q -r requirements.txt
    python3 scripts/batch_collect.py --output dataset/v1 --num-episodes 2 --num-steps 40 --seed 42
    python3 scripts/export_lerobot_style.py dataset/v1 --output dataset/v1/lerobot_export
  )
  resolve_integration_paths
fi

if [ ! -d "${LEROBOT_EXPORT:-}" ]; then
  echo "[WARN] LeRobot export missing: ${LEROBOT_EXPORT:-<unset>}"
  echo "       Re-run with --collect or export manually in episode-data-lab."
fi

if [ "${USE_DOCKER}" -eq 1 ]; then
  echo "==> Step B: bridge verify (Docker)"
  (
    cd "${BRIDGE_ROOT}"
    docker compose build
    docker compose run --rm verify
  )
  echo "[PASS] Docker integration verify complete"
  exit 0
fi

echo "==> Step B: bridge verify (local)"
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "[ERROR] ROS 2 Jazzy not found. Install Jazzy or use --docker." >&2
  exit 1
fi

if [ ! -f "${ROS2_WS_ROOT}/install/setup.bash" ]; then
  echo "==> colcon build (first time)"
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  (
    cd "${ROS2_WS_ROOT}"
    colcon build \
      --packages-select bridge_monitor_msgs pybullet_bridge dist_monitor risk_engine hoc_console moveit_config \
      --symlink-install
  )
fi

"${BRIDGE_ROOT}/scripts/verify_portfolio.sh"

echo "[PASS] Local integration demo complete"
echo "Next: ros2 launch pybullet_bridge portfolio_demo.launch.py sim_mode:=GUI real_source:=lerobot"
