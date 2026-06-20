#!/usr/bin/env bash
# Same-task calibration: dual-source same JointTrajectory → non-zero KL/W1/MMD with honest meaning.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-${HOME}/ros2_ws}"
ARTIFACT_DIR="${BRIDGE_ROOT}/docs/samples"
BASELINE_FRAC="${BASELINE_FRAC:-0.3}"

IIWA_NPZ="${ARTIFACT_DIR}/same-task-iiwa-dual.npz"
IIWA_JSON="${ARTIFACT_DIR}/same-task-iiwa-metrics.json"
LEROBOT_NPZ="${ARTIFACT_DIR}/same-task-lerobot-dual.npz"
LEROBOT_JSON="${ARTIFACT_DIR}/same-task-lerobot-metrics.json"

# shellcheck source=integration_paths.sh
source "${SCRIPT_DIR}/integration_paths.sh"
resolve_integration_paths

export EPISODE_DATA_LAB_ROOT LEROBOT_EXPORT BRIDGE_ROOT ROS2_WS_ROOT

echo "==> Same-task calibration experiment"
echo "    LEROBOT_EXPORT=${LEROBOT_EXPORT:-<unset>}"
echo "    BASELINE_FRAC=${BASELINE_FRAC}"

mkdir -p "${ARTIFACT_DIR}"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${BRIDGE_ROOT}"
verify_env_require_pkg dist_monitor
verify_env_require_pkg pybullet_bridge

if ! ros2 pkg executables dist_monitor 2>/dev/null | grep -q offline_compare; then
  echo "==> colcon build dist_monitor + pybullet_bridge"
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  (cd "${ROS2_WS_ROOT}" && colcon build \
    --packages-select dist_monitor pybullet_bridge --symlink-install)
  # shellcheck disable=SC1091
  source "${ROS2_WS_ROOT}/install/setup.bash"
fi

_stop_launch() {
  pkill -f "ros2 launch pybullet_bridge portfolio_demo" 2>/dev/null || true
  pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
  pkill -f "dual_source_recorder" 2>/dev/null || true
  sleep 1
}

_wait_for_bridge() {
  for _ in $(seq 1 25); do
    if ros2 topic list 2>/dev/null | grep -q '/bridge/sim/joint_states'; then
      return 0
    fi
    sleep 1
  done
  return 1
}

_run_capture() {
  local motion="$1"
  local duration="$2"
  local output="$3"
  local log_tag="$4"

  _stop_launch
  setsid ros2 launch pybullet_bridge portfolio_demo.launch.py \
    sim_mode:=DIRECT \
    real_source:=topic \
    motion_source:="${motion}" \
    lerobot_dataset_path:="${LEROBOT_EXPORT}" \
    episode_index:=0 \
    >/tmp/same_task_"${log_tag}".log 2>&1 &
  LAUNCH_PID=$!

  _cleanup_capture() {
    kill -- "-${LAUNCH_PID}" 2>/dev/null || kill "${LAUNCH_PID}" 2>/dev/null || true
  }
  trap _cleanup_capture RETURN

  if ! _wait_for_bridge; then
    echo "[FAIL] Bridge topics not ready (${log_tag})" >&2
    tail -30 "/tmp/same_task_${log_tag}.log" || true
    return 1
  fi

  sleep 1
  if ! python3 "${SCRIPT_DIR}/capture_dual_source_trajectory.py" \
      --duration "${duration}" \
      --output "${output}" \
      --min-samples 50; then
    echo "[FAIL] Dual capture failed (${log_tag})" >&2
    tail -30 "/tmp/same_task_${log_tag}.log" || true
    return 1
  fi

  _cleanup_capture
  trap - RETURN
  sleep 1
  return 0
}

_compare_dual() {
  local npz="$1"
  local json_out="$2"
  ros2 run dist_monitor offline_compare -- \
    --dual-npz "${npz}" \
    --baseline-frac "${BASELINE_FRAC}" \
    --min-samples 50 \
    --mmd-permutations 100 > "${json_out}"
  echo "  wrote ${json_out}"
}

echo "==> Experiment A: dual-source + iiwa_motion_demo (same command, domain-randomized Real)"
_run_capture iiwa 12 "${IIWA_NPZ}" iiwa
_compare_dual "${IIWA_NPZ}" "${IIWA_JSON}"

if [ -d "${LEROBOT_EXPORT:-}" ]; then
  echo "==> Experiment B: dual-source + LeRobot episode replay (cross-repo same motion)"
  _run_capture lerobot 20 "${LEROBOT_NPZ}" lerobot
  _compare_dual "${LEROBOT_NPZ}" "${LEROBOT_JSON}"
else
  echo "[WARN] LEROBOT_EXPORT missing; skipping Experiment B"
fi

_stop_launch

echo "==> Generate calibration report + charts (aligned)"
python3 "${SCRIPT_DIR}/regenerate_all_reports.py"

echo ""
echo "[PASS] Same-task calibration complete"
echo "  Report: ${BRIDGE_ROOT}/docs/samples/same-task-calibration-report.html"
echo "  IIWA metrics: ${IIWA_JSON}"
if [ -f "${LEROBOT_JSON}" ]; then
  echo "  LeRobot replay metrics: ${LEROBOT_JSON}"
fi
