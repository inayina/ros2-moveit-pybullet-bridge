#!/usr/bin/env bash
# P1-1: Capture real README assets + bonus (RViz recording, HOC browser screenshot).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"

SKIP_RVIZ="${SKIP_RVIZ:-false}"
SKIP_HOC_BROWSER="${SKIP_HOC_BROWSER:-false}"

if [ ! -f "${ROOT}/docs/samples/same-task-iiwa-dual.npz" ]; then
  echo "==> No dual NPZ — quick iiwa dual capture"
  export EPISODE_DATA_LAB_ROOT="${EPISODE_DATA_LAB_ROOT:-${HOME}/robot-sim-lab/robot-arm-episode-data-lab}"
  pkill -f "ros2 launch pybullet_bridge portfolio_demo" 2>/dev/null || true
  sleep 1
  ros2 launch pybullet_bridge portfolio_demo.launch.py \
    sim_mode:=DIRECT real_source:=topic motion_source:=iiwa \
    >/tmp/capture_readme_launch.log 2>&1 &
  LAUNCH_PID=$!
  for _ in $(seq 1 25); do
    ros2 topic list 2>/dev/null | grep -q '/bridge/sim/joint_states' && break
    sleep 1
  done
  sleep 1
  python3 "${SCRIPT_DIR}/capture_dual_source_trajectory.py" \
    --duration 12 \
    --output "${ROOT}/docs/samples/same-task-iiwa-dual.npz" \
    --min-samples 50
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
fi

# --- Bonus A: RViz real recording ---
RVIZ_OK=false
if [ "${SKIP_RVIZ}" != true ] && [ -n "${DISPLAY:-}" ]; then
  echo "==> P1-1 bonus A: RViz + Plan & Execute recording"
  if bash "${SCRIPT_DIR}/capture_m2_rviz_recording.sh"; then
    RVIZ_OK=true
    echo "  m2-iiwa-rviz.gif updated from screen capture"
  else
    echo "[WARN] RViz recording failed — keeping previous m2-iiwa-rviz.gif"
  fi
else
  echo "[SKIP] RViz recording (SKIP_RVIZ=true or no DISPLAY)"
fi

# --- NPZ-based GIFs (m3, m2-iiwa-pybullet; skip rviz if bonus A succeeded) ---
NPZ_ARGS=(--skip-hoc-browser)
if [ "${RVIZ_OK}" = true ]; then
  NPZ_ARGS+=(--skip-rviz-gif)
fi
python3 "${SCRIPT_DIR}/capture_readme_assets.py" "${NPZ_ARGS[@]}"

# --- Bonus B: HOC Playwright screenshot ---
HOC_PID=""
PORT_PID=""
cleanup() {
  kill "${HOC_PID}" "${PORT_PID}" 2>/dev/null || true
  pkill -f "ros2 launch pybullet_bridge portfolio_demo" 2>/dev/null || true
  pkill -f "ros2 launch hoc_console hoc_prod" 2>/dev/null || true
}
trap cleanup EXIT

VENV_PY="${ROOT}/.venv/bin/python3"
if [ "${SKIP_HOC_BROWSER}" != true ]; then
  if ! "${VENV_PY}" -c "import playwright" 2>/dev/null; then
    echo "==> Installing Playwright"
    bash "${SCRIPT_DIR}/install_playwright.sh"
  fi

  echo "==> P1-1 bonus B: HOC browser screenshot (hoc_prod :8080)"
  if [ -d "${ROOT}/hoc_console/frontend/node_modules" ] || (
    cd "${ROOT}/hoc_console/frontend" && npm install --silent && npm run build --silent
  ); then
    pkill -f "ros2 launch pybullet_bridge portfolio_demo" 2>/dev/null || true
    pkill -f "ros2 launch hoc_console hoc_prod" 2>/dev/null || true
    sleep 1
    ros2 launch pybullet_bridge portfolio_demo.launch.py \
      sim_mode:=DIRECT real_source:=topic motion_source:=iiwa \
      >/tmp/capture_readme_portfolio.log 2>&1 &
    PORT_PID=$!
    sleep 8
    ros2 launch hoc_console hoc_prod.launch.py >/tmp/capture_readme_hoc.log 2>&1 &
    HOC_PID=$!
    sleep 12
    if PATH="${ROOT}/.venv/bin:${PATH}" "${VENV_PY}" "${SCRIPT_DIR}/capture_readme_assets.py" \
        --hoc-url http://127.0.0.1:8080 --only-hoc; then
      echo "  m5-hoc-dashboard.png updated from browser"
    else
      echo "[WARN] HOC browser capture failed — metrics PNG retained"
    fi
  else
    echo "[WARN] HOC frontend build failed"
  fi
fi

echo ""
echo "[PASS] P1-1 README assets:"
ls -la "${ROOT}/docs/assets/m3-dual-source.gif" \
  "${ROOT}/docs/assets/m2-iiwa-pybullet.gif" \
  "${ROOT}/docs/assets/m2-iiwa-rviz.gif" \
  "${ROOT}/docs/assets/m5-hoc-dashboard.png" 2>/dev/null || true
