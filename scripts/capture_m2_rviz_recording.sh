#!/usr/bin/env bash
# P1-1 bonus: RViz + PyBullet recording during automated Plan & Execute.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ASSETS="${ROOT}/docs/assets"
OUT_GIF="${ASSETS}/m2-iiwa-rviz.gif"
TMP_DIR="${ROOT}/docs/samples/.capture_tmp"
RECORD_MP4="${TMP_DIR}/m2_rviz_capture.mp4"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"

mkdir -p "${TMP_DIR}" "${ASSETS}"

if [ -z "${DISPLAY:-}" ]; then
  echo "[ERROR] DISPLAY not set — RViz recording requires a graphical session." >&2
  echo "        Run on a machine with X11/Wayland, or: export DISPLAY=:0" >&2
  exit 1
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "[ERROR] ffmpeg required for screen recording" >&2
  exit 1
fi

SIM_MODE="${SIM_MODE:-GUI}"
USE_RVIZ="${USE_RVIZ:-true}"
RECORD_SEC="${RECORD_SEC:-28}"
VIDEO_SIZE="${VIDEO_SIZE:-1400x900}"
CAPTURE_MODE="${CAPTURE_MODE:-auto}"  # auto|window|screen
WINDOW_PATTERN="${WINDOW_PATTERN:-RViz|rviz2|MoveIt}"
MIN_RECORD_BYTES="${MIN_RECORD_BYTES:-100000}"
TMP_GIF="${TMP_DIR}/m2-iiwa-rviz.gif"

window_geometry() {
  local window_id info x y width height

  if ! command -v xwininfo >/dev/null 2>&1; then
    return 1
  fi

  window_id="$(
    xwininfo -root -tree 2>/dev/null \
      | awk -v pattern="${WINDOW_PATTERN}" '{ line = tolower($0); pat = tolower(pattern); if (line ~ pat) { print $1; exit } }'
  )"
  if [ -z "${window_id}" ]; then
    return 1
  fi

  info="$(xwininfo -id "${window_id}" 2>/dev/null || true)"
  x="$(printf '%s\n' "${info}" | awk -F: '/Absolute upper-left X/ { gsub(/ /, "", $2); print $2; exit }')"
  y="$(printf '%s\n' "${info}" | awk -F: '/Absolute upper-left Y/ { gsub(/ /, "", $2); print $2; exit }')"
  width="$(printf '%s\n' "${info}" | awk -F: '/Width/ { gsub(/ /, "", $2); print $2; exit }')"
  height="$(printf '%s\n' "${info}" | awk -F: '/Height/ { gsub(/ /, "", $2); print $2; exit }')"

  if [ -z "${x}" ] || [ -z "${y}" ] || [ -z "${width}" ] || [ -z "${height}" ]; then
    return 1
  fi

  printf '%s %s %s %s %s\n' "${window_id}" "${x}" "${y}" "${width}" "${height}"
}

capture_target() {
  local geometry window_id x y width height

  if [ "${CAPTURE_MODE}" != screen ]; then
    for _ in $(seq 1 20); do
      if geometry="$(window_geometry)"; then
        read -r window_id x y width height <<<"${geometry}"
        echo "==> Capture window ${window_id} (${width}x${height}+${x},${y})" >&2
        printf 'window %s %s\n' "${window_id}" "${width}x${height}"
        return 0
      fi
      sleep 1
    done

    if [ "${CAPTURE_MODE}" = window ]; then
      echo "[ERROR] No RViz/MoveIt window matched WINDOW_PATTERN=${WINDOW_PATTERN}" >&2
      return 1
    fi

    echo "[WARN] RViz window not found; falling back to full DISPLAY capture" >&2
  fi

  printf 'screen %s %s\n' "${VIDEO_SIZE}" "${DISPLAY}"
}

echo "==> Clean stale launches"
pkill -f "ros2 launch moveit_config m2_iiwa_demo" 2>/dev/null || true
pkill -f "/moveit_ros_move_group/move_group" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
sleep 2

echo "==> Launch m2_iiwa_demo (sim_mode=${SIM_MODE}, use_rviz=${USE_RVIZ})"
ros2 launch moveit_config m2_iiwa_demo.launch.py \
  sim_mode:="${SIM_MODE}" \
  use_rviz:="${USE_RVIZ}" \
  >/tmp/capture_m2_rviz_launch.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "${LAUNCH_PID}" "${RECORD_PID:-}" 2>/dev/null || true
  pkill -f "ffmpeg.*x11grab" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "==> Wait for move_group + rviz"
for _ in $(seq 1 40); do
  if ros2 action list 2>/dev/null | grep -q '/move_action'; then
    break
  fi
  sleep 1
done
sleep 4

if ! ros2 action list 2>/dev/null | grep -q '/move_action'; then
  echo "[FAIL] /move_action not ready" >&2
  tail -30 /tmp/capture_m2_rviz_launch.log || true
  exit 1
fi

read -r CAPTURE_KIND CAPTURE_ARG1 CAPTURE_ARG2 < <(capture_target)

if [ "${CAPTURE_KIND}" = window ]; then
  echo "==> Screen record (${CAPTURE_ARG2}, ${RECORD_SEC}s, window_id=${CAPTURE_ARG1}) → ${RECORD_MP4}"
  ffmpeg -y -loglevel error \
    -f x11grab -framerate 12 -window_id "${CAPTURE_ARG1}" -i "${DISPLAY}" \
    -t "${RECORD_SEC}" "${RECORD_MP4}" &
else
  echo "==> Screen record (${CAPTURE_ARG1}, ${RECORD_SEC}s, input=${CAPTURE_ARG2}) → ${RECORD_MP4}"
  ffmpeg -y -loglevel error \
    -f x11grab -framerate 12 -video_size "${CAPTURE_ARG1}" -i "${CAPTURE_ARG2}" \
    -t "${RECORD_SEC}" "${RECORD_MP4}" &
fi
RECORD_PID=$!
sleep 1

echo "==> Automated Plan & Execute"
if ! python3 "${SCRIPT_DIR}/run_m2_plan_execute_demo.py"; then
  echo "[WARN] Plan & Execute reported issues (recording may still be usable)"
fi

wait "${RECORD_PID}" 2>/dev/null || true
RECORD_PID=""

if [ ! -s "${RECORD_MP4}" ]; then
  echo "[FAIL] Recording empty — check DISPLAY and window visibility" >&2
  exit 1
fi

RECORD_BYTES="$(stat -c '%s' "${RECORD_MP4}")"
if [ "${RECORD_BYTES}" -lt "${MIN_RECORD_BYTES}" ]; then
  echo "[FAIL] Recording too small (${RECORD_BYTES} bytes < ${MIN_RECORD_BYTES}) — likely blank/hidden window" >&2
  echo "       Move RViz onto DISPLAY=${DISPLAY}, or lower MIN_RECORD_BYTES if this is expected." >&2
  exit 1
fi

echo "==> Convert MP4 → GIF"
ffmpeg -y -loglevel error -i "${RECORD_MP4}" \
  -vf "fps=10,scale=840:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
  "${TMP_GIF}"

mv "${TMP_GIF}" "${OUT_GIF}"
ls -la "${OUT_GIF}"
echo "[PASS] RViz recording → ${OUT_GIF}"
