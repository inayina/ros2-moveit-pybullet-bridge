#!/usr/bin/env bash
# Preflight checks for recording the portfolio / HOC demo video.
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
FRONTEND_DIR="${ROOT}/hoc_console/frontend"
TMP_DIR="${ROOT}/docs/samples/.recording_check"
TMP_MP4="${TMP_DIR}/screen_probe.mp4"

FAILS=0
WARNS=0

pass() {
  printf '[PASS] %s\n' "$1"
}

warn() {
  printf '[WARN] %s\n' "$1"
  WARNS=$((WARNS + 1))
}

fail() {
  printf '[FAIL] %s\n' "$1"
  FAILS=$((FAILS + 1))
}

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

check_cmd() {
  local cmd="$1"
  local hint="${2:-}"

  if have_cmd "${cmd}"; then
    pass "Command available: ${cmd}"
  else
    fail "Missing command: ${cmd}${hint:+ (${hint})}"
  fi
}

check_ros_pkg() {
  local pkg="$1"

  if ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    pass "ROS package available: ${pkg}"
  else
    fail "ROS package missing: ${pkg}; run colcon build from ~/ros2_ws"
  fi
}

load_ros_env() {
  export PATH="/usr/bin:/bin:/opt/ros/jazzy/bin:${PATH}"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV
  export ROS_LOG_DIR="${ROOT}/.ros_log"
  mkdir -p "${ROS_LOG_DIR}"

  if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    return 1
  fi

  # ROS setup scripts may read unset variables, so do not source them under nounset.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash

  local ws_root="${ROS2_WS_ROOT:-${HOME}/ros2_ws}"
  if [ -f "${ws_root}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${ws_root}/install/setup.bash"
  fi
  set -u
}

port_busy() {
  local port="$1"

  if have_cmd ss && ss -tln 2>/dev/null | awk '{ print $4 }' | grep -Eq "(:|\\])${port}$"; then
    return 0
  fi
  return 1
}

check_port_free() {
  local port="$1"
  local label="$2"

  if port_busy "${port}"; then
    warn "Port ${port} is already in use (${label}); stop stale demo processes before recording"
  else
    pass "Port ${port} is free (${label})"
  fi
}

json_bool() {
  local file="$1"
  local expr="$2"

  python3 - "$file" "$expr" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
expr = sys.argv[2]
try:
    data = json.loads(path.read_text())
except Exception:
    sys.exit(2)

value = data
for part in expr.split("."):
    if isinstance(value, dict) and part in value:
        value = value[part]
    else:
        sys.exit(2)

sys.exit(0 if bool(value) else 1)
PY
}

check_metric_bool() {
  local file="$1"
  local expr="$2"
  local label="$3"
  local severity="${4:-fail}"

  if [ ! -f "${file}" ]; then
    if [ "${severity}" = "warn" ]; then
      warn "Missing metric artifact: ${label} (${file#${ROOT}/})"
    else
      fail "Missing metric artifact: ${label} (${file#${ROOT}/})"
    fi
    return
  fi

  if json_bool "${file}" "${expr}"; then
    pass "${label}"
  else
    if [ "${severity}" = "warn" ]; then
      warn "${label} not passing; check ${file#${ROOT}/}"
    else
      fail "${label} not passing; check ${file#${ROOT}/}"
    fi
  fi
}

screen_geometry() {
  if have_cmd xwininfo && [ -n "${DISPLAY:-}" ]; then
    local info width height
    info="$(xwininfo -root 2>/dev/null || true)"
    width="$(printf '%s\n' "${info}" | awk -F: '/Width/ { gsub(/ /, "", $2); print $2; exit }')"
    height="$(printf '%s\n' "${info}" | awk -F: '/Height/ { gsub(/ /, "", $2); print $2; exit }')"
    if [ -n "${width}" ] && [ -n "${height}" ]; then
      printf '%sx%s\n' "${width}" "${height}"
      return 0
    fi
  fi

  printf '%s\n' "${VIDEO_SIZE:-1280x720}"
}

probe_screen_recording() {
  if [ -z "${DISPLAY:-}" ]; then
    fail "DISPLAY is not set; repository recording scripts use ffmpeg x11grab"
    return
  fi
  pass "DISPLAY is set: ${DISPLAY}"

  if [ -n "${WAYLAND_DISPLAY:-}" ]; then
    warn "WAYLAND_DISPLAY=${WAYLAND_DISPLAY}; x11grab may only work for XWayland-visible windows"
  fi

  if ! have_cmd ffmpeg; then
    fail "ffmpeg is required for screen recording"
    return
  fi

  mkdir -p "${TMP_DIR}"
  rm -f "${TMP_MP4}"

  local size
  size="$(screen_geometry)"
  printf '==> Probe screen recording: ffmpeg x11grab, DISPLAY=%s, size=%s, 2s\n' "${DISPLAY}" "${size}"
  if timeout 8 ffmpeg -y -loglevel error \
    -f x11grab -framerate 10 -video_size "${size}" -i "${DISPLAY}" \
    -t 2 "${TMP_MP4}" >/dev/null 2>&1; then
    if [ -s "${TMP_MP4}" ]; then
      local bytes
      bytes="$(stat -c '%s' "${TMP_MP4}")"
      pass "Screen recording probe produced ${bytes} bytes"
    else
      fail "Screen recording probe produced an empty file"
    fi
  else
    fail "ffmpeg x11grab probe failed; try an X11 session, export DISPLAY=:0, or use OBS/system recorder"
  fi
}

printf '=== Recording Readiness Check ===\n'
printf 'Repo: %s\n' "${ROOT}"
printf 'Time: %s\n\n' "$(date '+%Y-%m-%d %H:%M:%S %Z')"

printf '==> Environment\n'
if load_ros_env; then
  pass "ROS environment loaded"
else
  fail "ROS 2 Jazzy not found: /opt/ros/jazzy/setup.bash"
fi
check_cmd python3
check_cmd ros2
check_cmd ffmpeg "sudo apt install ffmpeg"
check_cmd xwininfo "sudo apt install x11-utils"
check_cmd npm "required by HOC frontend"
check_cmd node "required by HOC frontend"

printf '\n==> Screen Recording\n'
probe_screen_recording

printf '\n==> ROS Demo Packages\n'
for pkg in bridge_monitor_msgs pybullet_bridge dist_monitor risk_engine hoc_console manipulation_actions moveit_config; do
  check_ros_pkg "${pkg}"
done

printf '\n==> HOC Frontend\n'
if [ -d "${FRONTEND_DIR}" ]; then
  pass "Frontend directory exists: ${FRONTEND_DIR#${ROOT}/}"
  if [ -d "${FRONTEND_DIR}/node_modules" ]; then
    pass "Frontend node_modules exists"
  else
    warn "Frontend node_modules missing; run: cd hoc_console/frontend && npm install"
  fi
  if [ -f "${FRONTEND_DIR}/package.json" ]; then
    pass "Frontend package.json exists"
  else
    fail "Frontend package.json missing"
  fi
else
  fail "Frontend directory missing: ${FRONTEND_DIR#${ROOT}/}"
fi

printf '\n==> Demo Ports\n'
check_port_free 5173 "Vite dev UI"
check_port_free 8765 "HOC WebSocket"
check_port_free 8080 "HOC production UI"

printf '\n==> Current Verification Evidence\n'
check_metric_bool "${ROOT}/docs/samples/moveit-closure-metrics.json" "overall_passes" "MoveIt closure metrics pass"
check_metric_bool "${ROOT}/docs/samples/monitor-metrics.json" "overall_passes" "Distribution monitor metrics pass"
check_metric_bool "${ROOT}/docs/samples/risk-management-metrics.json" "overall_passes" "Risk management metrics pass"
check_metric_bool "${ROOT}/docs/samples/hoc-console-metrics.json" "overall_passes" "HOC console metrics pass"
check_metric_bool "${ROOT}/docs/samples/performance-nfr-metrics.json" "overall_passes" "Performance NFR metrics pass"
check_metric_bool "${ROOT}/docs/samples/safety-nfr-metrics.json" "overall_passes" "Safety NFR metrics pass"
check_metric_bool "${ROOT}/docs/samples/maintainability-nfr-metrics.json" "overall_passes" "Maintainability NFR metrics pass"
check_metric_bool "${ROOT}/docs/samples/bridge-comm-metrics.json" "overall_passes" "Bridge communication strict jitter check pass" warn

printf '\n==> Recommended Recording Commands\n'
printf 'Full HOC demo:\n'
printf '  source ~/ros2_ws/install/setup.bash\n'
printf '  ros2 launch hoc_console hoc_experiment.launch.py sim_mode:=GUI\n'
printf '  # Browser: http://localhost:5173\n\n'
printf 'RViz/MoveIt capture helper:\n'
printf '  ./scripts/capture_m2_rviz_recording.sh\n\n'

if [ "${FAILS}" -eq 0 ]; then
  if [ "${WARNS}" -eq 0 ]; then
    printf '[READY] Recording preflight passed with no warnings.\n'
  else
    printf '[READY_WITH_WARNINGS] Recording is possible, but review %s warning(s).\n' "${WARNS}"
  fi
  exit 0
fi

printf '[NOT_READY] %s failure(s), %s warning(s). Fix failures before recording.\n' "${FAILS}" "${WARNS}"
exit 1
