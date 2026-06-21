#!/usr/bin/env bash
# One-shot Policy Runner system validation: replay + sine_wave benchmarks + HTML report.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-${HOME}/ros2_ws}"
OUTPUT_DIR="${BRIDGE_ROOT}/docs/samples/system-validation"
EPISODES="${VALIDATION_EPISODES:-10}"
DURATION_SEC="${VALIDATION_DURATION_SEC:-3}"
SEED="${VALIDATION_SEED:-0}"
INFERENCE_FREQ="${VALIDATION_INFERENCE_FREQ:-20}"
START_TS="$(date +%s)"

section() { printf '\n========== %s ==========\n' "$*"; }

cleanup_ros() {
  pkill -f "ros2 launch pybullet_bridge test_monitoring" 2>/dev/null || true
  pkill -f "ros2 run pybullet_bridge policy_runner" 2>/dev/null || true
  pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
  pkill -f "/dist_monitor/monitor_node" 2>/dev/null || true
  pkill -f "/risk_engine/risk_node" 2>/dev/null || true
  sleep 1
}

trap cleanup_ros EXIT

section "1/5 Resolve dataset"
mkdir -p "${OUTPUT_DIR}/replay" "${OUTPUT_DIR}/sine_wave" "${OUTPUT_DIR}/ros_logs"
# shellcheck source=integration_paths.sh
source "${SCRIPT_DIR}/integration_paths.sh"
resolve_integration_paths

REPLAY_FIXTURE="${BRIDGE_ROOT}/pybullet_bridge/test/fixtures/planar_2dof_replay.pkl"
if [ ! -f "${REPLAY_FIXTURE}" ]; then
  python3 - <<PY
from pathlib import Path
from pybullet_bridge.learning.benchmark_fixtures import write_planar_replay_fixture
write_planar_replay_fixture(Path("${REPLAY_FIXTURE}"))
print("generated replay fixture:", "${REPLAY_FIXTURE}")
PY
fi

DATASET_INFO="${OUTPUT_DIR}/dataset_info.json"
python3 - <<PY
import json
from pathlib import Path
payload = {
    "episode_data_lab_root": "${EPISODE_DATA_LAB_ROOT:-}",
    "lerobot_export": "${LEROBOT_EXPORT:-}",
    "replay_fixture": "${REPLAY_FIXTURE}",
    "episode_data_lab_present": bool("${EPISODE_DATA_LAB_ROOT:-}" and Path("${EPISODE_DATA_LAB_ROOT:-}").is_dir()),
    "lerobot_export_present": bool("${LEROBOT_EXPORT:-}" and Path("${LEROBOT_EXPORT:-}").is_dir()),
}
Path("${DATASET_INFO}").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
print(json.dumps(payload, indent=2))
PY

section "2/5 Prepare environment"
# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${BRIDGE_ROOT}"
verify_env_require_pkg pybullet_bridge
verify_env_require_pkg dist_monitor
verify_env_require_pkg bridge_monitor_msgs

if ! python3 -c "import psutil" 2>/dev/null; then
  echo "[INFO] Installing psutil for benchmark resource sampling"
  python3 -m pip install psutil --break-system-packages -q \
    || python3 -m pip install psutil --user -q
fi

mkdir -p "${OUTPUT_DIR}/replay" "${OUTPUT_DIR}/sine_wave" "${OUTPUT_DIR}/ros_logs"
export ROS_LOG_DIR="${OUTPUT_DIR}/ros_logs"

cleanup_ros

section "3/5 Benchmark ReplayPolicy"
python3 "${SCRIPT_DIR}/benchmark_system.py" \
  --strategy replay \
  --episodes "${EPISODES}" \
  --duration-sec "${DURATION_SEC}" \
  --output-dir "${OUTPUT_DIR}/replay" \
  --replay-path "${REPLAY_FIXTURE}" \
  --seed "${SEED}" \
  --inference-freq "${INFERENCE_FREQ}" \
  --launch-stack \
  | tee "${OUTPUT_DIR}/ros_logs/replay_benchmark.log"

section "4/5 Benchmark SineWavePolicy"
python3 "${SCRIPT_DIR}/benchmark_system.py" \
  --strategy sine_wave \
  --episodes "${EPISODES}" \
  --duration-sec "${DURATION_SEC}" \
  --output-dir "${OUTPUT_DIR}/sine_wave" \
  --seed "${SEED}" \
  --inference-freq "${INFERENCE_FREQ}" \
  --launch-stack \
  | tee "${OUTPUT_DIR}/ros_logs/sine_wave_benchmark.log"

section "5/5 Generate validation report"
python3 "${SCRIPT_DIR}/generate_system_validation_report.py" \
  --output-dir "${OUTPUT_DIR}" \
  --replay-summary "${OUTPUT_DIR}/replay/benchmark_summary.json" \
  --sine-summary "${OUTPUT_DIR}/sine_wave/benchmark_summary.json" \
  --dataset-info "${DATASET_INFO}"

python3 "${SCRIPT_DIR}/check_policy_runner_benchmark.py" "${OUTPUT_DIR}/replay/benchmark_summary.json"
python3 "${SCRIPT_DIR}/check_policy_runner_benchmark.py" "${OUTPUT_DIR}/sine_wave/benchmark_summary.json"

ELAPSED="$(( $(date +%s) - START_TS ))"
section "Done in ${ELAPSED}s"
echo "Report: ${OUTPUT_DIR}/validation_report.html"
echo "Summary: ${OUTPUT_DIR}/validation_summary.json"

if [ "${ELAPSED}" -gt 300 ]; then
  echo "[WARN] Validation exceeded 5 minute budget (${ELAPSED}s)" >&2
fi
