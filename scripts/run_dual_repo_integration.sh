#!/usr/bin/env bash
# Dual-repo integration: episode-data-lab + bridge, with report and chart artifacts.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS2_WS_ROOT="${ROS2_WS_ROOT:-${HOME}/ros2_ws}"
ARTIFACT_DIR="${BRIDGE_ROOT}/docs/samples"
VALIDATION_JSON="${ARTIFACT_DIR}/dual-repo-validation.json"
SELF_JSON="${ARTIFACT_DIR}/dual-repo-offline-self-metrics.json"
CROSS_JSON="${ARTIFACT_DIR}/dual-repo-cross-source-metrics.json"
SIM_NPZ="${ARTIFACT_DIR}/bridge-sim-trajectory.npz"
ONLINE_JSON="${ARTIFACT_DIR}/dual-repo-online-smoke.json"

# shellcheck source=integration_paths.sh
source "${SCRIPT_DIR}/integration_paths.sh"
resolve_integration_paths

export EPISODE_DATA_LAB_ROOT LEROBOT_EXPORT BRIDGE_ROOT ROS2_WS_ROOT

echo "==> Dual-repo integration"
echo "    EPISODE_DATA_LAB_ROOT=${EPISODE_DATA_LAB_ROOT:-<unset>}"
echo "    LEROBOT_EXPORT=${LEROBOT_EXPORT:-<unset>}"
echo "    BRIDGE_ROOT=${BRIDGE_ROOT}"

if [ -z "${EPISODE_DATA_LAB_ROOT:-}" ] || [ ! -d "${EPISODE_DATA_LAB_ROOT}" ]; then
  echo "[ERROR] episode-data-lab not found. Set EPISODE_DATA_LAB_ROOT." >&2
  exit 1
fi

mkdir -p "${ARTIFACT_DIR}"

# --- Step A: validate episode-data-lab dataset ---
echo "==> Step A: validate_dataset (episode-data-lab)"
VALIDATION_OK=false
if [ -f "${EPISODE_DATA_LAB_ROOT}/scripts/validate_dataset.py" ]; then
  if (
    cd "${EPISODE_DATA_LAB_ROOT}"
    python3 scripts/validate_dataset.py dataset/v1 2>&1 | tee "${ARTIFACT_DIR}/dual-repo-validation.log"
  ); then
    VALIDATION_OK=true
  else
    echo "[WARN] validate_dataset reported issues (LeRobot export may still be usable for bridge)"
  fi
else
  echo "[WARN] validate_dataset.py not found; skipping"
fi

python3 - <<PY
import json
from pathlib import Path
lerobot = Path("${LEROBOT_EXPORT}")
info = lerobot / "meta" / "info.json"
ok = "$( [ "${VALIDATION_OK}" = true ] && echo True || echo False )"
payload = {"ok": ok == "True", "dataset": "dataset/v1", "lerobot_export_exists": lerobot.is_dir()}
if info.is_file():
    payload["lerobot_meta"] = json.loads(info.read_text(encoding="utf-8"))
Path("${VALIDATION_JSON}").write_text(json.dumps(payload, indent=2), encoding="utf-8")
print("wrote ${VALIDATION_JSON}")
PY

# --- Step B: offline self-compare (sanity: same LeRobot dataset) ---
echo "==> Step B: offline_compare self-check (LeRobot vs LeRobot)"
if [ ! -d "${LEROBOT_EXPORT:-}" ]; then
  echo "[ERROR] LeRobot export missing: ${LEROBOT_EXPORT:-<unset>}" >&2
  echo "        Run in episode-data-lab: python3 scripts/export_lerobot_style.py dataset/v1 --output dataset/v1/lerobot_export" >&2
  exit 1
fi

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${BRIDGE_ROOT}"
verify_env_require_pkg dist_monitor

if ! ros2 pkg executables dist_monitor 2>/dev/null | grep -q offline_compare; then
  echo "==> colcon build dist_monitor (offline_compare missing)"
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  (cd "${ROS2_WS_ROOT}" && colcon build --packages-select dist_monitor --symlink-install)
  # shellcheck disable=SC1091
  source "${ROS2_WS_ROOT}/install/setup.bash"
fi

ros2 run dist_monitor offline_compare -- \
  --real-dataset "${LEROBOT_EXPORT}" \
  --sim-dataset "${LEROBOT_EXPORT}" \
  --min-samples 50 > "${SELF_JSON}"
echo "  wrote ${SELF_JSON}"

# --- Step C: online capture — bridge Sim trajectory + lerobot Real ---
echo "==> Step C: capture bridge Sim trajectory (real_source:=lerobot)"
pkill -f "ros2 launch pybullet_bridge portfolio_demo" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
pkill -f "sim_trajectory_recorder" 2>/dev/null || true
sleep 1

ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=lerobot \
  lerobot_dataset_path:="${LEROBOT_EXPORT}" \
  >/tmp/dual_repo_lerobot.log 2>&1 &
LAUNCH_PID=$!

CAPTURE_OK=false
ONLINE_OK=false
METRICS_SEEN=false
ONLINE_METRICS_JSON="null"

for _ in $(seq 1 20); do
  sleep 1
  if ros2 topic list 2>/dev/null | grep -q '/bridge/sim/joint_states'; then
    break
  fi
done

if python3 "${SCRIPT_DIR}/capture_sim_trajectory.py" \
    --duration 12 \
    --output "${SIM_NPZ}" \
    --min-samples 50; then
  CAPTURE_OK=true
fi

if timeout 5 ros2 topic echo /monitor/distribution_metrics --once > /tmp/dual_repo_metrics.yaml 2>/dev/null; then
  METRICS_SEEN=true
  ONLINE_OK=true
  ONLINE_METRICS_JSON="$(python3 - <<'PY'
import json, re, sys
from pathlib import Path
text = Path("/tmp/dual_repo_metrics.yaml").read_text(encoding="utf-8", errors="replace")
def grab_float(key):
    m = re.search(rf"^{key}:\s*([-\d.eE]+)", text, re.M)
    return float(m.group(1)) if m else None
def grab_bool(key):
    m = re.search(rf"^{key}:\s*(true|false)", text, re.M)
    return m.group(1) == "true" if m else None
payload = {
    "kl_divergence_mean": grab_float("kl_divergence_mean"),
    "wasserstein_mean": grab_float("wasserstein_mean"),
    "mmd_statistic": grab_float("mmd_statistic"),
    "shift_detected": grab_bool("shift_detected"),
    "sample_count_sim": grab_float("sample_count_sim"),
    "sample_count_real": grab_float("sample_count_real"),
}
print(json.dumps(payload))
PY
)"
fi

kill "${LAUNCH_PID}" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true
sleep 1

python3 - <<PY
import json
from pathlib import Path
metrics = json.loads("""${ONLINE_METRICS_JSON}""")
payload = {
    "ok": str("${ONLINE_OK}").lower() == "true",
    "capture_ok": str("${CAPTURE_OK}").lower() == "true",
    "metrics_seen": str("${METRICS_SEEN}").lower() == "true",
    "real_source": "lerobot",
    "lerobot_export": "${LEROBOT_EXPORT}",
    "sim_npz": "${SIM_NPZ}",
    "online_metrics": metrics,
    "log_tail": Path("/tmp/dual_repo_lerobot.log").read_text(encoding="utf-8", errors="replace").splitlines()[-12:],
}
Path("${ONLINE_JSON}").write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
print("wrote ${ONLINE_JSON}")
PY

if [ "${CAPTURE_OK}" != true ]; then
  echo "[FAIL] Sim trajectory capture failed" >&2
  tail -20 /tmp/dual_repo_lerobot.log || true
  exit 1
fi

# --- Step B2: cross-source offline_compare (bridge Sim NPZ vs LeRobot Real) ---
echo "==> Step B2: offline_compare cross-source (bridge Sim NPZ vs LeRobot Real)"
ros2 run dist_monitor offline_compare -- \
  --real-dataset "${LEROBOT_EXPORT}" \
  --sim-npz "${SIM_NPZ}" \
  --min-samples 50 \
  --mmd-permutations 100 > "${CROSS_JSON}"
echo "  wrote ${CROSS_JSON}"

if [ "${ONLINE_OK}" != true ]; then
  echo "[WARN] Online /monitor/distribution_metrics not captured"
  tail -20 /tmp/dual_repo_lerobot.log || true
fi

# --- Step D: generate report + charts ---
echo "==> Step D: align reports and charts"
python3 "${SCRIPT_DIR}/regenerate_all_reports.py"

echo ""
echo "[PASS] Dual-repo integration complete"
echo "  Experiment report: ${BRIDGE_ROOT}/docs/samples/dual-repo-experiment-report.html"
echo "  Same-task report:  ${BRIDGE_ROOT}/docs/samples/same-task-calibration-report.html"
echo "  Integration:       ${BRIDGE_ROOT}/docs/samples/dual-repo-integration-report.html"
echo "  Guide:             ${BRIDGE_ROOT}/docs/EXPERIMENTS.md"
echo "  Cross-source:      ${CROSS_JSON}"
echo "  Sim NPZ:           ${SIM_NPZ}"
echo "  Charts:            ${BRIDGE_ROOT}/docs/assets/dual-repo-*.png"
echo ""
echo "  Next: ./scripts/run_same_task_calibration.sh  (if not run yet)"
