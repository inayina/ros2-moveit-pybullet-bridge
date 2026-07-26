#!/usr/bin/env bash
set -euo pipefail

EVIDENCE_DIR="${1:-/tmp/policy_runtime_m6_wiring_$(date -u +%Y%m%dT%H%M%SZ)}"
ROS_LOG_DIR="${ROS_LOG_DIR:-${EVIDENCE_DIR}/ros_logs}"
export ROS_LOG_DIR
mkdir -p "${EVIDENCE_DIR}" "${ROS_LOG_DIR}"

cleanup() {
  pkill -9 -f '/lib/hoc_console/hoc_server' 2>/dev/null || true
  pkill -9 -f '/lib/hoc_console/m6_wiring_probe' 2>/dev/null || true
  pkill -9 -f '/lib/risk_engine/risk_to_safety_bridge' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

timeout 45s ros2 launch hoc_console policy_runtime_m6_wiring.launch.py \
  evidence_dir:="${EVIDENCE_DIR}"

python3 - "${EVIDENCE_DIR}/m6_wiring_smoke.json" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
if not path.is_file():
    raise SystemExit(f'M6 evidence missing: {path}')
report = json.loads(path.read_text(encoding='utf-8'))
if report.get('status') != 'PASS':
    raise SystemExit(f'M6 wiring failed: {report.get("error")}')
print(path)
PY
