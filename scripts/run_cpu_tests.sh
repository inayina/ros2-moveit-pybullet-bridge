#!/usr/bin/env bash
# Reproducible CPU-only tests for downstream Panda mainline (no colcon / ROS required).
# Resolves ament_index_python gaps via source-tree URDF fallback in robot_profiles.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PYTHON="${PYTHON:-python3}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="${ROOT}/pybullet_bridge:${ROOT}/dist_monitor:${ROOT}/risk_engine:${ROOT}/hoc_console:${PYTHONPATH:-}"

cd "${ROOT}"

echo "==> Downstream CPU test env"
echo "    ROOT=${ROOT}"
echo "    PYTHON=${PYTHON}"
"${PYTHON}" - <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, str(Path("pybullet_bridge").resolve()))
from pybullet_bridge.robot_profiles import resolve_urdf_path, get_profile
p = get_profile("panda")
assert p.lineage == "panda_mainline"
path = resolve_urdf_path("panda")
print(f"    panda URDF: {path}")
assert Path(path).is_file(), path
legacy = get_profile("iiwa7")
assert legacy.lineage == "legacy_kuka"
print("    legacy iiwa7 lineage=legacy_kuka (isolated)")
PY

CPU_TESTS=(
  test/test_panda_handoff.py
  test/test_panda_action_adapter.py
  test/test_jsonl_action_replay_policy.py
  test/test_learning_policies.py
  test/test_robot_profiles.py
  test/test_policy_command_replay.py
)

echo "==> CPU pytest (Panda mainline; skips ROS launch / ament-only)"
cd "${ROOT}/pybullet_bridge"
"${PYTHON}" -m pytest "${CPU_TESTS[@]}" -q \
  --ignore=test/test_m1_launch.py \
  --ignore=test/test_full_system_launch.py \
  --ignore=test/test_launch_imports.py \
  --ignore=test/test_sensor_fusion.py

echo "[PASS] Downstream CPU tests"
