#!/usr/bin/env bash
# Verify NFR-M / NFR-REP maintainability and reproducibility evidence.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT="${1:-${ROOT}/docs/samples/maintainability-nfr-metrics.json}"
COVERAGE_DIR="${2:-docs/samples/maintainability-coverage}"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"

echo "==> Preflight: maintainability packages"
for pkg in bridge_monitor_msgs pybullet_bridge dist_monitor risk_engine hoc_console manipulation_actions moveit_config; do
  verify_env_require_pkg "${pkg}"
done

echo "==> Measure NFR-M / NFR-REP maintainability"
python3 "${SCRIPT_DIR}/check_maintainability_nfr.py" \
  --root "${ROOT}" \
  --output "${OUTPUT}" \
  --coverage-dir "${COVERAGE_DIR}"

echo "[PASS] NFR-M/NFR-REP verification complete: ${OUTPUT}"
