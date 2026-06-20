#!/usr/bin/env bash
# Full risk monitoring verification (P1–P3 + smoke checks).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${ROOT}"
echo "==> Phase S5a (D4 + R2 degraded)"
./scripts/verify_risk_d4.sh

echo "==> Phase S5b (D3 + soft limits)"
./scripts/verify_risk_d3.sh

echo "==> Phase S5c (D5 planning failure)"
./scripts/verify_risk_d5.sh

echo "==> CSV export unit smoke"
/usr/bin/python3 -m pytest hoc_console/test/test_report_csv.py -q

echo "==> Watchdog unit smoke"
/usr/bin/python3 -m pytest pybullet_bridge/test/test_watchdog.py -q

echo "[PASS] verify_risk_complete.sh — five-dimension risk stack verified"
