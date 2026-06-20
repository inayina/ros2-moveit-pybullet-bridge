#!/usr/bin/env bash
# One-click local experiment: clean stale HOC ports, then launch sim + monitor + HOC UI.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"

echo "==> Stopping stale HOC / Vite processes"
pkill -f "/hoc_console/hoc_server" 2>/dev/null || true
pkill -f "vite --host" 2>/dev/null || true
sleep 1

if ss -tlnp 2>/dev/null | grep -qE ':8765|:5173'; then
  echo "[FAIL] Ports 8765/5173 still busy. Check:" >&2
  ss -tlnp 2>/dev/null | grep -E ':8765|:5173' || true
  exit 1
fi

echo "==> Launching portfolio_demo + HOC (DIRECT mode)"
echo "    Browser: http://localhost:5173"
echo "    Wait ~30s for monitor warmup before charts show non-zero KL/W1."
exec ros2 launch hoc_console hoc_experiment.launch.py sim_mode:=DIRECT "$@"
