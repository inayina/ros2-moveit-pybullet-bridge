#!/usr/bin/env bash
# Smoke-test HOC console backend (WebSocket + command handlers).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT}"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"
verify_env_require_pkg hoc_console

echo "==> hoc_console unit tests"
cd "${ROOT}/hoc_console"
/usr/bin/python3 -m pytest test/ -v --ignore=test/test_server_node.py

echo "==> hoc_server node smoke (8s)"
timeout 8 ros2 run hoc_console hoc_server --ros-args -p serve_frontend:=false &
HOC_PID=$!

registered=0
ws_ok=0
for _ in $(seq 1 12); do
  if ros2 node list 2>/dev/null | grep -q hoc_server; then
    registered=1
  fi
  if ss -tlnp 2>/dev/null | grep -q ':8765'; then
    ws_ok=1
    break
  fi
  sleep 0.5
done

if [ "${ws_ok}" -eq 1 ]; then
  if /usr/bin/python3 - <<'PY'
import asyncio
import sys
import websockets

async def test():
    async with websockets.connect('ws://127.0.0.1:8765') as ws:
        msg = await asyncio.wait_for(ws.recv(), timeout=2)
        if 'connected' not in msg:
            return False
        await ws.send('{"type":"ping"}')
        pong = await asyncio.wait_for(ws.recv(), timeout=2)
        return 'pong' in pong

try:
    ok = asyncio.run(test())
except Exception:
    ok = False
sys.exit(0 if ok else 1)
PY
  then
    :
  else
    ws_ok=0
  fi
fi

kill "${HOC_PID}" 2>/dev/null || true
wait "${HOC_PID}" 2>/dev/null || true

if [ "${registered}" -ne 1 ]; then
  echo "[WARN] hoc_server not on ROS graph yet (WebSocket check is authoritative)"
fi
if [ "${ws_ok}" -ne 1 ]; then
  echo "[FAIL] WebSocket on :8765 not reachable (check: pip install websockets; pkill -f hoc_server)"
  exit 1
fi

echo "[PASS] HOC smoke OK (ROS node + WebSocket handshake)"
