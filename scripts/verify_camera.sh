#!/usr/bin/env bash
# Verify PyBullet camera → ROS → HOC HTTP preview chain.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=verify_env.sh
source "${ROOT}/scripts/verify_env.sh"
verify_env_init "${ROOT}"

pkill -f bridge_node 2>/dev/null || true
pkill -f hoc_server 2>/dev/null || true
sleep 1

echo "==> Start bridge (planar_2dof + camera)"
ros2 run pybullet_bridge bridge_node --ros-args \
  -p robot_profile:=planar_2dof \
  -p enable_dual_source:=false \
  -p enable_camera:=true &
BRIDGE_PID=$!
sleep 4

if ! ros2 topic list | grep -q '/bridge/camera/image_compressed'; then
  kill "$BRIDGE_PID" 2>/dev/null || true
  echo "[FAIL] /bridge/camera/image_compressed not published"
  exit 1
fi

echo "==> Start hoc_server (camera HTTP :8766)"
ros2 run hoc_console hoc_server --ros-args -p serve_frontend:=false &
HOC_PID=$!
sleep 4

if ! curl -sf --max-time 3 http://127.0.0.1:8766/hoc/camera/latest.jpg -o /tmp/hoc_cam.jpg; then
  kill "$BRIDGE_PID" "$HOC_PID" 2>/dev/null || true
  echo "[FAIL] http://127.0.0.1:8766/hoc/camera/latest.jpg unavailable"
  echo "       Install: python3 -m pip install pillow websockets aiohttp"
  exit 1
fi

BYTES=$(wc -c < /tmp/hoc_cam.jpg)
if [ "$BYTES" -lt 500 ]; then
  kill "$BRIDGE_PID" "$HOC_PID" 2>/dev/null || true
  echo "[FAIL] camera JPEG too small (${BYTES} bytes)"
  exit 1
fi

kill "$BRIDGE_PID" "$HOC_PID" 2>/dev/null || true
echo "[PASS] Camera chain OK (${BYTES} bytes JPEG at :8766/hoc/camera/latest.jpg)"
