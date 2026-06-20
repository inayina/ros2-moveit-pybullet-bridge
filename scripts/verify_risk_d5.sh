#!/usr/bin/env bash
# Phase S5c: D5 planning failure stats smoke test.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck source=verify_env.sh
source "${SCRIPT_DIR}/verify_env.sh"
verify_env_init "${ROOT}"

echo "==> Cleaning stale ROS processes"
pkill -f "ros2 launch pybullet_bridge portfolio_demo" 2>/dev/null || true
pkill -f "/manipulation_actions/manipulation_node" 2>/dev/null || true
pkill -f "/risk_engine/risk_node" 2>/dev/null || true
sleep 1

echo "==> Launch portfolio_demo (DIRECT)"
setsid ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic >/tmp/verify_risk_d5.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -- "-${LAUNCH_PID}" 2>/dev/null || kill "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 12

echo "==> Send Pick goal then cancel (record planning failure)"
/usr/bin/python3 - <<'PY'
import sys
import time

import rclpy
from bridge_monitor_msgs.action import Pick
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.action import ActionClient
from rclpy.node import Node


class PickCancelSmoke(Node):
    def __init__(self):
        super().__init__('pick_cancel_smoke')
        self._client = ActionClient(self, Pick, '/manipulation/pick')

    def run(self) -> int:
        if not self._client.wait_for_server(timeout_sec=15.0):
            print('[FAIL] Pick action server timeout', file=sys.stderr)
            return 1

        goal = Pick.Goal()
        goal.grasp_pose = PoseStamped()
        goal.grasp_pose.header.frame_id = 'lbr_iiwa_link_0'
        goal.grasp_pose.pose = Pose(
            position=Point(x=0.45, y=0.0, z=0.35),
            orientation=Quaternion(w=1.0),
        )
        goal.grasp_timeout_sec = 30.0

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print('[FAIL] Pick goal rejected', file=sys.stderr)
            return 1

        time.sleep(0.5)
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
        return 0


rclpy.init()
node = PickCancelSmoke()
try:
    raise SystemExit(node.run())
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

sleep 3

echo "==> Check /risk/planning_stats"
STATS=$(timeout 8 ros2 topic echo /risk/planning_stats --once 2>/dev/null || true)
if ! echo "$STATS" | grep -q 'planning_stats'; then
  echo "[FAIL] /risk/planning_stats not publishing" >&2
  exit 1
fi
if ! echo "$STATS" | grep -q 'failure_rate'; then
  echo "[FAIL] failure_rate missing from planning_stats" >&2
  exit 1
fi
echo "  planning_stats topic OK"

echo "==> Check planning_failure in /risk/status attribution"
RISK=$(timeout 8 ros2 topic echo /risk/status --once 2>/dev/null || true)
if ! echo "$RISK" | grep -q 'planning_failure'; then
  echo "[FAIL] planning_failure missing from risk attribution" >&2
  exit 1
fi
echo "  planning_failure attribution OK"

cleanup
trap - EXIT
sleep 1

echo "[PASS] verify_risk_d5.sh complete (D5 planning failure stats)"
