#!/usr/bin/env bash
# Smoke test: manipulation Pick action with bridge fallback (no MoveIt required).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# shellcheck source=verify_env.sh
source "${ROOT}/scripts/verify_env.sh"
verify_env_init "${ROOT}"

echo "==> 1/4 Environment"
verify_env_require_pkg pybullet_bridge
verify_env_require_pkg manipulation_actions

echo "==> 2/4 Clean stale processes"
pkill -f "ros2 launch pybullet_bridge portfolio_demo" 2>/dev/null || true
pkill -f "/manipulation_actions/manipulation_node" 2>/dev/null || true
pkill -f "/pybullet_bridge/bridge_node" 2>/dev/null || true
sleep 1

echo "==> 3/4 Launch portfolio_demo (DIRECT, 20s)"
timeout 20 ros2 launch pybullet_bridge portfolio_demo.launch.py \
  sim_mode:=DIRECT real_source:=topic 2>&1 | tail -8 &
LAUNCH_PID=$!
cleanup() { kill "$LAUNCH_PID" 2>/dev/null || true; wait "$LAUNCH_PID" 2>/dev/null || true; }
trap cleanup EXIT

sleep 8

echo "==> 4/4 Send Pick goal (bridge fallback)"
if ! ros2 action list 2>/dev/null | grep -q '/manipulation/pick'; then
  echo "[FAIL] /manipulation/pick action server not available" >&2
  exit 1
fi

/usr/bin/python3 - <<'PY'
import sys
import rclpy
from bridge_monitor_msgs.action import Pick
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.action import ActionClient
from rclpy.node import Node


class PickSmoke(Node):
    def __init__(self):
        super().__init__('pick_smoke')
        self._client = ActionClient(self, Pick, '/manipulation/pick')

    def run(self) -> int:
        if not self._client.wait_for_server(timeout_sec=10.0):
            print('[FAIL] Pick action server timeout', file=sys.stderr)
            return 1

        goal = Pick.Goal()
        goal.grasp_pose = PoseStamped()
        goal.grasp_pose.header.frame_id = 'lbr_iiwa_link_0'
        goal.grasp_pose.pose = Pose(
            position=Point(x=0.45, y=0.0, z=0.35),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        goal.planning_group = 'manipulator'
        goal.end_effector_link = 'lbr_iiwa_link_7'
        goal.pre_grasp_offset_m = 0.08
        goal.grasp_timeout_sec = 25.0

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print('[FAIL] Pick goal rejected', file=sys.stderr)
            return 1

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        if not result_future.done():
            print('[FAIL] Pick result timeout', file=sys.stderr)
            return 1

        result = result_future.result().result
        if not result.success:
            print(f'[FAIL] Pick failed: {result.message}', file=sys.stderr)
            return 1

        print(f'[PASS] Pick completed: {result.message}')
        return 0


rclpy.init()
node = PickSmoke()
try:
    raise SystemExit(node.run())
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

echo "[PASS] verify_pick.sh complete"
