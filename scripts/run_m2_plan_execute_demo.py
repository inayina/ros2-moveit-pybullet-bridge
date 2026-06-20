#!/usr/bin/env python3
"""Automated Plan & Execute for m2_iiwa_demo (MoveGroup / joint fallback)."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node

from manipulation_actions.move_group_client import MoveGroupClient
from pybullet_bridge.robot_profiles import IIWA_HOME, IIWA_JOINTS


def _pose(x: float, y: float, z: float, *, frame: str = 'lbr_iiwa_link_0') -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose = Pose(
        position=Point(x=x, y=y, z=z),
        orientation=Quaternion(x=0.0, y=0.7071068, z=0.0, w=0.7071068),
    )
    return msg


def _joint_goal(positions: list[float]) -> None:
    """Fallback: FollowJointTrajectory via ros2 CLI (no extra deps)."""
    import subprocess

    names = ','.join(IIWA_JOINTS[: len(positions)])
    pos = ','.join(str(float(v)) for v in positions)
    cmd = (
        f"ros2 action send_goal /arm_controller/follow_joint_trajectory "
        f"control_msgs/action/FollowJointTrajectory "
        f"\"{{trajectory: {{joint_names: [{names}], "
        f"points: [{{positions: [{pos}], time_from_start: {{sec: 4}}}}]}}}}\" "
        f"--feedback"
    )
    subprocess.run(['bash', '-lc', cmd], check=False, timeout=30)


class M2PlanExecuteDemo(Node):
    def __init__(self) -> None:
        super().__init__('m2_plan_execute_demo')
        self._mg = MoveGroupClient(self, action_name='/move_action')

    def run(self) -> int:
        if not self._mg.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('MoveGroup /move_action not available')
            return 1

        poses = [
            ('reach_a', _pose(0.42, 0.0, 0.38)),
            ('reach_b', _pose(0.40, 0.18, 0.42)),
            ('reach_c', _pose(0.36, -0.12, 0.36)),
        ]
        ok_any = False
        for label, pose in poses:
            pose.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info(f'MoveGroup plan & execute: {label}')
            ok, _, msg = self._mg.move_to_pose(
                pose,
                planning_group='manipulator',
                end_effector_link='lbr_iiwa_link_7',
                timeout_sec=45.0,
            )
            self.get_logger().info(f'  {label}: ok={ok} — {msg}')
            ok_any = ok_any or ok
            time.sleep(0.8)

        if not ok_any:
            self.get_logger().warn('MoveGroup failed — joint sweep fallback')
            home = list(IIWA_HOME)
            for i in range(3):
                joints = home[:]
                joints[1] += 0.25 * math.sin(i * 1.2)
                joints[3] += 0.2 * math.cos(i * 0.9)
                joints[5] += 0.15 * math.sin(i * 1.5)
                _joint_goal(joints)
                time.sleep(4.5)
            ok_any = True

        return 0 if ok_any else 2


def main() -> int:
    rclpy.init()
    node = M2PlanExecuteDemo()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
