#!/usr/bin/env python3
"""Automated Plan & Execute for m2_iiwa_demo (MoveGroup / joint fallback)."""

from __future__ import annotations

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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


class M2PlanExecuteDemo(Node):
    def __init__(self) -> None:
        super().__init__('m2_plan_execute_demo')
        self._mg = MoveGroupClient(self, action_name='/move_action')
        self._bridge_pub = self.create_publisher(JointTrajectory, '/bridge/command', 10)

    def _publish_visible_joint_motion(self) -> None:
        """Fallback for recording: drive bridge directly with a smooth multi-point trajectory."""
        home = list(IIWA_HOME)
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(IIWA_JOINTS)

        for idx in range(80):
            phase = idx / 79.0
            point = JointTrajectoryPoint()
            point.positions = home[:]
            point.positions[1] += 0.35 * math.sin(2.0 * math.pi * phase)
            point.positions[3] += 0.28 * math.sin(4.0 * math.pi * phase + 0.4)
            point.positions[5] += 0.20 * math.sin(3.0 * math.pi * phase)
            point.time_from_start.sec = int(8.0 * phase)
            point.time_from_start.nanosec = int((8.0 * phase - point.time_from_start.sec) * 1e9)
            traj.points.append(point)

        deadline = time.time() + 2.0
        while self._bridge_pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Publishing direct /bridge/command fallback trajectory for recording')
        self._bridge_pub.publish(traj)

    def run(self) -> int:
        if not self._mg.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('MoveGroup /move_action not available')
            return 1

        home = list(IIWA_HOME)
        joint_goals = [
            ('joint_a', [home[0] + 0.55, home[1] - 0.25, home[2] + 0.45, home[3] + 0.38, home[4] + 0.25, home[5] - 0.30, home[6] + 0.35]),
            ('joint_b', [home[0] - 0.55, home[1] + 0.25, home[2] - 0.45, home[3] - 0.35, home[4] - 0.25, home[5] + 0.25, home[6] - 0.35]),
            ('joint_c', [home[0] + 0.35, home[1] - 0.15, home[2] + 0.55, home[3] + 0.25, home[4] - 0.45, home[5] - 0.20, home[6] + 0.50]),
        ]
        ok_any = False
        for label, positions in joint_goals:
            self.get_logger().info(f'MoveGroup joint plan & execute: {label}')
            ok, _, msg = self._mg.move_to_joint_positions(
                list(IIWA_JOINTS),
                positions,
                planning_group='manipulator',
                timeout_sec=45.0,
            )
            self.get_logger().info(f'  {label}: ok={ok} — {msg}')
            ok_any = ok_any or ok
            time.sleep(0.8)

        if not ok_any:
            if os.environ.get('ALLOW_DIRECT_FALLBACK', '').lower() == 'true':
                self.get_logger().warn('MoveGroup failed — direct bridge trajectory fallback')
                self._publish_visible_joint_motion()
                time.sleep(8.5)
                ok_any = True
            else:
                self.get_logger().error('MoveGroup failed and direct fallback is disabled')

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
