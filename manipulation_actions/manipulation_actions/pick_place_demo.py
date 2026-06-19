#!/usr/bin/env python3
"""Send demo Pick or Place goals (requires manipulation_actions + bridge or move_group)."""

from __future__ import annotations

import sys

import rclpy
from bridge_monitor_msgs.action import Pick, Place
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.action import ActionClient
from rclpy.node import Node


class PickPlaceDemo(Node):
    def __init__(self) -> None:
        super().__init__('pick_place_demo')
        self.declare_parameter('mode', 'pick')
        self.declare_parameter('frame_id', 'lbr_iiwa_link_0')
        self._pick_client = ActionClient(self, Pick, '/manipulation/pick')
        self._place_client = ActionClient(self, Place, '/manipulation/place')

    def _grasp_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('frame_id').value
        pose.pose = Pose(
            position=Point(x=0.45, y=0.0, z=0.35),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
        )
        return pose

    def _place_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('frame_id').value
        pose.pose = Pose(
            position=Point(x=0.40, y=0.15, z=0.35),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
        )
        return pose

    def _send_goal(self, client: ActionClient, goal, action_name: str, timeout_sec: float) -> int:
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'{action_name} action server not available')
            return 1

        self.get_logger().info(f'Sending {action_name} goal...')
        send_future = client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'{action_name} goal rejected')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        result = result_future.result().result
        self.get_logger().info(
            f'{action_name} result: success={result.success} msg={result.message}')
        return 0 if result.success else 1

    def run(self) -> int:
        mode = self.get_parameter('mode').value
        if mode == 'pick':
            goal = Pick.Goal()
            goal.grasp_pose = self._grasp_pose()
            goal.end_effector_link = 'lbr_iiwa_link_7'
            goal.planning_group = 'manipulator'
            goal.pre_grasp_offset_m = 0.10
            goal.grasp_timeout_sec = 45.0
            return self._send_goal(self._pick_client, goal, 'Pick', 50.0)
        if mode == 'place':
            goal = Place.Goal()
            goal.place_pose = self._place_pose()
            goal.planning_group = 'manipulator'
            goal.retreat_offset_m = 0.10
            return self._send_goal(self._place_client, goal, 'Place', 60.0)

        self.get_logger().error(f'Unknown mode {mode!r}; use pick or place')
        return 1

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        phase = getattr(fb, 'phase', 'unknown')
        progress = getattr(fb, 'progress', 0.0)
        self.get_logger().info(f'Feedback: phase={phase} progress={progress:.2f}')


def main(args=None) -> int:
    rclpy.init(args=args)
    node = PickPlaceDemo()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
