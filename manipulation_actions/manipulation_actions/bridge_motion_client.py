"""Fallback joint-space motion via /bridge/command when MoveIt is unavailable."""

from __future__ import annotations

import copy
import time
from typing import TYPE_CHECKING

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if TYPE_CHECKING:
    from rclpy.node import Node


class BridgeMotionClient:
    """Send interpolated joint trajectories to /bridge/command."""

    def __init__(
        self,
        node: Node,
        *,
        joint_names: list[str],
        home_positions: list[float],
        goal_tolerance: float = 0.03,
        motion_duration_sec: float = 3.0,
    ) -> None:
        self._node = node
        self._joint_names = list(joint_names)
        self._home = list(home_positions)
        self._goal_tolerance = goal_tolerance
        self._motion_duration_sec = motion_duration_sec
        self._positions: dict[str, float] = {}
        self._cmd_pub = node.create_publisher(JointTrajectory, '/bridge/command', 10)
        node.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            qos_profile_sensor_data,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._positions[name] = float(pos)

    def wait_for_joint_states(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if all(name in self._positions for name in self._joint_names):
                return True
            time.sleep(0.05)
        return False

    def move_to_pose(
        self,
        pose: PoseStamped,
        *,
        planning_group: str,
        end_effector_link: str,
        timeout_sec: float = 30.0,
    ) -> tuple[bool, JointTrajectory | None, str]:
        del planning_group, end_effector_link, timeout_sec
        if not self.wait_for_joint_states():
            return False, None, 'No /joint_states feedback for bridge motion'

        start = [
            self._positions.get(name, self._home[i] if i < len(self._home) else 0.0)
            for i, name in enumerate(self._joint_names)
        ]
        target = self._pose_to_joint_target(pose, start)
        trajectory = self._build_trajectory(start, target, self._motion_duration_sec)
        trajectory.header.stamp = self._node.get_clock().now().to_msg()
        self._cmd_pub.publish(trajectory)

        deadline = time.monotonic() + self._motion_duration_sec + 2.0
        while time.monotonic() < deadline:
            if self._at_target(target):
                return True, trajectory, 'Bridge joint motion succeeded'
            time.sleep(0.02)
        return False, trajectory, 'Bridge joint motion timed out'

    def _pose_to_joint_target(
        self,
        pose: PoseStamped,
        start: list[float],
    ) -> list[float]:
        """Heuristic joint nudge from pose position (fallback without IK)."""
        target = list(start)
        px = pose.pose.position.x
        py = pose.pose.position.y
        pz = pose.pose.position.z
        if len(target) >= 1:
            target[0] = max(-2.8, min(2.8, px * 0.8))
        if len(target) >= 2:
            target[1] = max(-2.0, min(2.0, 0.4 + pz * 0.5))
        if len(target) >= 4:
            target[3] = max(-2.8, min(0.2, -1.2 + py * 0.4))
        if len(target) >= 6:
            target[5] = max(0.5, min(2.5, 1.2 + pz * 0.3))
        return target

    def _build_trajectory(
        self,
        start: list[float],
        target: list[float],
        duration_sec: float,
        steps: int = 40,
    ) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = self._joint_names
        for i in range(steps + 1):
            alpha = i / steps
            point = JointTrajectoryPoint()
            point.positions = [
                s + alpha * (t - s) for s, t in zip(start, target)
            ]
            t = duration_sec * alpha
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1.0) * 1e9),
            )
            msg.points.append(point)
        return msg

    def _at_target(self, target: list[float]) -> bool:
        for name, goal in zip(self._joint_names, target):
            if name not in self._positions:
                return False
            if abs(self._positions[name] - goal) > self._goal_tolerance:
                return False
        return True

    def cancel_all(self) -> None:
        hold = JointTrajectory()
        hold.header.stamp = self._node.get_clock().now().to_msg()
        hold.joint_names = self._joint_names
        point = JointTrajectoryPoint()
        point.positions = [
            self._positions.get(name, self._home[i] if i < len(self._home) else 0.0)
            for i, name in enumerate(self._joint_names)
        ]
        hold.points = [point]
        self._cmd_pub.publish(hold)
