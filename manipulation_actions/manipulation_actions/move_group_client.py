"""MoveIt2 MoveGroup action client for pose targets."""

from __future__ import annotations

import copy
from typing import TYPE_CHECKING

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory

if TYPE_CHECKING:
    from rclpy.node import Node


def build_pose_constraints(
    pose: PoseStamped,
    link_name: str,
    *,
    position_tolerance: float = 0.01,
    orientation_tolerance: float = 0.1,
) -> Constraints:
    """Build MoveIt pose goal constraints for a single end-effector link."""
    constraints = Constraints()

    pos = PositionConstraint()
    pos.header = pose.header
    pos.link_name = link_name
    region = SolidPrimitive()
    region.type = SolidPrimitive.SPHERE
    region.dimensions = [position_tolerance]
    pos.constraint_region.primitives = [region]
    pos.constraint_region.primitive_poses = [copy.deepcopy(pose.pose)]
    pos.weight = 1.0
    constraints.position_constraints = [pos]

    orient = OrientationConstraint()
    orient.header = pose.header
    orient.link_name = link_name
    orient.orientation = copy.deepcopy(pose.pose.orientation)
    orient.absolute_x_axis_tolerance = orientation_tolerance
    orient.absolute_y_axis_tolerance = orientation_tolerance
    orient.absolute_z_axis_tolerance = orientation_tolerance
    orient.weight = 1.0
    constraints.orientation_constraints = [orient]
    return constraints


class MoveGroupClient:
    """Thin wrapper around /move_action for plan-and-execute to a pose."""

    def __init__(
        self,
        node: Node,
        *,
        action_name: str = '/move_action',
        planning_time_sec: float = 5.0,
        num_planning_attempts: int = 5,
    ) -> None:
        self._node = node
        self._client = ActionClient(node, MoveGroup, action_name)
        self._planning_time_sec = planning_time_sec
        self._num_planning_attempts = num_planning_attempts
        self._active_goal_handle = None

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def move_to_pose(
        self,
        pose: PoseStamped,
        *,
        planning_group: str,
        end_effector_link: str,
        timeout_sec: float = 30.0,
    ) -> tuple[bool, JointTrajectory | None, str]:
        if not self._client.server_is_ready():
            if not self.wait_for_server(timeout_sec=min(timeout_sec, 5.0)):
                return False, None, f'MoveGroup action server not available: {self._client._action_name}'

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = planning_group
        goal.request.num_planning_attempts = self._num_planning_attempts
        goal.request.allowed_planning_time = self._planning_time_sec
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.goal_constraints = [
            build_pose_constraints(pose, end_effector_link),
        ]

        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=timeout_sec)
        if not send_future.done():
            return False, None, 'Timed out sending MoveGroup goal'

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, None, 'MoveGroup goal rejected'

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=timeout_sec)
        self._active_goal_handle = None
        if not result_future.done():
            goal_handle.cancel_goal_async()
            return False, None, 'Timed out waiting for MoveGroup result'

        action_result = result_future.result().result
        if action_result.error_code.val != MoveItErrorCodes.SUCCESS:
            return (
                False,
                None,
                f'MoveGroup failed: error_code={action_result.error_code.val}',
            )

        trajectory = action_result.planned_trajectory.joint_trajectory
        if not trajectory.joint_names:
            trajectory = action_result.executed_trajectory.joint_trajectory
        return True, trajectory, 'MoveGroup succeeded'

    def cancel_all(self) -> None:
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
            self._active_goal_handle = None
        if self._client.server_is_ready():
            self._client.cancel_all_goals_async()
