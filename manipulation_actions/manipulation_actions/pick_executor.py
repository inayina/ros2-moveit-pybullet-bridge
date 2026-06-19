"""Pick action execution logic."""

from __future__ import annotations

from typing import Protocol

from bridge_monitor_msgs.action import Pick
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

from manipulation_actions.gripper_stub import GripperStub
from manipulation_actions.pose_utils import copy_with_z_offset, offset_along_approach


class MotionPlanner(Protocol):
    def move_to_pose(
        self,
        pose: PoseStamped,
        *,
        planning_group: str,
        end_effector_link: str,
        timeout_sec: float,
    ) -> tuple[bool, JointTrajectory | None, str]:
        ...

    def cancel_all(self) -> None:
        ...


class PickExecutor:
    """Run approach → grasp → lift phases for Pick.action."""

    def __init__(
        self,
        *,
        default_planning_group: str = 'manipulator',
        default_end_effector_link: str = 'lbr_iiwa_link_7',
        default_pre_grasp_offset_m: float = 0.10,
        default_lift_offset_m: float = 0.05,
        gripper: GripperStub | None = None,
    ) -> None:
        self._default_planning_group = default_planning_group
        self._default_end_effector_link = default_end_effector_link
        self._default_pre_grasp_offset_m = default_pre_grasp_offset_m
        self._default_lift_offset_m = default_lift_offset_m
        self._gripper = gripper

    def execute(
        self,
        goal: Pick.Goal,
        goal_handle,
        planner: MotionPlanner,
    ) -> Pick.Result:
        result = Pick.Result()

        planning_group = goal.planning_group or self._default_planning_group
        ee_link = goal.end_effector_link or self._default_end_effector_link
        pre_offset = goal.pre_grasp_offset_m or self._default_pre_grasp_offset_m
        timeout_sec = goal.grasp_timeout_sec if goal.grasp_timeout_sec > 0 else 30.0

        pre_grasp = offset_along_approach(goal.grasp_pose, -pre_offset)
        lift_pose = copy_with_z_offset(goal.grasp_pose, self._default_lift_offset_m)

        phases: list[tuple[str, PoseStamped, float]] = [
            ('approach', pre_grasp, 0.3),
            ('grasp', goal.grasp_pose, 0.6),
            ('lift', lift_pose, 0.9),
        ]

        last_trajectory: JointTrajectory | None = None
        for phase_name, target_pose, progress in phases:
            if goal_handle.is_cancel_requested:
                planner.cancel_all()
                goal_handle.canceled()
                result.success = False
                result.message = f'Pick canceled during {phase_name}'
                return result

            feedback = Pick.Feedback()
            feedback.phase = phase_name
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)

            ok, trajectory, message = planner.move_to_pose(
                target_pose,
                planning_group=planning_group,
                end_effector_link=ee_link,
                timeout_sec=timeout_sec,
            )
            if not ok:
                result.success = False
                result.message = f'{phase_name}: {message}'
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return result
            if trajectory is not None:
                last_trajectory = trajectory

        if self._gripper is not None:
            self._gripper.close()

        result.success = True
        result.message = 'Pick completed'
        result.executed_trajectory = last_trajectory or JointTrajectory()
        goal_handle.succeed()
        return result
