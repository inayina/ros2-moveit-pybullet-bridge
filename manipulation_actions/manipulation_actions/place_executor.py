"""Place action execution logic."""

from __future__ import annotations

from typing import Protocol

from bridge_monitor_msgs.action import Place
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

from manipulation_actions.gripper_stub import GripperStub
from manipulation_actions.pose_utils import offset_along_approach


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


class PlaceExecutor:
    """Run approach → release → retreat phases for Place.action."""

    def __init__(
        self,
        *,
        default_planning_group: str = 'manipulator',
        default_end_effector_link: str = 'lbr_iiwa_link_7',
        default_retreat_offset_m: float = 0.08,
        gripper: GripperStub | None = None,
    ) -> None:
        self._default_planning_group = default_planning_group
        self._default_end_effector_link = default_end_effector_link
        self._default_retreat_offset_m = default_retreat_offset_m
        self._gripper = gripper

    def execute(
        self,
        goal: Place.Goal,
        goal_handle,
        planner: MotionPlanner,
        *,
        end_effector_link: str | None = None,
        timeout_sec: float = 30.0,
    ) -> Place.Result:
        result = Place.Result()

        planning_group = goal.planning_group or self._default_planning_group
        ee_link = end_effector_link or self._default_end_effector_link
        retreat = goal.retreat_offset_m or self._default_retreat_offset_m

        phases: list[tuple[str, PoseStamped, float]] = [
            ('approach', offset_along_approach(goal.place_pose, -retreat), 0.3),
            ('release', goal.place_pose, 0.6),
            ('retreat', offset_along_approach(goal.place_pose, retreat), 0.9),
        ]

        for phase_name, target_pose, progress in phases:
            if goal_handle.is_cancel_requested:
                planner.cancel_all()
                goal_handle.canceled()
                result.success = False
                result.message = f'Place canceled during {phase_name}'
                return result

            feedback = Place.Feedback()
            feedback.phase = phase_name
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)

            ok, _trajectory, message = planner.move_to_pose(
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

            if phase_name == 'release' and self._gripper is not None:
                self._gripper.open()

        result.success = True
        result.message = 'Place completed'
        goal_handle.succeed()
        return result
