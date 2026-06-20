#!/usr/bin/env python3
"""Verify MoveIt -> FollowJointTrajectory -> PyBullet closure evidence."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, MotionPlanRequest, MoveItErrorCodes, PlanningOptions, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

from manipulation_actions.move_group_client import MoveGroupClient, build_joint_constraints
from pybullet_bridge.robot_profiles import IIWA_HOME, IIWA_JOINTS


@dataclass
class GoalResult:
    label: str
    success: bool
    message: str
    rmse_rad: float | None
    max_abs_error_rad: float | None
    planned_points: int


@dataclass
class CollisionResult:
    scene_applied: bool
    rejected: bool
    error_code: int | None
    message: str


class MoveItClosureChecker(Node):
    def __init__(self) -> None:
        super().__init__('moveit_closure_checker')
        self._mg = MoveGroupClient(self, action_name='/move_action')
        self._move_action = ActionClient(self, MoveGroup, '/move_action')
        self._planning_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._latest_joint_state: JointState | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            qos_profile_sensor_data,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def wait_ready(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest_joint_state is not None and self._mg.wait_for_server(timeout_sec=0.1):
                return True
        return False

    def wait_for_tf(self, base: str, tip: str, timeout_sec: float) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout_sec
        last_error = ''
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                self._tf_buffer.lookup_transform(base, tip, Time(), timeout=Duration(seconds=0.1))
                return True, f'{base}->{tip}'
            except TransformException as exc:
                last_error = str(exc)
        return False, last_error or f'{base}->{tip} unavailable'

    def run_joint_goal(
        self,
        label: str,
        target: list[float],
        *,
        timeout_sec: float,
        settle_sec: float,
    ) -> GoalResult:
        ok, trajectory, message = self._mg.move_to_joint_positions(
            list(IIWA_JOINTS),
            target,
            planning_group='manipulator',
            timeout_sec=timeout_sec,
        )
        planned_points = len(trajectory.points) if trajectory is not None else 0
        deadline = time.monotonic() + settle_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        rmse, max_abs = self._joint_error(target)
        return GoalResult(
            label=label,
            success=ok,
            message=message,
            rmse_rad=rmse,
            max_abs_error_rad=max_abs,
            planned_points=planned_points,
        )

    def run_collision_rejection(self, *, timeout_sec: float) -> CollisionResult:
        if not self._apply_collision_box(add=True, timeout_sec=timeout_sec):
            return CollisionResult(
                scene_applied=False,
                rejected=False,
                error_code=None,
                message='Timed out applying collision scene',
            )

        try:
            target = _goals()[0][1]
            ok, error_code, message = self._plan_only_joint_goal(target, timeout_sec=timeout_sec)
            return CollisionResult(
                scene_applied=True,
                rejected=not ok,
                error_code=error_code,
                message=message,
            )
        finally:
            self._apply_collision_box(add=False, timeout_sec=timeout_sec)

    def _apply_collision_box(self, *, add: bool, timeout_sec: float) -> bool:
        if not self._planning_scene.wait_for_service(timeout_sec=timeout_sec):
            return False

        obj = CollisionObject()
        obj.header.frame_id = 'lbr_iiwa_link_0'
        obj.id = 'fr_mov_blocking_box'
        obj.operation = CollisionObject.ADD if add else CollisionObject.REMOVE

        if add:
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [3.0, 3.0, 3.0]
            pose = Pose()
            pose.position.z = 0.8
            pose.orientation.w = 1.0
            obj.primitives = [box]
            obj.primitive_poses = [pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._planning_scene.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return bool(future.done() and future.result() is not None and future.result().success)

    def _plan_only_joint_goal(
        self,
        target: list[float],
        *,
        timeout_sec: float,
    ) -> tuple[bool, int | None, str]:
        if not self._move_action.wait_for_server(timeout_sec=timeout_sec):
            return False, None, 'MoveGroup action server not available'

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = 'manipulator'
        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = 2.0
        goal.request.max_velocity_scaling_factor = 0.4
        goal.request.max_acceleration_scaling_factor = 0.4
        goal.request.goal_constraints = [
            build_joint_constraints(list(IIWA_JOINTS), target),
        ]
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        send_future = self._move_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        if not send_future.done():
            return False, None, 'Timed out sending collision plan-only goal'

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, None, 'Collision plan-only goal rejected by action server'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            return False, None, 'Timed out waiting for collision plan-only result'

        action_result = result_future.result().result
        code = int(action_result.error_code.val)
        ok = code == MoveItErrorCodes.SUCCESS
        if ok:
            return True, code, 'Unexpectedly planned through blocking collision scene'
        return False, code, f'Collision scene rejected plan as expected: error_code={code}'

    def _joint_error(self, target: list[float]) -> tuple[float | None, float | None]:
        msg = self._latest_joint_state
        if msg is None:
            return None, None
        positions = dict(zip(msg.name, msg.position))
        errors: list[float] = []
        for name, desired in zip(IIWA_JOINTS, target):
            if name not in positions:
                return None, None
            errors.append(float(positions[name]) - float(desired))
        rmse = math.sqrt(statistics.fmean(err * err for err in errors))
        return rmse, max(abs(err) for err in errors)


def _goals() -> list[tuple[str, list[float]]]:
    home = list(IIWA_HOME)
    return [
        (
            'joint_a',
            [
                home[0] + 0.30,
                home[1] - 0.12,
                home[2] + 0.22,
                home[3] + 0.18,
                home[4] + 0.12,
                home[5] - 0.12,
                home[6] + 0.16,
            ],
        ),
        (
            'joint_b',
            [
                home[0] - 0.28,
                home[1] + 0.10,
                home[2] - 0.20,
                home[3] - 0.14,
                home[4] - 0.12,
                home[5] + 0.10,
                home[6] - 0.14,
            ],
        ),
        (
            'joint_c',
            [
                home[0] + 0.18,
                home[1] - 0.08,
                home[2] + 0.26,
                home[3] + 0.10,
                home[4] - 0.18,
                home[5] - 0.08,
                home[6] + 0.20,
            ],
        ),
        (
            'return_home',
            home,
        ),
    ]


def _build_result(
    *,
    goal_results: list[GoalResult],
    collision_result: CollisionResult,
    tf_ok: bool,
    tf_message: str,
    rmse_threshold: float,
) -> dict:
    successes = sum(1 for item in goal_results if item.success)
    success_rate = successes / len(goal_results) if goal_results else 0.0
    rmse_values = [
        item.rmse_rad
        for item in goal_results
        if item.success and item.rmse_rad is not None
    ]
    max_rmse = max(rmse_values) if rmse_values else None
    return {
        'timestamp_unix': round(time.time(), 3),
        'criteria': {
            'standard_goal_success_rate_min': 0.95,
            'rmse_rad_max': rmse_threshold,
            'tf_required': 'lbr_iiwa_link_0->lbr_iiwa_link_7',
            'collision_rejection': 'blocking collision scene should make plan-only request fail',
        },
        'move_group': {
            'goals': [
                {
                    'label': item.label,
                    'success': item.success,
                    'message': item.message,
                    'rmse_rad': None if item.rmse_rad is None else round(item.rmse_rad, 6),
                    'max_abs_error_rad': (
                        None if item.max_abs_error_rad is None else round(item.max_abs_error_rad, 6)
                    ),
                    'planned_points': item.planned_points,
                }
                for item in goal_results
            ],
            'successes': successes,
            'trials': len(goal_results),
            'success_rate': round(success_rate, 3),
            'passes_success_rate': success_rate >= 0.95,
        },
        'execution': {
            'max_rmse_rad': None if max_rmse is None else round(max_rmse, 6),
            'passes_rmse': max_rmse is not None and max_rmse <= rmse_threshold,
        },
        'tf': {
            'passes': tf_ok,
            'message': tf_message,
        },
        'collision_rejection': {
            'passes': collision_result.scene_applied and collision_result.rejected,
            'scene_applied': collision_result.scene_applied,
            'rejected': collision_result.rejected,
            'error_code': collision_result.error_code,
            'message': collision_result.message,
        },
        'overall_passes': (
            success_rate >= 0.95
            and max_rmse is not None
            and max_rmse <= rmse_threshold
            and tf_ok
            and collision_result.scene_applied
            and collision_result.rejected
        ),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Verify FR-MOV MoveIt closure evidence.')
    parser.add_argument('--output', type=Path, default=Path('docs/samples/moveit-closure-metrics.json'))
    parser.add_argument('--startup-timeout-sec', type=float, default=45.0)
    parser.add_argument('--goal-timeout-sec', type=float, default=60.0)
    parser.add_argument('--settle-sec', type=float, default=1.0)
    parser.add_argument('--rmse-threshold-rad', type=float, default=0.05)
    parser.add_argument('--collision-timeout-sec', type=float, default=15.0)
    args = parser.parse_args(argv)

    rclpy.init(args=[])
    node = MoveItClosureChecker()
    try:
        if not node.wait_ready(args.startup_timeout_sec):
            print('[FAIL] MoveGroup or /joint_states not ready', file=sys.stderr)
            return 1

        tf_ok, tf_message = node.wait_for_tf(
            'lbr_iiwa_link_0',
            'lbr_iiwa_link_7',
            timeout_sec=10.0,
        )

        results: list[GoalResult] = []
        for label, target in _goals():
            node.get_logger().info(f'FR-MOV goal: {label}')
            results.append(
                node.run_joint_goal(
                    label,
                    target,
                    timeout_sec=args.goal_timeout_sec,
                    settle_sec=args.settle_sec,
                ),
            )

        node.get_logger().info('FR-MOV collision rejection scene')
        collision_result = node.run_collision_rejection(timeout_sec=args.collision_timeout_sec)

        payload = _build_result(
            goal_results=results,
            collision_result=collision_result,
            tf_ok=tf_ok,
            tf_message=tf_message,
            rmse_threshold=args.rmse_threshold_rad,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        if not payload['overall_passes']:
            print(f'[FAIL] MoveIt closure criteria failed: {args.output}', file=sys.stderr)
            return 1
        print(f'[PASS] MoveIt closure criteria met: {args.output}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
