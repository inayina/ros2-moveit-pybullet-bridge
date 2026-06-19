"""FollowJointTrajectory action server bridging MoveIt2 to pybullet_bridge."""

from __future__ import annotations

import copy
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryControllerNode(Node):
    """MoveIt2 controller shim: Action -> /bridge/command, feedback from /joint_states."""

    def __init__(self) -> None:
        super().__init__('arm_trajectory_controller')

        self.declare_parameter('joint_names', ['joint1', 'joint2'])
        self.declare_parameter('action_name', '/arm_controller/follow_joint_trajectory')
        self.declare_parameter('goal_tolerance', 0.03)
        self.declare_parameter('goal_time_margin_sec', 1.0)
        self.declare_parameter('execution_duration_scaling', 1.5)

        self._joint_names = list(self.get_parameter('joint_names').value)
        self._positions: dict[str, float] = {}
        self._cb_group = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(JointTrajectory, '/bridge/command', 10)
        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joint_state,
            qos_profile_sensor_data,
            callback_group=self._cb_group,
        )

        action_name = self.get_parameter('action_name').value
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self.get_logger().info(f'FollowJointTrajectory server ready: {action_name}')

    def _on_joint_state(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._positions[name] = float(pos)

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        trajectory = goal_request.trajectory
        if not trajectory.joint_names or not trajectory.points:
            self.get_logger().warn('Rejected goal: empty trajectory')
            return GoalResponse.REJECT

        expected = set(self._joint_names)
        requested = set(trajectory.joint_names)
        if not requested.issubset(expected):
            self.get_logger().warn(
                f'Rejected goal: joints {requested} not subset of {expected}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = list(trajectory.joint_names)
        final_point = trajectory.points[-1]
        goal_positions = {
            name: float(final_point.positions[i])
            for i, name in enumerate(joint_names)
        }
        duration_sec = self._trajectory_duration_sec(trajectory)
        timeout_sec = (
            duration_sec * float(self.get_parameter('execution_duration_scaling').value)
            + float(self.get_parameter('goal_time_margin_sec').value)
        )
        goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        cmd = copy.deepcopy(trajectory)
        cmd.header.stamp = self.get_clock().now().to_msg()
        self._cmd_pub.publish(cmd)
        self.get_logger().info(
            f'Executing trajectory: joints={joint_names}, '
            f'points={len(trajectory.points)}, duration={duration_sec:.2f}s')

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names
        result = FollowJointTrajectory.Result()
        start = time.monotonic()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_hold(joint_names)
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = 'Trajectory canceled'
                return result

            feedback.actual.positions = [
                self._positions.get(name, 0.0) for name in joint_names]
            goal_handle.publish_feedback(feedback)

            if self._positions_at_goal(goal_positions, joint_names, goal_tolerance):
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = 'Goal reached'
                self.get_logger().info('Trajectory execution succeeded')
                return result

            if time.monotonic() - start > timeout_sec:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = (
                    f'Timeout after {timeout_sec:.1f}s '
                    f'(duration={duration_sec:.1f}s)')
                self.get_logger().error(result.error_string)
                return result

            time.sleep(0.02)

        goal_handle.abort()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = 'Node shutting down'
        return result

    def _publish_hold(self, joint_names: list[str]) -> None:
        hold = JointTrajectory()
        hold.header.stamp = self.get_clock().now().to_msg()
        hold.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [self._positions.get(name, 0.0) for name in joint_names]
        pt.time_from_start.sec = 0
        hold.points = [pt]
        self._cmd_pub.publish(hold)

    @staticmethod
    def _trajectory_duration_sec(trajectory: JointTrajectory) -> float:
        if not trajectory.points:
            return 0.0
        last = trajectory.points[-1].time_from_start
        return float(last.sec) + float(last.nanosec) * 1e-9

    def _positions_at_goal(
        self,
        goal_positions: dict[str, float],
        joint_names: list[str],
        tolerance: float,
    ) -> bool:
        for name in joint_names:
            if name not in self._positions:
                return False
            if abs(self._positions[name] - goal_positions[name]) > tolerance:
                return False
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
