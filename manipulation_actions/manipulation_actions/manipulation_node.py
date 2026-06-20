"""ROS 2 node hosting Pick and Place action servers."""

from __future__ import annotations

import json

import rclpy
from bridge_monitor_msgs.action import Pick, Place
from bridge_monitor_msgs.msg import RiskStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from manipulation_actions.bridge_motion_client import BridgeMotionClient
from manipulation_actions.gripper_stub import GripperStub
from manipulation_actions.move_group_client import MoveGroupClient
from manipulation_actions.pick_executor import PickExecutor
from manipulation_actions.place_executor import PlaceExecutor


class ManipulationActionsNode(Node):
    """Expose /manipulation/pick and /manipulation/place action servers."""

    def __init__(self) -> None:
        super().__init__('manipulation_actions')

        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('end_effector_link', 'lbr_iiwa_link_7')
        self.declare_parameter('move_group_action', '/move_action')
        self.declare_parameter('use_moveit', True)
        self.declare_parameter('pre_grasp_offset_m', 0.10)
        self.declare_parameter('lift_offset_m', 0.05)
        self.declare_parameter('retreat_offset_m', 0.08)
        self.declare_parameter('grasp_timeout_sec', 30.0)
        self.declare_parameter('joint_names', [
            'lbr_iiwa_joint_1',
            'lbr_iiwa_joint_2',
            'lbr_iiwa_joint_3',
            'lbr_iiwa_joint_4',
            'lbr_iiwa_joint_5',
            'lbr_iiwa_joint_6',
            'lbr_iiwa_joint_7',
        ])
        self.declare_parameter('home_positions', [
            0.0, 0.785398, 0.0, -1.570796, 0.0, 1.570796, 0.0,
        ])

        self._cb_group = ReentrantCallbackGroup()
        self._gripper = GripperStub(self)
        self._pick_executor = PickExecutor(
            default_planning_group=self.get_parameter('planning_group').value,
            default_end_effector_link=self.get_parameter('end_effector_link').value,
            default_pre_grasp_offset_m=self.get_parameter('pre_grasp_offset_m').value,
            default_lift_offset_m=self.get_parameter('lift_offset_m').value,
            gripper=self._gripper,
        )
        self._place_executor = PlaceExecutor(
            default_planning_group=self.get_parameter('planning_group').value,
            default_end_effector_link=self.get_parameter('end_effector_link').value,
            default_retreat_offset_m=self.get_parameter('retreat_offset_m').value,
            gripper=self._gripper,
        )

        self._move_group = MoveGroupClient(
            self,
            action_name=self.get_parameter('move_group_action').value,
        )
        self._bridge_motion = BridgeMotionClient(
            self,
            joint_names=list(self.get_parameter('joint_names').value),
            home_positions=list(self.get_parameter('home_positions').value),
        )

        self._pick_server = ActionServer(
            self,
            Pick,
            '/manipulation/pick',
            execute_callback=self._execute_pick,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self._place_server = ActionServer(
            self,
            Place,
            '/manipulation/place',
            execute_callback=self._execute_place,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self.create_subscription(
            RiskStatus,
            '/risk/status',
            self._on_risk_status,
            10,
            callback_group=self._cb_group,
        )
        self._planning_result_pub = self.create_publisher(
            String, '/manipulation/planning_result', 10)

        use_moveit = self.get_parameter('use_moveit').value
        if use_moveit and self._move_group.wait_for_server(timeout_sec=3.0):
            self._planner = self._move_group
            self.get_logger().info('Using MoveGroup planner (/move_action)')
        else:
            self._planner = self._bridge_motion
            self.get_logger().warn(
                'MoveGroup unavailable; using /bridge/command fallback planner')

        self.get_logger().info(
            'Pick/Place action servers ready: /manipulation/pick, /manipulation/place')

    def _planner_for_request(self):
        if self.get_parameter('use_moveit').value and self._move_group.wait_for_server(timeout_sec=0.5):
            return self._move_group
        return self._bridge_motion

    def _goal_callback(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _on_risk_status(self, msg: RiskStatus) -> None:
        if msg.e_stop_active or msg.level >= 3:
            self._move_group.cancel_all()
            self._bridge_motion.cancel_all()

    def _publish_planning_result(self, action: str, success: bool, message: str) -> None:
        msg = String()
        msg.data = json.dumps({
            'action': action,
            'success': success,
            'message': message,
        }, separators=(',', ':'))
        self._planning_result_pub.publish(msg)

    def _execute_pick(self, goal_handle):
        goal = goal_handle.request
        if goal.grasp_timeout_sec <= 0:
            goal.grasp_timeout_sec = float(self.get_parameter('grasp_timeout_sec').value)
        result = self._pick_executor.execute(goal, goal_handle, self._planner_for_request())
        self._publish_planning_result('pick', result.success, result.message)
        return result

    def _execute_place(self, goal_handle):
        timeout = float(self.get_parameter('grasp_timeout_sec').value)
        result = self._place_executor.execute(
            goal_handle.request,
            goal_handle,
            self._planner_for_request(),
            timeout_sec=timeout,
        )
        self._publish_planning_result('place', result.success, result.message)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManipulationActionsNode()
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
