"""Node-level tests for manipulation action servers."""

from __future__ import annotations

import threading
import time

import pytest
import rclpy
from bridge_monitor_msgs.action import Pick
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from manipulation_actions.manipulation_node import ManipulationActionsNode


class _JointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('manipulation_test_joint_pub')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self._names = [
            'lbr_iiwa_joint_1',
            'lbr_iiwa_joint_2',
            'lbr_iiwa_joint_3',
            'lbr_iiwa_joint_4',
            'lbr_iiwa_joint_5',
            'lbr_iiwa_joint_6',
            'lbr_iiwa_joint_7',
        ]
        self._positions = [0.0, 0.785398, 0.0, -1.570796, 0.0, 1.570796, 0.0]

    def publish_once(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._names
        msg.position = list(self._positions)
        self.pub.publish(msg)


@pytest.fixture(scope='module')
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def manipulation_stack(ros_context):
    node = ManipulationActionsNode()
    node.set_parameters([
        rclpy.parameter.Parameter('use_moveit', rclpy.Parameter.Type.BOOL, False),
    ])
    publisher = _JointStatePublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(publisher)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    publisher.publish_once()
    yield node, publisher
    executor.shutdown()
    thread.join(timeout=2.0)
    publisher.destroy_node()
    node.destroy_node()


def test_manipulation_pick_action_server_accepts_goal(manipulation_stack):
    node, _publisher = manipulation_stack
    client = ActionClient(node, Pick, '/manipulation/pick')
    assert client.wait_for_server(timeout_sec=5.0)

    goal = Pick.Goal()
    goal.grasp_pose = PoseStamped()
    goal.grasp_pose.header.frame_id = 'lbr_iiwa_link_0'
    goal.grasp_pose.pose = Pose(
        position=Point(x=0.4, y=0.0, z=0.3),
        orientation=Quaternion(w=1.0),
    )
    goal.planning_group = 'manipulator'
    goal.pre_grasp_offset_m = 0.05
    goal.grasp_timeout_sec = 5.0

    send_future = client.send_goal_async(goal)
    deadline = time.time() + 10.0
    while time.time() < deadline and not send_future.done():
        time.sleep(0.05)
    assert send_future.done(), 'Pick goal send timed out'
    handle = send_future.result()
    assert handle is not None and handle.accepted

    result_future = handle.get_result_async()
    deadline = time.time() + 12.0
    while time.time() < deadline and not result_future.done():
        time.sleep(0.05)
    assert result_future.done(), 'Pick result timed out'
    client.destroy()
