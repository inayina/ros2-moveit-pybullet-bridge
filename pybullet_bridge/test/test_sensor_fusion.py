"""Unit tests for sensor fusion and gravity/inertia compensation node."""

from __future__ import annotations

import rclpy
import pytest
import numpy as np
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import WrenchStamped
from bridge_monitor_msgs.msg import GraspStatus

from pybullet_bridge.sensor_fusion_node import SensorFusionNode


class _GraspStatusObserver:
    def __init__(self, node: rclpy.node.Node) -> None:
        self.received_msgs: list[GraspStatus] = []
        self.sub = node.create_subscription(
            GraspStatus,
            '/bridge/sim/grasp_status',
            self._callback,
            10,
        )

    def _callback(self, msg: GraspStatus) -> None:
        self.received_msgs.append(msg)


@pytest.fixture(scope='module', autouse=True)
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_sensor_fusion_node_initialization():
    node = SensorFusionNode()
    assert node.get_name() == 'sensor_fusion_node'
    node.destroy_node()


def test_sensor_fusion_compensation_and_estimation():
    node = SensorFusionNode()
    observer = _GraspStatusObserver(node)

    # Mock messages
    joint_msg = JointState()
    joint_msg.header.stamp.sec = 100
    joint_msg.header.stamp.nanosec = 0
    joint_msg.name = [f'panda_joint{idx+1}' for idx in range(7)]
    # Set to zero positions
    joint_msg.position = [0.0] * 7

    image_msg = Image()
    image_msg.header.stamp = joint_msg.header.stamp

    # In zero-gravity position, gripper points downwards.
    # Gripper mass is 0.73kg, CoM z offset is 0.05m.
    # Under gravity g = -9.81 m/s^2 along z axis:
    # Expected gravity force along sensor local z axis (pointing opposite to gravity) is 0.73 * 9.81 = 7.1613 N.
    # Torque expected is 0.0.
    # Let's inject a raw FT reading that has EXACTLY this gravity force, so net force should be zero!
    ft_msg = WrenchStamped()
    ft_msg.header.stamp = joint_msg.header.stamp
    ft_msg.wrench.force.x = 0.0
    ft_msg.wrench.force.y = 0.0
    ft_msg.wrench.force.z = 7.1613
    ft_msg.wrench.torque.x = 0.0
    ft_msg.wrench.torque.y = 0.0
    ft_msg.wrench.torque.z = 0.0

    # Call the synced callback directly
    node._on_synced_data(joint_msg, image_msg, ft_msg)

    # Spin once to process published status message
    rclpy.spin_once(node, timeout_sec=0.01)

    assert len(observer.received_msgs) == 1
    status = observer.received_msgs[0]

    # Compensated net wrench should be very close to zero
    assert status.net_wrench.force.x == pytest.approx(0.0, abs=1e-2)
    assert status.net_wrench.force.y == pytest.approx(0.0, abs=1e-2)
    assert status.net_wrench.force.z == pytest.approx(0.0, abs=1e-2)
    assert status.net_wrench.torque.x == pytest.approx(0.0, abs=1e-2)
    assert status.net_wrench.torque.y == pytest.approx(0.0, abs=1e-2)
    assert status.net_wrench.torque.z == pytest.approx(0.0, abs=1e-2)

    # Net contact norm is 0.0 < contact_force_threshold (2.0), so grasp is not established
    assert not status.grasp_established
    assert not status.object_slipped

    # Test Case 2: Inject large contact force (e.g. 5.0 N along x axis)
    # Total force = gravity + contact = [0.0, 0.0, 7.1613] + [5.0, 0.0, 0.0]
    ft_msg_contact = WrenchStamped()
    ft_msg_contact.header.stamp.sec = 100
    ft_msg_contact.header.stamp.nanosec = 100000000  # +0.1s
    ft_msg_contact.wrench.force.x = 5.0
    ft_msg_contact.wrench.force.y = 0.0
    ft_msg_contact.wrench.force.z = 7.1613

    joint_msg2 = JointState()
    joint_msg2.header.stamp = ft_msg_contact.header.stamp
    joint_msg2.name = joint_msg.name
    joint_msg2.position = joint_msg.position

    node._on_synced_data(joint_msg2, image_msg, ft_msg_contact)
    rclpy.spin_once(node, timeout_sec=0.01)

    assert len(observer.received_msgs) == 2
    status2 = observer.received_msgs[1]

    # Compensated net force should be around [5.0, 0.0, 0.0]
    assert status2.net_wrench.force.x == pytest.approx(5.0, abs=1e-2)
    assert status2.net_wrench.force.y == pytest.approx(0.0, abs=1e-2)
    assert status2.net_wrench.force.z == pytest.approx(0.0, abs=1e-2)

    # 5.0 N > 2.0 N threshold -> grasp established!
    assert status2.grasp_established
    assert not status2.object_slipped

    node.destroy_node()
