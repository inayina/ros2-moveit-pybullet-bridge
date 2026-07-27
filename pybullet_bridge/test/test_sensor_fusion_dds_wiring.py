"""ROS graph wiring test for the experimental three-stream sensor fusion node."""

from __future__ import annotations

import time

from bridge_monitor_msgs.msg import GraspStatus
from geometry_msgs.msg import WrenchStamped
from pybullet_bridge.sensor_fusion_node import SensorFusionNode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState


@pytest.fixture(scope='module', autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin_until(executor, predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_three_sensor_streams_cross_ros_graph_and_produce_fused_status():
    fusion = SensorFusionNode()
    probe = Node('sensor_fusion_dds_wiring_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(fusion)
    executor.add_node(probe)

    statuses: list[GraspStatus] = []
    status_sub = probe.create_subscription(
        GraspStatus,
        '/bridge/sim/grasp_status',
        statuses.append,
        10,
    )
    joint_pub = probe.create_publisher(
        JointState, '/bridge/sim/joint_states', qos_profile_sensor_data)
    camera_pub = probe.create_publisher(
        Image, '/camera/color/image_raw', qos_profile_sensor_data)
    ft_pub = probe.create_publisher(
        WrenchStamped, '/ft_sensor', qos_profile_sensor_data)

    try:
        assert _spin_until(
            executor,
            lambda: all(pub.get_subscription_count() == 1 for pub in (
                joint_pub, camera_pub, ft_pub)),
            5.0,
        ), 'sensor-data publishers were not discovered by all fusion subscribers'

        for topic in (
            '/bridge/sim/joint_states',
            '/camera/color/image_raw',
            '/ft_sensor',
        ):
            endpoints = fusion.get_subscriptions_info_by_topic(topic)
            assert len(endpoints) == 1
            assert endpoints[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT

        stamp = probe.get_clock().now().to_msg()
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = [f'panda_joint{index}' for index in range(1, 8)]
        joint.position = [0.0] * 7
        image = Image()
        image.header.stamp = stamp
        wrench = WrenchStamped()
        wrench.header.stamp = stamp

        # Publish through rclpy endpoints; do not call the fusion callback. Repeating
        # the same coherent triplet tolerates DDS discovery variance in CI while the
        # first received triplet is still the one asserted.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not statuses:
            joint_pub.publish(joint)
            camera_pub.publish(image)
            ft_pub.publish(wrench)
            executor.spin_once(timeout_sec=0.1)

        assert statuses, 'no fused GraspStatus crossed the ROS 2 graph'
        assert status_sub.get_publisher_count() == 1
        assert statuses[0].header.frame_id == 'panda_hand'
    finally:
        executor.remove_node(probe)
        executor.remove_node(fusion)
        probe.destroy_node()
        fusion.destroy_node()
