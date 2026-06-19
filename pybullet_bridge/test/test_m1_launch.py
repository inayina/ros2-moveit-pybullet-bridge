"""M1 launch integration test: m1_demo.launch.py + joint motion."""

from __future__ import annotations

import threading
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class _JointStateCollector(Node):
    def __init__(self) -> None:
        super().__init__('m1_launch_test_collector')
        self.samples: list[tuple[list[str], list[float]]] = []
        self.create_subscription(
            JointState, '/bridge/sim/joint_states', self._cb, qos_profile_sensor_data)

    def _cb(self, msg: JointState) -> None:
        if len(msg.position) >= 2:
            self.samples.append((list(msg.name), list(msg.position)))


@pytest.mark.launch_test
def generate_test_description():
    pkg = FindPackageShare('pybullet_bridge')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', 'm1_demo.launch.py'])),
            launch_arguments={'sim_mode': 'DIRECT'}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestM1Launch(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.collector = _JointStateCollector()
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.collector)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.executor.shutdown()
        cls.spin_thread.join(timeout=2.0)
        cls.collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def test_joint_states_and_motion(self) -> None:
        # joint_sweep_demo: 1s launch delay + 2s publish_delay + 4s trajectory
        time.sleep(10.0)

        self.assertGreaterEqual(
            len(self.collector.samples), 2,
            'expected /bridge/sim/joint_states samples from m1_demo')

        names = self.collector.samples[-1][0]
        peak = max(abs(v) for _, pos in self.collector.samples for v in pos)
        self.assertEqual(names, ['joint1', 'joint2'])
        self.assertGreater(peak, 0.3)
