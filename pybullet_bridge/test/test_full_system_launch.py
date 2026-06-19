"""Full-system launch integration test: bridge + monitor + risk pipeline."""

from __future__ import annotations

import threading
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from bridge_monitor_msgs.msg import DistributionMetrics, RiskStatus
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class _PipelineCollector(Node):
    def __init__(self) -> None:
        super().__init__('full_system_launch_test_collector')
        self.joint_samples: list[JointState] = []
        self.metrics: list[DistributionMetrics] = []
        self.risk_statuses: list[RiskStatus] = []
        self.create_subscription(
            JointState, '/bridge/sim/joint_states', self._on_joint, qos_profile_sensor_data)
        self.create_subscription(
            DistributionMetrics, '/monitor/distribution_metrics', self._on_metrics, 10)
        self.create_subscription(RiskStatus, '/risk/status', self._on_risk, 10)

    def _on_joint(self, msg: JointState) -> None:
        self.joint_samples.append(msg)

    def _on_metrics(self, msg: DistributionMetrics) -> None:
        self.metrics.append(msg)

    def _on_risk(self, msg: RiskStatus) -> None:
        self.risk_statuses.append(msg)


@pytest.mark.launch_test
def generate_test_description():
    pkg = FindPackageShare('pybullet_bridge')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', 'test_monitoring.launch.py'])),
            launch_arguments={'sim_mode': 'DIRECT'}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestFullSystemLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.collector = _PipelineCollector()
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

    def test_monitoring_pipeline_topics(self) -> None:
        deadline = time.time() + 12.0
        while time.time() < deadline:
            if (self.collector.joint_samples
                    and self.collector.metrics
                    and self.collector.risk_statuses):
                break
            time.sleep(0.2)

        self.assertTrue(
            self.collector.joint_samples,
            'bridge should publish /bridge/sim/joint_states')
        self.assertTrue(
            self.collector.metrics,
            'dist_monitor should publish /monitor/distribution_metrics')
        self.assertTrue(
            self.collector.risk_statuses,
            'risk_engine should publish /risk/status')

        latest_risk = self.collector.risk_statuses[-1]
        self.assertGreaterEqual(latest_risk.level, 0)
        self.assertEqual(len(latest_risk.attribution), 5)
