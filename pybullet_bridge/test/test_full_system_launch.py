"""Full-system launch integration test: bridge + monitor + risk pipeline."""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from bridge_monitor_msgs.msg import DistributionMetrics, RiskStatus
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
def test_full_system_launch_publishes_monitoring_pipeline() -> None:
    """Run the monitoring launch as a subprocess with explicit teardown."""
    old_domain_id = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = old_domain_id or '72'
    env = os.environ.copy()
    proc = subprocess.Popen(
        [
            'ros2',
            'launch',
            'pybullet_bridge',
            'test_monitoring.launch.py',
            'sim_mode:=DIRECT',
        ],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    rclpy.init()
    collector = _PipelineCollector()
    executor = SingleThreadedExecutor()
    executor.add_node(collector)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        deadline = time.time() + 16.0
        while time.time() < deadline:
            if collector.joint_samples and collector.metrics and collector.risk_statuses:
                break
            time.sleep(0.2)

        assert collector.joint_samples, 'bridge should publish /bridge/sim/joint_states'
        assert collector.metrics, 'dist_monitor should publish /monitor/distribution_metrics'
        assert collector.risk_statuses, 'risk_engine should publish /risk/status'

        latest_risk = collector.risk_statuses[-1]
        assert latest_risk.level >= 0
        assert len(latest_risk.attribution) == 5
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except PermissionError:
                proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except PermissionError:
                    proc.kill()
                proc.wait(timeout=5.0)
        if old_domain_id is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = old_domain_id
