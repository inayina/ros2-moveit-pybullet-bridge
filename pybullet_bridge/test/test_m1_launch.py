"""M1 launch integration test: m1_demo.launch.py + joint motion."""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
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
def test_m1_launch_publishes_joint_motion() -> None:
    """Run the launch file as a subprocess so pytest always owns shutdown."""
    old_domain_id = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = old_domain_id or '71'
    env = os.environ.copy()
    proc = subprocess.Popen(
        ['ros2', 'launch', 'pybullet_bridge', 'm1_demo.launch.py', 'sim_mode:=DIRECT'],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    rclpy.init()
    collector = _JointStateCollector()
    executor = SingleThreadedExecutor()
    executor.add_node(collector)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        deadline = time.time() + 14.0
        while time.time() < deadline:
            if len(collector.samples) >= 2:
                peak = max(abs(v) for _, pos in collector.samples for v in pos)
                if peak > 0.3:
                    break
            time.sleep(0.2)

        assert len(collector.samples) >= 2, 'expected /bridge/sim/joint_states samples'
        names = collector.samples[-1][0]
        peak = max(abs(v) for _, pos in collector.samples for v in pos)
        assert names == ['joint1', 'joint2']
        assert peak > 0.3
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
