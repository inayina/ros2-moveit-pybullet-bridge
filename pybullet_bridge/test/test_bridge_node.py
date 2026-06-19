"""Node-level tests for pybullet_bridge (requires PyBullet + URDF)."""

from __future__ import annotations

import math
import threading
import time

import pytest
import rclpy
from bridge_monitor_msgs.msg import SimRealError
from bridge_monitor_msgs.msg import RiskStatus
from builtin_interfaces.msg import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

pytest.importorskip('pybullet')

from pybullet_bridge.bridge_node import PyBulletBridgeNode  # noqa: E402


class _TrajectoryPublisher(Node):
    def __init__(self) -> None:
        super().__init__('test_trajectory_publisher')
        self.pub = self.create_publisher(JointTrajectory, '/bridge/command', 10)

    def publish_sweep(self) -> None:
        deadline = time.time() + 2.0
        while time.time() < deadline and self.pub.get_subscription_count() == 0:
            time.sleep(0.05)

        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2']
        for i in range(21):
            phase = math.pi * i / 20
            pt = JointTrajectoryPoint()
            pt.positions = [0.8 * math.sin(phase), 0.8 * math.cos(phase)]
            t = 4.0 * i / 20
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            msg.points.append(pt)
        self.pub.publish(msg)


class _BridgeObserver(Node):
    def __init__(self) -> None:
        super().__init__('test_bridge_observer')
        self.sim_states: list[JointState] = []
        self.real_states: list[JointState] = []
        self.system_states: list[String] = []
        self.errors: list[SimRealError] = []
        self.create_subscription(
            JointState, '/bridge/sim/joint_states', self._on_sim, qos_profile_sensor_data)
        self.create_subscription(
            JointState, '/bridge/real/joint_states', self._on_real, qos_profile_sensor_data)
        self.create_subscription(
            String, '/bridge/system_state', self._on_system, 10)
        self.create_subscription(
            SimRealError, '/bridge/sim_real_error', self._on_error, qos_profile_sensor_data)

    def _on_sim(self, msg: JointState) -> None:
        self.sim_states.append(msg)

    def _on_real(self, msg: JointState) -> None:
        self.real_states.append(msg)

    def _on_system(self, msg: String) -> None:
        self.system_states.append(msg)

    def _on_error(self, msg: SimRealError) -> None:
        self.errors.append(msg)


def _make_bridge_stack(enable_dual_source: bool):
    if rclpy.ok():
        rclpy.shutdown()
    flag = 'true' if enable_dual_source else 'false'
    rclpy.init(args=[
        '--ros-args',
        '-p', f'enable_dual_source:={flag}',
        '-p', 'robot_profile:=planar_2dof',
    ])
    bridge = PyBulletBridgeNode()
    if not bridge._pybullet_ok:  # noqa: SLF001
        bridge.destroy_node()
        rclpy.shutdown()
        pytest.skip('PyBullet init failed (colcon build + source install required)')

    publisher = _TrajectoryPublisher()
    observer = _BridgeObserver()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(publisher)
    executor.add_node(observer)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return bridge, publisher, observer, executor, thread


def _teardown_stack(bridge, publisher, observer, executor, thread) -> None:
    executor.shutdown()
    thread.join(timeout=2.0)
    observer.destroy_node()
    publisher.destroy_node()
    bridge.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def bridge_stack():
    stack = _make_bridge_stack(enable_dual_source=False)
    yield stack
    _teardown_stack(*stack)


@pytest.fixture
def dual_bridge_stack():
    stack = _make_bridge_stack(enable_dual_source=True)
    yield stack
    _teardown_stack(*stack)


def test_bridge_publishes_joint_states(bridge_stack):
    _bridge, publisher, observer, _executor, _thread = bridge_stack
    publisher.publish_sweep()

    deadline = time.time() + 8.0
    while time.time() < deadline and len(observer.sim_states) < 5:
        time.sleep(0.1)

    assert observer.sim_states, 'expected /bridge/sim/joint_states'
    assert observer.sim_states[-1].name == ['joint1', 'joint2']
    peak = max(abs(v) for s in observer.sim_states for v in s.position)
    assert peak > 0.3
    assert any(s.data == 'RUNNING' for s in observer.system_states)


def test_dual_source_publishes_divergent_states_and_error(dual_bridge_stack):
    bridge, publisher, observer, _executor, _thread = dual_bridge_stack
    if not bridge._dual_source:  # noqa: SLF001
        pytest.skip('dual-source disabled')

    publisher.publish_sweep()

    deadline = time.time() + 10.0
    while time.time() < deadline and (
        len(observer.sim_states) < 5 or len(observer.errors) < 3
    ):
        time.sleep(0.1)

    assert observer.sim_states and observer.real_states
    assert observer.errors, 'expected /bridge/sim_real_error stream'
    assert observer.errors[-1].joint_names == ['joint1', 'joint2']
    assert len(observer.errors[-1].q_error) == 2
    assert len(observer.errors[-1].dq_error) == 2

    max_q_gap = max(
        abs(s - r)
        for s, r in zip(observer.sim_states[-1].position, observer.real_states[-1].position)
    )
    assert max_q_gap > 0.0 or max(abs(e) for e in observer.errors[-1].q_error) > 0.0


class _RiskPublisher(Node):
    def __init__(self) -> None:
        super().__init__('test_risk_publisher')
        self.pub = self.create_publisher(RiskStatus, '/risk/status', 10)

    def publish_e_stop(self, active: bool) -> None:
        deadline = time.time() + 2.0
        while time.time() < deadline and self.pub.get_subscription_count() == 0:
            time.sleep(0.05)
        msg = RiskStatus()
        msg.e_stop_active = active
        msg.level = 3 if active else 0
        self.pub.publish(msg)


def test_bridge_honors_risk_e_stop(bridge_stack):
    bridge, _publisher, observer, _executor, _thread = bridge_stack
    risk_pub = _RiskPublisher()
    risk_executor = SingleThreadedExecutor()
    risk_executor.add_node(risk_pub)
    risk_thread = threading.Thread(target=risk_executor.spin, daemon=True)
    risk_thread.start()

    try:
        risk_pub.publish_e_stop(True)
        deadline = time.time() + 3.0
        while time.time() < deadline and not bridge._e_stop:  # noqa: SLF001
            time.sleep(0.05)
        assert bridge._e_stop is True  # noqa: SLF001

        deadline = time.time() + 3.0
        while time.time() < deadline and not any(s.data == 'E_STOP' for s in observer.system_states):
            time.sleep(0.05)
        assert any(s.data == 'E_STOP' for s in observer.system_states)

        risk_pub.publish_e_stop(False)
        deadline = time.time() + 3.0
        while time.time() < deadline and bridge._e_stop:  # noqa: SLF001
            time.sleep(0.05)
        assert bridge._e_stop is False  # noqa: SLF001
    finally:
        risk_executor.shutdown()
        risk_thread.join(timeout=2.0)
        risk_pub.destroy_node()
