"""ROS 2 node that runs a policy and publishes JointTrajectory commands."""

from __future__ import annotations

import time
from typing import Dict, Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from bridge_monitor_msgs.msg import DistributionMetrics
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pybullet_bridge.learning.base_policy import BasePolicy
from pybullet_bridge.learning.replay_policy import ReplayPolicy
from pybullet_bridge.learning.sine_wave_policy import SineWavePolicy


class PolicyRunner(Node):
    """Run a baseline policy against the latest simulated joint state."""

    def __init__(self) -> None:
        super().__init__('policy_runner')
        self._declare_parameters()

        self._strategy_type = ''
        self._joint_names: list[str] = []
        self._latest_obs: Optional[Dict[str, np.ndarray]] = None
        self._latest_metrics: Optional[DistributionMetrics] = None
        self._policy: Optional[BasePolicy] = None
        self._rng: Optional[np.random.Generator] = None
        self._active = False
        self._reason = 'inactive'

        self._last_successful_action_mono = time.monotonic()
        self._last_inference_latency_ms = 0.0
        self._last_metrics_log_mono = 0.0

        self._cmd_pub: Optional[rclpy.publisher.Publisher] = None
        self._health_pub: Optional[rclpy.publisher.Publisher] = None
        self._inference_timer: Optional[rclpy.timer.Timer] = None
        self._health_timer: Optional[rclpy.timer.Timer] = None

        self.on_configure()
        self.on_activate()

    def _declare_parameters(self) -> None:
        self.declare_parameter('strategy_type', 'replay')
        self.declare_parameter('replay_path', '')
        self.declare_parameter('policy_inference_freq', 0)
        self.declare_parameter('joint_names', [])
        self.declare_parameter('command_topic', '/bridge/command')
        self.declare_parameter('joint_state_topic', '/bridge/sim/joint_states')
        self.declare_parameter('metrics_topic', '/monitor/distribution_metrics')
        self.declare_parameter('system_health_topic', '/system_health')
        self.declare_parameter('fault_injection_enabled', False)
        self.declare_parameter('fault_sleep_probability', 0.0)
        self.declare_parameter('fault_sleep_sec', 0.1)
        self.declare_parameter('watchdog_timeout_sec', 1.0)
        self.declare_parameter('sine_amplitude', 0.05)
        self.declare_parameter('sine_frequency_hz', 0.2)
        self.declare_parameter('seed', 0)

    @property
    def policy(self) -> BasePolicy:
        if self._policy is None:
            raise RuntimeError('PolicyRunner is not configured')
        return self._policy

    @property
    def health_reason(self) -> str:
        return self._reason

    def on_configure(self) -> bool:
        self._strategy_type = str(self.get_parameter('strategy_type').value)
        self._joint_names = list(self.get_parameter('joint_names').value)
        self._policy = self._build_policy()
        self._rng = np.random.default_rng(int(self.get_parameter('seed').value))

        command_topic = str(self.get_parameter('command_topic').value)
        health_topic = str(self.get_parameter('system_health_topic').value)
        joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        metrics_topic = str(self.get_parameter('metrics_topic').value)

        self._cmd_pub = self.create_publisher(JointTrajectory, command_topic, 10)
        self._health_pub = self.create_publisher(DiagnosticArray, health_topic, 10)
        self.create_subscription(
            JointState,
            joint_state_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DistributionMetrics,
            metrics_topic,
            self._on_metrics,
            10,
        )
        return True

    def on_activate(self) -> bool:
        if self._policy is None:
            return False

        self._policy.reset()
        self._active = True
        self._reason = 'ok'
        self._last_successful_action_mono = time.monotonic()

        inference_hz = self._policy.inference_freq
        self._inference_timer = self.create_timer(1.0 / inference_hz, self._on_timer)
        self._health_timer = self.create_timer(1.0, self._on_health_timer)

        self.get_logger().info(
            f'[PolicyRunner] 加载策略: {self._strategy_type}, '
            f'推理频率: {inference_hz} Hz'
        )
        return True

    def on_deactivate(self) -> bool:
        self._active = False
        self._reason = 'inactive'
        self._destroy_timers()
        self._publish_health(force_level=DiagnosticStatus.OK)
        return True

    def on_cleanup(self) -> bool:
        self.on_deactivate()
        self._latest_obs = None
        self._latest_metrics = None
        self._policy = None
        self._rng = None
        return True

    def on_shutdown(self) -> bool:
        self.get_logger().info('[PolicyRunner] shutdown')
        return self.on_cleanup()

    def _destroy_timers(self) -> None:
        if self._inference_timer is not None:
            self._inference_timer.cancel()
            self.destroy_timer(self._inference_timer)
            self._inference_timer = None
        if self._health_timer is not None:
            self._health_timer.cancel()
            self.destroy_timer(self._health_timer)
            self._health_timer = None

    def _build_policy(self) -> BasePolicy:
        requested_freq = int(self.get_parameter('policy_inference_freq').value)
        strategy = self._strategy_type.strip().lower()

        if strategy == 'replay':
            replay_path = str(self.get_parameter('replay_path').value)
            if not replay_path:
                raise ValueError('replay_path is required when strategy_type is replay')
            kwargs = {}
            if requested_freq > 0:
                kwargs['inference_freq'] = requested_freq
            return ReplayPolicy(replay_path, **kwargs)

        if strategy == 'sine_wave':
            inference_freq = requested_freq if requested_freq > 0 else 50
            return SineWavePolicy(
                amplitude=float(self.get_parameter('sine_amplitude').value),
                frequency_hz=float(self.get_parameter('sine_frequency_hz').value),
                inference_freq=inference_freq,
                seed=int(self.get_parameter('seed').value),
            )

        raise ValueError(f'Unsupported strategy_type: {self._strategy_type}')

    def _on_joint_state(self, msg: JointState) -> None:
        if not self._joint_names:
            self._joint_names = list(msg.name)

        self._latest_obs = {
            'joint_positions': np.asarray(msg.position, dtype=np.float64),
            'joint_velocities': np.asarray(msg.velocity, dtype=np.float64),
            'stamp_sec': np.asarray([self._stamp_to_sec(msg)], dtype=np.float64),
        }

    def _on_metrics(self, msg: DistributionMetrics) -> None:
        self._latest_metrics = msg
        now = time.monotonic()
        if now - self._last_metrics_log_mono >= 1.0:
            self._last_metrics_log_mono = now
            self.get_logger().info(
                f'[PolicyRunner] metrics kl={msg.kl_divergence_mean:.4f} '
                f'w1={msg.wasserstein_mean:.4f} mmd={msg.mmd_statistic:.4f} '
                f'shift={msg.shift_detected}'
            )

    def _on_timer(self) -> None:
        if not self._active or self._policy is None or self._latest_obs is None:
            return

        if self._maybe_inject_fault():
            self._reason = 'stalled'
            self._publish_health(force_level=DiagnosticStatus.WARN)
            return

        start = time.monotonic()
        try:
            action = np.asarray(self._policy.get_action(self._latest_obs), dtype=np.float64)
        except Exception as exc:  # noqa: BLE001 - policy errors become health diagnostics
            self._reason = 'exception'
            self.get_logger().error(f'Policy inference failed: {exc}')
            self._publish_health(force_level=DiagnosticStatus.ERROR)
            return

        elapsed_ms = (time.monotonic() - start) * 1000.0
        self._last_inference_latency_ms = elapsed_ms
        watchdog_ms = float(self.get_parameter('watchdog_timeout_sec').value) * 1000.0
        if elapsed_ms > watchdog_ms:
            self._reason = 'stalled'
            self._publish_health(force_level=DiagnosticStatus.ERROR)
            return

        if action.ndim != 1:
            self._reason = 'exception'
            self.get_logger().error('Policy action must be a 1D array')
            self._publish_health(force_level=DiagnosticStatus.ERROR)
            return
        if len(self._joint_names) != action.shape[0]:
            self._reason = 'exception'
            self.get_logger().error(
                'Policy action dimension does not match joint_names: '
                f'{action.shape[0]} vs {len(self._joint_names)}'
            )
            self._publish_health(force_level=DiagnosticStatus.ERROR)
            return

        if self._cmd_pub is not None:
            self._cmd_pub.publish(self._make_command(action))
        self._last_successful_action_mono = time.monotonic()
        self._reason = 'ok'

    def _on_health_timer(self) -> None:
        if not self._active:
            self._publish_health(force_level=DiagnosticStatus.OK)
            return

        watchdog_sec = float(self.get_parameter('watchdog_timeout_sec').value)
        idle_sec = time.monotonic() - self._last_successful_action_mono
        if idle_sec > watchdog_sec and self._reason != 'inactive':
            self._reason = 'stalled'
            level = DiagnosticStatus.ERROR
        elif self._reason == 'exception':
            level = DiagnosticStatus.ERROR
        elif self._reason == 'stalled':
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.OK
        self._publish_health(force_level=level)

    def _maybe_inject_fault(self) -> bool:
        if not bool(self.get_parameter('fault_injection_enabled').value):
            return False
        if self._rng is None:
            return False

        probability = float(self.get_parameter('fault_sleep_probability').value)
        if probability <= 0.0:
            return False
        if self._rng.random() >= probability:
            return False

        sleep_sec = float(self.get_parameter('fault_sleep_sec').value)
        time.sleep(sleep_sec)
        return True

    def _publish_health(self, force_level: int | None = None) -> None:
        if self._health_pub is None:
            return

        idle_ms = (time.monotonic() - self._last_successful_action_mono) * 1000.0
        if force_level is not None:
            level = force_level
        elif self._reason == 'inactive':
            level = DiagnosticStatus.OK
        elif self._reason == 'exception':
            level = DiagnosticStatus.ERROR
        elif self._reason == 'stalled':
            level = DiagnosticStatus.WARN
        else:
            level = DiagnosticStatus.OK

        status = DiagnosticStatus()
        status.name = 'policy_runner.health'
        status.hardware_id = 'policy_runner'
        status.level = level
        status.message = self._reason
        status.values = [
            KeyValue(key='strategy_type', value=self._strategy_type),
            KeyValue(
                key='inference_latency_ms',
                value=f'{self._last_inference_latency_ms:.3f}',
            ),
            KeyValue(key='last_action_age_ms', value=f'{idle_ms:.3f}'),
            KeyValue(
                key='fault_injection',
                value=str(bool(self.get_parameter('fault_injection_enabled').value)).lower(),
            ),
            KeyValue(key='reason', value=self._reason),
        ]

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status.append(status)
        self._health_pub.publish(diag)

    def _make_command(self, action: np.ndarray) -> JointTrajectory:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self._joint_names)

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in action]
        point.time_from_start = self._duration_from_seconds(
            1.0 / self._policy.inference_freq
        )
        msg.points = [point]
        return msg

    @staticmethod
    def _duration_from_seconds(seconds: float) -> Duration:
        whole = int(seconds)
        return Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))

    @staticmethod
    def _stamp_to_sec(msg: JointState) -> float:
        stamp = msg.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PolicyRunner()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
