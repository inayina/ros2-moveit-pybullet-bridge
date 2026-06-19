"""ROS 2 node for KL/MMD distribution shift monitoring."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from bridge_monitor_msgs.msg import DistributionMetrics

from dist_monitor.lerobot_loader import LeRobotTrajectory, load_lerobot_dataset
from dist_monitor.metrics_core import MetricsConfig, compute_distribution_metrics
from dist_monitor.sliding_window import SlidingWindow
from dist_monitor.time_aligner import TimeAligner


def _stamp_to_sec(msg) -> float:
    return float(msg.sec) + float(msg.nanosec) * 1e-9


def _extract_full_state(msg: JointState) -> np.ndarray:
    pos = np.asarray(msg.position, dtype=float)
    if msg.velocity:
        vel = np.asarray(msg.velocity, dtype=float)
    else:
        vel = np.zeros_like(pos)
    n = min(len(pos), len(vel))
    return np.concatenate([pos[:n], vel[:n]])


class DistMonitorNode(Node):
    """Subscribe to dual-source joint states and publish distribution metrics."""

    def __init__(self) -> None:
        super().__init__('dist_monitor')

        self.declare_parameter('window_duration_sec', 5.0)
        self.declare_parameter('update_frequency_hz', 10.0)
        self.declare_parameter('kl_threshold_mean', 0.15)
        self.declare_parameter('mmd_threshold', 0.05)
        self.declare_parameter('mmd_p_value_alpha', 0.05)
        self.declare_parameter('mmd_permutation_count', 50)
        self.declare_parameter('mmd_max_samples', 100)
        self.declare_parameter('mmd_gamma', 0.0)
        self.declare_parameter('use_kl', True)
        self.declare_parameter('use_mmd', True)
        self.declare_parameter('min_samples', 50)
        self.declare_parameter('align_tolerance_sec', 0.02)
        self.declare_parameter('baseline_duration_sec', 30.0)
        self.declare_parameter('real_source', 'topic')
        self.declare_parameter('lerobot_dataset_path', '')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        duration = self.get_parameter('window_duration_sec').value
        self._sim_window = SlidingWindow(duration)
        self._real_window = SlidingWindow(duration)
        self._aligner = TimeAligner(self.get_parameter('align_tolerance_sec').value)

        self._joint_names: list[str] = []
        self._baseline_errors: np.ndarray | None = None
        self._baseline_ready = False
        self._baseline_started_at: float | None = None
        self._baseline_buffer: list[np.ndarray] = []

        self._lerobot_traj: LeRobotTrajectory | None = None
        self._lerobot_origin: float | None = None
        self._load_lerobot_if_configured()

        self.create_subscription(
            JointState, '/bridge/sim/joint_states', self._on_sim, sensor_qos)
        if self.get_parameter('real_source').value == 'topic':
            self.create_subscription(
                JointState, '/bridge/real/joint_states', self._on_real, sensor_qos)

        self._metrics_pub = self.create_publisher(
            DistributionMetrics, '/monitor/distribution_metrics', 10)
        self._error_pub = self.create_publisher(
            JointState, '/monitor/tracking_error', sensor_qos)

        self.create_service(Trigger, '/monitor/reset_baseline', self._handle_reset_baseline)

        hz = self.get_parameter('update_frequency_hz').value
        self._timer = self.create_timer(1.0 / hz, self._compute_and_publish)
        self.get_logger().info('dist_monitor node started.')

    def _load_lerobot_if_configured(self) -> None:
        if self.get_parameter('real_source').value != 'lerobot':
            return
        path = self.get_parameter('lerobot_dataset_path').value
        if not path:
            self.get_logger().error('lerobot_dataset_path is required when real_source=lerobot')
            return
        try:
            self._lerobot_traj = load_lerobot_dataset(path)
            self._lerobot_origin = time.monotonic()
            self._joint_names = list(self._lerobot_traj.joint_names)
            self.get_logger().info(
                f'Loaded LeRobot Real trajectory: {len(self._lerobot_traj.timestamps)} samples')
        except Exception as exc:
            self.get_logger().error(f'Failed to load LeRobot dataset: {exc}')

    def _on_sim(self, msg: JointState) -> None:
        if msg.name:
            self._joint_names = list(msg.name)
        t = _stamp_to_sec(msg.header.stamp)
        self._sim_window.push(t, _extract_full_state(msg))

        if self.get_parameter('real_source').value == 'lerobot' and self._lerobot_traj is not None:
            rel_t = t - (self._lerobot_origin or t)
            pos = self._lerobot_traj.lookup_nearest_position(rel_t, self._aligner.tolerance)
            if pos is not None:
                full = self._lerobot_traj.lookup_nearest(rel_t, self._aligner.tolerance)
                if full is not None:
                    self._real_window.push(t, full)

    def _on_real(self, msg: JointState) -> None:
        t = _stamp_to_sec(msg.header.stamp)
        self._real_window.push(t, _extract_full_state(msg))

    def _metrics_config(self) -> MetricsConfig:
        return MetricsConfig(
            kl_threshold_mean=self.get_parameter('kl_threshold_mean').value,
            mmd_threshold=self.get_parameter('mmd_threshold').value,
            mmd_p_value_alpha=self.get_parameter('mmd_p_value_alpha').value,
            mmd_permutation_count=self.get_parameter('mmd_permutation_count').value,
            mmd_max_samples=self.get_parameter('mmd_max_samples').value,
            mmd_gamma=self.get_parameter('mmd_gamma').value,
            use_kl=self.get_parameter('use_kl').value,
            use_mmd=self.get_parameter('use_mmd').value,
            min_samples=self.get_parameter('min_samples').value,
            align_tolerance_sec=self.get_parameter('align_tolerance_sec').value,
        )

    def _maybe_collect_baseline(self, errors: np.ndarray, now_sec: float) -> None:
        if self._baseline_ready:
            return

        if self._baseline_started_at is None:
            self._baseline_started_at = now_sec

        self._baseline_buffer.append(errors)
        duration = self.get_parameter('baseline_duration_sec').value
        if now_sec - self._baseline_started_at >= duration:
            self._baseline_errors = np.vstack(self._baseline_buffer)
            self._baseline_ready = True
            self.get_logger().info(
                f'Baseline captured: {len(self._baseline_errors)} error samples')

    def _compute_and_publish(self) -> None:
        metrics = DistributionMetrics()
        metrics.header.stamp = self.get_clock().now().to_msg()
        metrics.joint_names = self._joint_names
        metrics.window_duration_sec = self.get_parameter('window_duration_sec').value
        metrics.sample_count_sim = self._sim_window.count()
        metrics.sample_count_real = self._real_window.count()
        metrics.mmd_threshold = self.get_parameter('mmd_threshold').value
        metrics.detection_method = 'none'
        metrics.shift_detected = False

        min_samples = self.get_parameter('min_samples').value
        aligned_sim, aligned_real = self._aligner.align_windows(self._sim_window, self._real_window)
        n = len(aligned_sim)
        if n < min_samples:
            self._metrics_pub.publish(metrics)
            return

        n_dof = aligned_sim.shape[1] // 2
        errors = aligned_sim[:, :n_dof] - aligned_real[:, :n_dof]
        now_sec = _stamp_to_sec(metrics.header.stamp)
        self._maybe_collect_baseline(errors, now_sec)

        baseline = self._baseline_errors if self._baseline_ready else None
        result = compute_distribution_metrics(
            self._sim_window.get_timestamps(),
            self._sim_window.get_samples(),
            self._real_window.get_timestamps(),
            self._real_window.get_samples(),
            baseline_errors=baseline,
            cfg=self._metrics_config(),
        )

        metrics.kl_divergence_per_joint = result.kl_per_joint
        metrics.kl_divergence_mean = result.kl_mean
        metrics.mmd_statistic = result.mmd_statistic
        metrics.mmd_p_value = result.mmd_p_value
        metrics.shift_detected = result.shift_detected if self._baseline_ready else False
        metrics.detection_method = result.detection_method if self._baseline_ready else 'none'

        self._metrics_pub.publish(metrics)

        err_msg = JointState()
        err_msg.header = metrics.header
        err_msg.name = self._joint_names
        if len(errors) > 0:
            err_msg.position = errors[-1].tolist()
        self._error_pub.publish(err_msg)

    def _handle_reset_baseline(self, request, response):
        aligned_sim, aligned_real = self._aligner.align_windows(self._sim_window, self._real_window)
        if len(aligned_sim) == 0:
            response.success = False
            response.message = 'No aligned samples available for baseline reset.'
            return response

        n_dof = aligned_sim.shape[1] // 2
        self._baseline_errors = aligned_sim[:, :n_dof] - aligned_real[:, :n_dof]
        self._baseline_ready = True
        self._baseline_buffer = [self._baseline_errors]
        response.success = True
        response.message = f'Baseline reset with {len(self._baseline_errors)} aligned samples.'
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DistMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
