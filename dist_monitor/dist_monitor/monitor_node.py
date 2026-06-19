"""ROS 2 node for KL/MMD distribution shift monitoring."""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from bridge_monitor_msgs.msg import DistributionMetrics

from dist_monitor.kl_divergence import kl_per_joint
from dist_monitor.mmd_test import median_heuristic_gamma, permutation_test
from dist_monitor.sliding_window import SlidingWindow


class DistMonitorNode(Node):
    """Subscribe to dual-source joint states and publish distribution metrics."""

    def __init__(self) -> None:
        super().__init__('dist_monitor')

        self.declare_parameter('window_duration_sec', 5.0)
        self.declare_parameter('update_frequency_hz', 10.0)
        self.declare_parameter('kl_threshold_mean', 0.15)
        self.declare_parameter('mmd_threshold', 0.05)
        self.declare_parameter('mmd_permutation_count', 100)
        self.declare_parameter('mmd_gamma', 1.0)
        self.declare_parameter('use_kl', True)
        self.declare_parameter('use_mmd', True)
        self.declare_parameter('min_samples', 50)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._sim_window = SlidingWindow(
            self.get_parameter('window_duration_sec').value)
        self._real_window = SlidingWindow(
            self.get_parameter('window_duration_sec').value)
        self._baseline_errors: list = []
        self._joint_names: list[str] = []

        self.create_subscription(
            JointState, '/bridge/sim/joint_states', self._on_sim, sensor_qos)
        self.create_subscription(
            JointState, '/bridge/real/joint_states', self._on_real, sensor_qos)

        self._metrics_pub = self.create_publisher(
            DistributionMetrics, '/monitor/distribution_metrics', 10)
        self._error_pub = self.create_publisher(
            JointState, '/monitor/tracking_error', sensor_qos)

        self.create_service(Trigger, '/monitor/reset_baseline', self._handle_reset_baseline)

        hz = self.get_parameter('update_frequency_hz').value
        self._timer = self.create_timer(1.0 / hz, self._compute_and_publish)
        self.get_logger().info('dist_monitor node started (scaffold).')

    def _on_sim(self, msg: JointState) -> None:
        if msg.name:
            self._joint_names = list(msg.name)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._sim_window.push(t, np.array(msg.position))

    def _on_real(self, msg: JointState) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._real_window.push(t, np.array(msg.position))

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
        if metrics.sample_count_sim < min_samples or metrics.sample_count_real < min_samples:
            self._metrics_pub.publish(metrics)
            return

        sim_arr = self._sim_window.get_array()
        real_arr = self._real_window.get_array()
        n = min(len(sim_arr), len(real_arr))
        if n == 0:
            self._metrics_pub.publish(metrics)
            return

        errors = sim_arr[:n] - real_arr[:n]

        if self.get_parameter('use_kl').value:
            baseline = errors if not self._baseline_errors else np.array(self._baseline_errors)
            kl_vals = kl_per_joint(baseline[:n], errors)
            metrics.kl_divergence_per_joint = kl_vals
            metrics.kl_divergence_mean = float(np.mean(kl_vals)) if kl_vals else 0.0

        if self.get_parameter('use_mmd').value:
            gamma = self.get_parameter('mmd_gamma').value
            if gamma <= 0:
                gamma = median_heuristic_gamma(sim_arr[:n], real_arr[:n])
            mmd_stat, p_val = permutation_test(
                sim_arr[:n], real_arr[:n], gamma,
                self.get_parameter('mmd_permutation_count').value)
            metrics.mmd_statistic = mmd_stat
            metrics.mmd_p_value = p_val

        kl_flag = (
            self.get_parameter('use_kl').value
            and metrics.kl_divergence_mean > self.get_parameter('kl_threshold_mean').value
        )
        mmd_flag = (
            self.get_parameter('use_mmd').value
            and metrics.mmd_p_value < 0.05
            and metrics.mmd_statistic > metrics.mmd_threshold
        )
        if kl_flag and mmd_flag:
            metrics.shift_detected = True
            metrics.detection_method = 'both'
        elif kl_flag:
            metrics.shift_detected = True
            metrics.detection_method = 'kl'
        elif mmd_flag:
            metrics.shift_detected = True
            metrics.detection_method = 'mmd'

        self._metrics_pub.publish(metrics)

        err_msg = JointState()
        err_msg.header = metrics.header
        err_msg.name = self._joint_names
        if len(errors) > 0:
            err_msg.position = errors[-1].tolist()
        self._error_pub.publish(err_msg)

    def _handle_reset_baseline(self, request, response):
        sim_arr = self._sim_window.get_array()
        real_arr = self._real_window.get_array()
        n = min(len(sim_arr), len(real_arr))
        if n > 0:
            self._baseline_errors = (sim_arr[:n] - real_arr[:n]).tolist()
        response.success = True
        response.message = f'Baseline reset with {n} samples.'
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
