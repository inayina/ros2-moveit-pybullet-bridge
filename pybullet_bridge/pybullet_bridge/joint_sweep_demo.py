"""Demo node: send a sinusoidal joint trajectory to pybullet_bridge."""

from __future__ import annotations

import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointSweepDemo(Node):
    """Publish a simple two-joint sweep trajectory for M1 validation."""

    def __init__(self) -> None:
        super().__init__('joint_sweep_demo')
        self.declare_parameter('joint_names', ['joint1', 'joint2'])
        self.declare_parameter('amplitude', 0.8)
        self.declare_parameter('duration_sec', 4.0)
        self.declare_parameter('publish_delay_sec', 2.0)

        self._pub = self.create_publisher(JointTrajectory, '/bridge/command', 10)
        delay = self.get_parameter('publish_delay_sec').value
        self._timer = self.create_timer(delay, self._publish_once)
        self._sent = False

    def _publish_once(self) -> None:
        if self._sent:
            return
        self._sent = True
        self._timer.cancel()

        joint_names = list(self.get_parameter('joint_names').value)
        amplitude = self.get_parameter('amplitude').value
        duration = self.get_parameter('duration_sec').value
        steps = 40

        msg = JointTrajectory()
        msg.joint_names = joint_names

        for i in range(steps + 1):
            phase = math.pi * i / steps
            t = duration * i / steps
            point = JointTrajectoryPoint()
            point.positions = [
                amplitude * math.sin(phase),
                amplitude * math.cos(phase),
            ]
            point.time_from_start = Duration(
                sec=int(t), nanosec=int((t % 1.0) * 1e9))
            msg.points.append(point)

        self._pub.publish(msg)
        self.get_logger().info(
            f'Published sweep trajectory: joints={joint_names}, '
            f'duration={duration}s, points={len(msg.points)}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointSweepDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
