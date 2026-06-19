"""Demo node: sinusoidal motion for KUKA iiwa7 (portfolio recording)."""

from __future__ import annotations

import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pybullet_bridge.robot_profiles import IIWA_HOME, IIWA_JOINTS


class IiwaMotionDemo(Node):
    """Publish a smooth 7-DOF sweep for portfolio / integration demos."""

    def __init__(self) -> None:
        super().__init__('iiwa_motion_demo')
        self.declare_parameter('joint_names', IIWA_JOINTS)
        self.declare_parameter('amplitude', 0.25)
        self.declare_parameter('duration_sec', 6.0)
        self.declare_parameter('publish_delay_sec', 1.0)

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
        amplitude = float(self.get_parameter('amplitude').value)
        duration = float(self.get_parameter('duration_sec').value)
        steps = 50
        home = IIWA_HOME[:len(joint_names)]

        msg = JointTrajectory()
        msg.joint_names = joint_names

        for i in range(steps + 1):
            phase = 2.0 * math.pi * i / steps
            t = duration * i / steps
            offsets = [
                amplitude * math.sin(phase + j * 0.4)
                for j in range(len(joint_names))
            ]
            point = JointTrajectoryPoint()
            point.positions = [h + o for h, o in zip(home, offsets)]
            point.time_from_start = Duration(
                sec=int(t), nanosec=int((t % 1.0) * 1e9))
            msg.points.append(point)

        self._pub.publish(msg)
        self.get_logger().info(
            f'Published iiwa motion: joints={len(joint_names)}, '
            f'duration={duration}s, points={len(msg.points)}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IiwaMotionDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
