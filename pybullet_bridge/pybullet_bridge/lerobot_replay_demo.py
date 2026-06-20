"""Publish a LeRobot episode as JointTrajectory for same-task bridge replay."""

from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pybullet_bridge.integration_paths import default_lerobot_export_path
from pybullet_bridge.robot_profiles import IIWA_JOINTS


class LeRobotReplayDemo(Node):
    """Replay one LeRobot episode to /bridge/command (same-task calibration)."""

    def __init__(self) -> None:
        super().__init__('lerobot_replay_demo')
        self.declare_parameter('lerobot_dataset_path', default_lerobot_export_path())
        self.declare_parameter('episode_index', 0)
        self.declare_parameter('publish_delay_sec', 1.5)
        self.declare_parameter('joint_names', IIWA_JOINTS)

        self._pub = self.create_publisher(JointTrajectory, '/bridge/command', 10)
        delay = float(self.get_parameter('publish_delay_sec').value)
        self._timer = self.create_timer(delay, self._publish_once)
        self._sent = False

    def _publish_once(self) -> None:
        if self._sent:
            return
        self._sent = True
        self._timer.cancel()

        dataset = Path(self.get_parameter('lerobot_dataset_path').value)
        episode_index = int(self.get_parameter('episode_index').value)
        joint_names = list(self.get_parameter('joint_names').value)

        repo_root = Path(__file__).resolve().parents[2]
        dist_path = repo_root.parent / 'dist_monitor'
        if str(dist_path) not in sys.path:
            sys.path.insert(0, str(dist_path))

        from dist_monitor.lerobot_loader import load_lerobot_dataset

        traj = load_lerobot_dataset(dataset, episode_indices=[episode_index])
        positions = traj.positions
        timestamps = traj.timestamps
        n_dof = min(positions.shape[1], len(joint_names))

        msg = JointTrajectory()
        msg.joint_names = joint_names[:n_dof]
        for i in range(len(positions)):
            t = float(timestamps[i])
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in positions[i, :n_dof]]
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1.0) * 1e9),
            )
            msg.points.append(point)

        self._pub.publish(msg)
        self.get_logger().info(
            f'Published LeRobot replay: episode={episode_index}, '
            f'points={len(msg.points)}, duration={timestamps[-1]:.1f}s, '
            f'dataset={dataset}',
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeRobotReplayDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
