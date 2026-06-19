"""Publish demo trajectories when HOC experiments run."""

from __future__ import annotations

import math

from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


IIWA_JOINTS = [
    'lbr_iiwa_joint_1',
    'lbr_iiwa_joint_2',
    'lbr_iiwa_joint_3',
    'lbr_iiwa_joint_4',
    'lbr_iiwa_joint_5',
    'lbr_iiwa_joint_6',
    'lbr_iiwa_joint_7',
]

IIWA_HOME = [0.0, 0.4, 0.0, -1.2, 0.0, 1.6, 0.0]


def publish_iiwa_sweep(
    node: Node,
    *,
    amplitude: float = 0.28,
    duration_sec: float = 6.0,
    seed: int = 0,
    steps: int = 50,
) -> None:
    """Send one smooth trajectory to /bridge/command (visible in sim + metrics)."""
    pub = node.create_publisher(JointTrajectory, '/bridge/command', 10)
    joint_names = IIWA_JOINTS
    home = IIWA_HOME[: len(joint_names)]

    msg = JointTrajectory()
    msg.joint_names = joint_names

    for i in range(steps + 1):
        phase = 2.0 * math.pi * i / steps
        t = duration_sec * i / steps
        offsets = [
            amplitude * math.sin(phase + j * 0.35 + seed * 0.01)
            for j in range(len(joint_names))
        ]
        point = JointTrajectoryPoint()
        point.positions = [h + o for h, o in zip(home, offsets)]
        point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
        msg.points.append(point)

    pub.publish(msg)
    node.get_logger().info(
        f'Scenario motion published: {len(joint_names)} joints, {duration_sec:.1f}s')


def publish_iiwa_point_to_point(
    node: Node,
    *,
    seed: int = 0,
    strength: float = 0.5,
) -> None:
    """SC-01: moderate point-to-point sweep."""
    publish_iiwa_sweep(
        node,
        amplitude=0.18 + 0.12 * strength,
        duration_sec=6.0,
        seed=seed,
    )


def publish_iiwa_trajectory_track(
    node: Node,
    *,
    seed: int = 0,
    strength: float = 0.5,
) -> None:
    """SC-02: longer, smoother tracking motion."""
    publish_iiwa_sweep(
        node,
        amplitude=0.10 + 0.08 * strength,
        duration_sec=10.0,
        seed=seed,
        steps=80,
    )


def publish_iiwa_collision_avoid(
    node: Node,
    *,
    seed: int = 0,
    strength: float = 0.5,
) -> None:
    """SC-03: cautious small-amplitude motion."""
    publish_iiwa_sweep(
        node,
        amplitude=0.05 + 0.05 * strength,
        duration_sec=5.0,
        seed=seed,
        steps=40,
    )


def publish_iiwa_randomization_scan(
    node: Node,
    *,
    seed: int = 0,
    strength: float = 0.5,
) -> None:
    """SC-04: multiple short passes at different motion amplitudes."""
    for idx, scale in enumerate((0.6, 1.0, 1.4)):
        publish_iiwa_sweep(
            node,
            amplitude=(0.08 + 0.06 * strength) * scale,
            duration_sec=2.5,
            seed=seed + idx,
            steps=30,
        )


def publish_iiwa_estop_recovery(
    node: Node,
    *,
    seed: int = 0,
    strength: float = 0.5,
) -> None:
    """SC-05: motion segment used before/after e-stop recovery."""
    publish_iiwa_sweep(
        node,
        amplitude=0.12 + 0.10 * strength,
        duration_sec=4.0,
        seed=seed,
        steps=35,
    )
