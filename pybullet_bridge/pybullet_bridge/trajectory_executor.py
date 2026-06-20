"""JointTrajectory time interpolation for PyBullet position control."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass
class _ActiveTrajectory:
    joint_names: list[str]
    points: list[JointTrajectoryPoint]
    start_time_sec: float


@dataclass
class TrajectoryExecutor:
    """Sample JointTrajectory messages against wall/sim elapsed time."""

    _active: _ActiveTrajectory | None = None
    _hold_positions: dict[str, float] = field(default_factory=dict)

    def set_hold_positions(self, joint_names: list[str], positions: list[float]) -> None:
        self._hold_positions = dict(zip(joint_names, positions))

    def set_trajectory(self, msg: JointTrajectory, start_time_sec: float) -> None:
        if not msg.joint_names or not msg.points:
            return
        # Deep-copy points: rclpy may reuse message buffers after callback returns
        copied_points: list[JointTrajectoryPoint] = []
        for pt in msg.points:
            copy_pt = JointTrajectoryPoint()
            copy_pt.positions = list(pt.positions)
            copy_pt.velocities = list(pt.velocities)
            copy_pt.accelerations = list(pt.accelerations)
            copy_pt.effort = list(pt.effort)
            copy_pt.time_from_start = copy.deepcopy(pt.time_from_start)
            copied_points.append(copy_pt)

        self._active = _ActiveTrajectory(
            joint_names=list(msg.joint_names),
            points=copied_points,
            start_time_sec=start_time_sec,
        )

    def clear(self) -> None:
        self._active = None

    @property
    def has_active_trajectory(self) -> bool:
        return self._active is not None

    def sample(self, now_sec: float, time_scale: float = 1.0) -> dict[str, float]:
        if self._active is None:
            return dict(self._hold_positions)

        scale = max(min(float(time_scale), 1.0), 0.05)
        elapsed = (now_sec - self._active.start_time_sec) * scale
        points = self._active.points
        first_t = self._point_time_sec(points[0])

        if elapsed <= 0.0:
            return dict(self._hold_positions)

        if elapsed < first_t:
            return dict(self._hold_positions)

        if elapsed >= self._point_time_sec(points[-1]):
            self._hold_positions = self._positions_from_point(points[-1])
            self._active = None
            return dict(self._hold_positions)

        for idx in range(len(points) - 1):
            t0 = self._point_time_sec(points[idx])
            t1 = self._point_time_sec(points[idx + 1])
            if t0 <= elapsed <= t1:
                alpha = 0.0 if t1 == t0 else (elapsed - t0) / (t1 - t0)
                p0 = self._positions_from_point(points[idx])
                p1 = self._positions_from_point(points[idx + 1])
                return {
                    name: p0[name] + alpha * (p1[name] - p0[name])
                    for name in self._active.joint_names
                }

        return self._positions_from_point(points[-1])

    def _positions_from_point(self, point: JointTrajectoryPoint) -> dict[str, float]:
        assert self._active is not None
        positions = list(point.positions)
        if len(positions) < len(self._active.joint_names):
            positions.extend([0.0] * (len(self._active.joint_names) - len(positions)))
        return dict(zip(self._active.joint_names, positions))

    @staticmethod
    def _point_time_sec(point: JointTrajectoryPoint) -> float:
        return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
