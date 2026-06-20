"""Unit tests for JointTrajectory interpolation."""

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pybullet_bridge.trajectory_executor import TrajectoryExecutor


def _make_trajectory() -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = ['j1', 'j2']
    p0 = JointTrajectoryPoint()
    p0.positions = [0.0, 0.0]
    p0.time_from_start = Duration(sec=1)
    p1 = JointTrajectoryPoint()
    p1.positions = [1.0, 2.0]
    p1.time_from_start = Duration(sec=3)
    msg.points = [p0, p1]
    return msg


def test_hold_positions_without_trajectory():
    executor = TrajectoryExecutor()
    executor.set_hold_positions(['j1'], [0.5])
    assert executor.sample(0.0) == {'j1': 0.5}
    assert not executor.has_active_trajectory


def test_before_start_uses_hold():
    executor = TrajectoryExecutor()
    executor.set_hold_positions(['j1', 'j2'], [0.0, 0.0])
    executor.set_trajectory(_make_trajectory(), start_time_sec=10.0)
    assert executor.sample(9.0) == {'j1': 0.0, 'j2': 0.0}


def test_interpolation_mid_segment():
    executor = TrajectoryExecutor()
    executor.set_hold_positions(['j1', 'j2'], [0.0, 0.0])
    executor.set_trajectory(_make_trajectory(), start_time_sec=0.0)
    # t=2s is halfway between 1s and 3s
    sampled = executor.sample(2.0)
    assert sampled['j1'] == 0.5
    assert sampled['j2'] == 1.0


def test_trajectory_completes_and_clears():
    executor = TrajectoryExecutor()
    executor.set_hold_positions(['j1', 'j2'], [0.0, 0.0])
    executor.set_trajectory(_make_trajectory(), start_time_sec=0.0)
    final = executor.sample(3.0)
    assert final == {'j1': 1.0, 'j2': 2.0}
    assert not executor.has_active_trajectory
    assert executor.sample(4.0) == {'j1': 1.0, 'j2': 2.0}


def test_empty_trajectory_ignored():
    executor = TrajectoryExecutor()
    executor.set_hold_positions(['j1'], [0.3])
    msg = JointTrajectory()
    msg.joint_names = []
    msg.points = []
    executor.set_trajectory(msg, start_time_sec=0.0)
    assert executor.sample(0.0) == {'j1': 0.3}


def test_degraded_time_scale_halves_progress():
    executor = TrajectoryExecutor()
    executor.set_hold_positions(['j1', 'j2'], [0.0, 0.0])
    executor.set_trajectory(_make_trajectory(), start_time_sec=0.0)
    normal = executor.sample(2.0, time_scale=1.0)
    degraded = executor.sample(2.0, time_scale=0.5)
    assert normal['j1'] == 0.5
    assert degraded['j1'] == 0.0
    assert degraded == {'j1': 0.0, 'j2': 0.0}
