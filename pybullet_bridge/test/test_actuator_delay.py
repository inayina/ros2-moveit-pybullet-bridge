"""Unit tests for actuator delay buffer."""

from pybullet_bridge.actuator_delay import ActuatorDelayBuffer


def test_delay_returns_older_targets():
    buf = ActuatorDelayBuffer(delay_sec=0.05)
    buf.push(0.00, {'joint1': 0.0})
    buf.push(0.01, {'joint1': 0.1})
    buf.push(0.02, {'joint1': 0.2})

    targets, exec_time = buf.sample(0.05)
    assert targets['joint1'] == 0.0
    assert exec_time == 0.0


def test_zero_delay_uses_latest():
    buf = ActuatorDelayBuffer(delay_sec=0.0)
    buf.push(0.1, {'joint1': 0.5})
    targets, exec_time = buf.sample(0.2)
    assert targets['joint1'] == 0.5
    assert exec_time == 0.1
