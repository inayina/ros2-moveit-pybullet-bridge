"""Unit tests for HOC command helpers."""

from __future__ import annotations

import pytest
import rclpy

from hoc_console.hoc_server import HocServerNode


@pytest.fixture(scope='module')
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_randomization_defaults_match_bridge_config(ros_context):
    node = HocServerNode()
    node.set_parameters([
        rclpy.parameter.Parameter('serve_frontend', rclpy.Parameter.Type.BOOL, False),
    ])
    try:
        cfg = node._build_randomization_config({'seed': 1, 'strength': 0.5})  # noqa: SLF001
        assert cfg.joint_damping_min == 0.0
        assert cfg.joint_damping_max == 0.5
        assert cfg.joint_friction_max == 0.3
        assert cfg.motor_strength_min == 0.85
        assert cfg.motor_strength_max == 1.15
        assert cfg.payload_mass_max == 0.5
    finally:
        node.destroy_node()
