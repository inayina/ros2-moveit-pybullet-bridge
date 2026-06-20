"""Tests for direct PyBullet simulation source behavior."""

import pytest

from pybullet_bridge.robot_profiles import resolve_profile_config
from pybullet_bridge.sim_source import SimSource, SimSourceConfig


def test_sim_source_initializes_to_profile_home_positions():
    pytest.importorskip('pybullet')

    cfg = resolve_profile_config('planar_2dof')
    sim = SimSource(
        SimSourceConfig(
            urdf_path=cfg['urdf_path'],
            use_gui=False,
            home_positions=cfg['home_positions'],
        )
    )

    try:
        assert sim.initialize()
        snapshot = sim.read_state()
        assert snapshot.names == ['joint1', 'joint2']
        assert snapshot.positions == pytest.approx(cfg['home_positions'], abs=1e-6)
    finally:
        sim.shutdown()
