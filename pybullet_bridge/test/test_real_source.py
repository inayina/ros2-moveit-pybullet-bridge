"""Unit tests for Real-Source domain randomization (requires PyBullet)."""

from __future__ import annotations

import os

import pytest

pytest.importorskip('pybullet')

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from pybullet_bridge.real_source import RealSource, RealSourceConfig  # noqa: E402


def _urdf_path() -> str:
    share = get_package_share_directory('pybullet_bridge')
    return os.path.join(share, 'urdf', 'planar_2dof.urdf')


def test_real_source_resamples_on_reset():
    real = RealSource(RealSourceConfig(
        urdf_path=_urdf_path(),
        random_seed=11,
        randomization_strength=1.0,
    ))
    if not real.initialize():
        pytest.skip('PyBullet init failed')

    first = real.latest_sample
    assert first is not None
    real.reset()
    second = real.latest_sample
    assert second is not None
    assert second.episode_seed != first.episode_seed
    real.shutdown()


def test_real_source_applies_actuator_delay():
    real = RealSource(RealSourceConfig(
        urdf_path=_urdf_path(),
        random_seed=17,
        time_delay_range_ms=(20.0, 20.0),
    ))
    if not real.initialize():
        pytest.skip('PyBullet init failed')

    assert real.latest_sample is not None
    assert real.latest_sample.actuator_delay_ms == pytest.approx(20.0)
    real.set_position_targets_by_name({'joint1': 0.5, 'joint2': 0.3})
    real.step(0.0)
    assert real.last_execution_sim_time <= 0.0
    real.shutdown()
