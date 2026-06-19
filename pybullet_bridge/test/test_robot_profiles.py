"""Tests for robot profile registry."""

import os

import pytest

from pybullet_bridge.robot_profiles import (
    DEFAULT_CI_PROFILE,
    DEFAULT_PORTFOLIO_PROFILE,
    get_profile,
    resolve_profile_config,
    resolve_urdf_path,
)


def test_default_profiles():
    assert DEFAULT_CI_PROFILE == 'planar_2dof'
    assert DEFAULT_PORTFOLIO_PROFILE == 'iiwa7'


def test_planar_profile_paths():
    cfg = resolve_profile_config('planar_2dof')
    assert os.path.isfile(cfg['urdf_path'])
    assert len(cfg['home_positions']) == 2
    assert cfg['end_effector_link'] == 'tool0'


def test_iiwa_profile_paths():
    cfg = resolve_profile_config('iiwa7')
    assert os.path.isfile(cfg['urdf_path'])
    assert len(cfg['home_positions']) == 7
    assert cfg['end_effector_link'] == 'lbr_iiwa_link_7'


def test_unknown_profile_raises():
    with pytest.raises(ValueError):
        get_profile('unknown_robot')


@pytest.mark.skipif(
    not os.path.isfile(resolve_urdf_path('iiwa7')),
    reason='kuka_iiwa URDF not installed',
)
def test_iiwa_pybullet_load():
    pytest.importorskip('pybullet')
    import pybullet as p

    urdf = resolve_urdf_path('iiwa7')
    client = p.connect(p.DIRECT)
    robot = p.loadURDF(urdf, useFixedBase=True)
    assert p.getNumJoints(robot) >= 7
    p.disconnect()
