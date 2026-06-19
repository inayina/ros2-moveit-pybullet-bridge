"""Robot profile registry for Plan C (2-DOF CI vs iiwa7 portfolio integration)."""

from __future__ import annotations

import os
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class RobotProfile:
    name: str
    urdf_relpath: str
    home_positions: tuple[float, ...]
    end_effector_link: str
    role: str


IIWA_HOME = (0.0, 0.785398, 0.0, -1.570796, 0.0, 1.570796, 0.0)

IIWA_JOINTS = (
    'lbr_iiwa_joint_1',
    'lbr_iiwa_joint_2',
    'lbr_iiwa_joint_3',
    'lbr_iiwa_joint_4',
    'lbr_iiwa_joint_5',
    'lbr_iiwa_joint_6',
    'lbr_iiwa_joint_7',
)

ROBOT_PROFILES: dict[str, RobotProfile] = {
    'planar_2dof': RobotProfile(
        name='planar_2dof',
        urdf_relpath='planar_2dof.urdf',
        home_positions=(0.8, -0.6),
        end_effector_link='tool0',
        role='CI / M1 smoke tests',
    ),
    'iiwa7': RobotProfile(
        name='iiwa7',
        urdf_relpath=os.path.join('kuka_iiwa', 'model.urdf'),
        home_positions=IIWA_HOME,
        end_effector_link='lbr_iiwa_link_7',
        role='Portfolio / episode-data-lab / M4 calibration',
    ),
}

DEFAULT_CI_PROFILE = 'planar_2dof'
DEFAULT_PORTFOLIO_PROFILE = 'iiwa7'


def list_profiles() -> list[str]:
    return list(ROBOT_PROFILES.keys())


def get_profile(name: str) -> RobotProfile:
    key = name.strip().lower()
    if key not in ROBOT_PROFILES:
        known = ', '.join(sorted(ROBOT_PROFILES))
        raise ValueError(f'Unknown robot profile {name!r}. Choose from: {known}')
    return ROBOT_PROFILES[key]


def _package_urdf_dir() -> str:
    share = get_package_share_directory('pybullet_bridge')
    return os.path.join(share, 'urdf')


def resolve_urdf_path(profile_name: str) -> str:
    """Return absolute URDF path for a profile (package share, then pybullet_data)."""
    profile = get_profile(profile_name)
    bundled = os.path.join(_package_urdf_dir(), profile.urdf_relpath)
    if os.path.isfile(bundled):
        return bundled

    if profile.name == 'iiwa7':
        try:
            import pybullet_data

            fallback = os.path.join(pybullet_data.getDataPath(), 'kuka_iiwa', 'model.urdf')
            if os.path.isfile(fallback):
                return fallback
        except ImportError:
            pass

    raise FileNotFoundError(
        f'URDF for profile {profile_name!r} not found at {bundled}',
    )


def resolve_urdf_robot_description(profile_name: str, *, for_moveit: bool = False) -> str:
    """Return URDF XML; rewrite mesh paths for MoveIt package:// resolution."""
    from pathlib import Path

    path = resolve_urdf_path(profile_name)
    text = Path(path).read_text(encoding='utf-8')
    if for_moveit and get_profile(profile_name).name == 'iiwa7':
        prefix = 'package://pybullet_bridge/urdf/kuka_iiwa/'
        text = text.replace('filename="meshes/', f'filename="{prefix}meshes/')
    return text


def resolve_profile_config(profile_name: str) -> dict:
    """Return bridge node config dict for the given profile."""
    profile = get_profile(profile_name)
    return {
        'robot_profile': profile.name,
        'urdf_path': resolve_urdf_path(profile.name),
        'home_positions': list(profile.home_positions),
        'end_effector_link': profile.end_effector_link,
    }
