"""Robot profile registry for Plan C (2-DOF CI vs iiwa7 portfolio integration).

Panda is the current mainline profile. ``iiwa7`` is Legacy/KUKA and must not be
mixed with Panda handoff / training release claims.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RobotProfile:
    name: str
    urdf_relpath: str
    home_positions: tuple[float, ...]
    end_effector_link: str
    role: str
    joint_names: tuple[str, ...]
    lineage: str = 'panda_mainline'  # or 'legacy_kuka' / 'ci_fixture'


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
        joint_names=('joint1', 'joint2'),
        lineage='ci_fixture',
    ),
    'iiwa7': RobotProfile(
        name='iiwa7',
        urdf_relpath=os.path.join('kuka_iiwa', 'model.urdf'),
        home_positions=IIWA_HOME,
        end_effector_link='lbr_iiwa_link_7',
        role='Legacy KUKA / historical portfolio only',
        joint_names=IIWA_JOINTS,
        lineage='legacy_kuka',
    ),
    'panda': RobotProfile(
        name='panda',
        urdf_relpath=os.path.join('franka_panda', 'panda.urdf'),
        home_positions=(0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398),
        end_effector_link='panda_link7',
        role='Panda mainline / episode-data-lab / handoff',
        joint_names=(
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7',
        ),
        lineage='panda_mainline',
    ),
}

DEFAULT_CI_PROFILE = 'planar_2dof'
DEFAULT_PORTFOLIO_PROFILE = 'panda'


def list_profiles() -> list[str]:
    return list(ROBOT_PROFILES.keys())


def get_profile(name: str) -> RobotProfile:
    key = name.strip().lower()
    if key not in ROBOT_PROFILES:
        known = ', '.join(sorted(ROBOT_PROFILES))
        raise ValueError(f'Unknown robot profile {name!r}. Choose from: {known}')
    return ROBOT_PROFILES[key]


def _source_tree_urdf_dir() -> str | None:
    """Fallback when ament index is unavailable (editable / CPU-only tests)."""
    # .../pybullet_bridge/pybullet_bridge/robot_profiles.py → package root
    candidate = Path(__file__).resolve().parents[1] / 'urdf'
    if candidate.is_dir():
        return str(candidate)
    return None


def _package_urdf_dir() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory('pybullet_bridge')
        return os.path.join(share, 'urdf')
    except Exception:
        fallback = _source_tree_urdf_dir()
        if fallback is not None:
            return fallback
        raise


def resolve_urdf_path(profile_name: str) -> str:
    """Return absolute URDF path (ament share → source tree → pybullet_data)."""
    profile = get_profile(profile_name)
    try:
        bundled = os.path.join(_package_urdf_dir(), profile.urdf_relpath)
    except Exception:
        bundled = ''
    if bundled and os.path.isfile(bundled):
        return bundled

    source_dir = _source_tree_urdf_dir()
    if source_dir is not None:
        source_urdf = os.path.join(source_dir, profile.urdf_relpath)
        if os.path.isfile(source_urdf):
            return source_urdf

    if profile.name == 'iiwa7':
        try:
            import pybullet_data

            fallback = os.path.join(pybullet_data.getDataPath(), 'kuka_iiwa', 'model.urdf')
            if os.path.isfile(fallback):
                return fallback
        except ImportError:
            pass

    if profile.name == 'panda':
        try:
            import pybullet_data

            fallback = os.path.join(pybullet_data.getDataPath(), 'franka_panda', 'panda.urdf')
            if os.path.isfile(fallback):
                return fallback
        except ImportError:
            pass

    raise FileNotFoundError(
        f'URDF for profile {profile_name!r} not found under package share or source tree '
        f'(relpath={profile.urdf_relpath!r}). Build the workspace or use scripts/run_cpu_tests.sh.',
    )


def resolve_urdf_robot_description(profile_name: str, *, for_moveit: bool = False) -> str:
    """Return URDF XML; rewrite mesh paths for MoveIt package:// resolution."""
    from pathlib import Path

    path = resolve_urdf_path(profile_name)
    text = Path(path).read_text(encoding='utf-8')
    if for_moveit and get_profile(profile_name).name == 'iiwa7':
        prefix = 'package://pybullet_bridge/urdf/kuka_iiwa/'
        text = text.replace('filename="meshes/', f'filename="{prefix}meshes/')
    elif for_moveit and get_profile(profile_name).name == 'panda':
        prefix = 'package://pybullet_bridge/urdf/franka_panda/'
        text = text.replace('package://meshes/', f'{prefix}meshes/')
    return text


def resolve_profile_config(profile_name: str) -> dict:
    """Return bridge node config dict for the given profile."""
    profile = get_profile(profile_name)
    return {
        'robot_profile': profile.name,
        'urdf_path': resolve_urdf_path(profile.name),
        'home_positions': list(profile.home_positions),
        'end_effector_link': profile.end_effector_link,
        'joint_names': list(profile.joint_names),
    }
