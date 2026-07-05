"""Joint name normalization for iiwa7 / episode-data-lab LeRobot integration."""

from __future__ import annotations

# Keep in sync with pybullet_bridge.robot_profiles.IIWA_JOINTS
IIWA_ARM_JOINTS: tuple[str, ...] = (
    'lbr_iiwa_joint_1',
    'lbr_iiwa_joint_2',
    'lbr_iiwa_joint_3',
    'lbr_iiwa_joint_4',
    'lbr_iiwa_joint_5',
    'lbr_iiwa_joint_6',
    'lbr_iiwa_joint_7',
)

GRIPPER_JOINTS: tuple[str, ...] = (
    'left_finger_joint',
    'right_finger_joint',
)

PANDA_ARM_JOINTS: tuple[str, ...] = (
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
)

PANDA_GRIPPER_JOINTS: tuple[str, ...] = (
    'panda_leftfinger',
    'panda_rightfinger',
)

_LEGACY_ZERO = {f'joint_{index}': name for index, name in enumerate(IIWA_ARM_JOINTS)}
_LEGACY_ONE = {f'joint_{index + 1}': name for index, name in enumerate(IIWA_ARM_JOINTS)}
_LEGACY_PANDA_ZERO = {f'joint_{index}': name for index, name in enumerate(PANDA_ARM_JOINTS)}


def normalize_joint_names(names: list[str], robot: str = 'kuka_iiwa') -> list[str]:
    """Map legacy LeRobot names to URDF / bridge canonical names."""
    if not names:
        if robot == 'panda':
            return list(PANDA_ARM_JOINTS)
        return list(IIWA_ARM_JOINTS)

    if list(names) == list(PANDA_ARM_JOINTS):
        return list(names)

    if list(names) == list(IIWA_ARM_JOINTS):
        return list(names)

    if (robot == 'panda' or robot == 'panda_ee_delta_gripper_v0') and len(names) == len(PANDA_ARM_JOINTS) + len(PANDA_GRIPPER_JOINTS):
        arm = names[: len(PANDA_ARM_JOINTS)]
        gripper = names[len(PANDA_ARM_JOINTS) :]
        normalized_arm = normalize_joint_names(list(arm), robot=robot)
        if gripper == ['joint_7', 'joint_8'] or gripper == ['joint_8', 'joint_9']:
            return normalized_arm + list(PANDA_GRIPPER_JOINTS)
        if gripper == list(PANDA_GRIPPER_JOINTS):
            return normalized_arm + list(PANDA_GRIPPER_JOINTS)

    if (robot == 'kuka_iiwa' or robot == 'iiwa7') and len(names) == len(IIWA_ARM_JOINTS) + len(GRIPPER_JOINTS):
        arm = names[: len(IIWA_ARM_JOINTS)]
        gripper = names[len(IIWA_ARM_JOINTS) :]
        normalized_arm = normalize_joint_names(list(arm), robot=robot)
        if gripper == ['joint_7', 'joint_8'] or gripper == ['joint_8', 'joint_9']:
            return normalized_arm + list(GRIPPER_JOINTS)
        if gripper == list(GRIPPER_JOINTS):
            return normalized_arm + list(GRIPPER_JOINTS)

    if (robot == 'panda' or robot == 'panda_ee_delta_gripper_v0') and len(names) == len(PANDA_ARM_JOINTS):
        if all(name in _LEGACY_PANDA_ZERO for name in names):
            return [_LEGACY_PANDA_ZERO[name] for name in names]

    if robot == 'kuka_iiwa' or robot == 'iiwa7':
        if len(names) == len(IIWA_ARM_JOINTS):
            if all(name in _LEGACY_ZERO for name in names):
                return [_LEGACY_ZERO[name] for name in names]
            if all(name in _LEGACY_ONE for name in names):
                return [_LEGACY_ONE[name] for name in names]

    return list(names)


def reorder_joint_vector(
    names: list[str],
    values: list[float],
    target_names: list[str],
) -> list[float]:
    """Reorder a joint vector to match target_names; raises if a name is missing."""
    if not names or list(names) == list(target_names):
        return list(values)
    lookup = dict(zip(names, values))
    return [float(lookup[name]) for name in target_names]
