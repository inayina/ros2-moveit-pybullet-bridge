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

_LEGACY_ZERO = {f'joint_{index}': name for index, name in enumerate(IIWA_ARM_JOINTS)}
_LEGACY_ONE = {f'joint_{index + 1}': name for index, name in enumerate(IIWA_ARM_JOINTS)}


def normalize_joint_names(names: list[str]) -> list[str]:
    """Map legacy LeRobot names to URDF / bridge canonical names."""
    if not names:
        return list(IIWA_ARM_JOINTS)

    if list(names) == list(IIWA_ARM_JOINTS):
        return list(names)

    if len(names) == len(IIWA_ARM_JOINTS) + len(GRIPPER_JOINTS):
        arm = names[: len(IIWA_ARM_JOINTS)]
        gripper = names[len(IIWA_ARM_JOINTS) :]
        normalized_arm = normalize_joint_names(list(arm))
        if gripper == ['joint_7', 'joint_8'] or gripper == ['joint_8', 'joint_9']:
            return normalized_arm + list(GRIPPER_JOINTS)
        if gripper == list(GRIPPER_JOINTS):
            return normalized_arm + list(GRIPPER_JOINTS)

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
