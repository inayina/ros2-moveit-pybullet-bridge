"""Tests for joint name normalization (iiwa7 / LeRobot legacy aliases)."""

from dist_monitor.joint_names import (
    IIWA_ARM_JOINTS,
    GRIPPER_JOINTS,
    normalize_joint_names,
    reorder_joint_vector,
)


def test_normalize_legacy_zero_indexed():
    legacy = [f'joint_{index}' for index in range(7)]
    assert normalize_joint_names(legacy) == list(IIWA_ARM_JOINTS)


def test_normalize_legacy_one_indexed():
    legacy = [f'joint_{index + 1}' for index in range(7)]
    assert normalize_joint_names(legacy) == list(IIWA_ARM_JOINTS)


def test_normalize_canonical_is_idempotent():
    assert normalize_joint_names(list(IIWA_ARM_JOINTS)) == list(IIWA_ARM_JOINTS)


def test_reorder_joint_vector():
    names = ['b', 'a']
    values = [2.0, 1.0]
    assert reorder_joint_vector(names, values, ['a', 'b']) == [1.0, 2.0]


def test_normalize_gripper_legacy():
    legacy = [f'joint_{index}' for index in range(9)]
    expected = list(IIWA_ARM_JOINTS) + list(GRIPPER_JOINTS)
    assert normalize_joint_names(legacy) == expected
