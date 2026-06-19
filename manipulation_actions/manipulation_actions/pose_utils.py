"""Pose helpers for pick/place motion phases."""

from __future__ import annotations

import copy

from geometry_msgs.msg import PoseStamped


def _quat_rotate_local_z(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Return the pose +Z axis direction in the parent frame."""
    # R * [0, 0, 1] for unit quaternion
    x = 2.0 * (qx * qz + qw * qy)
    y = 2.0 * (qy * qz - qw * qx)
    z = 1.0 - 2.0 * (qx * qx + qy * qy)
    return x, y, z


def offset_along_approach(pose: PoseStamped, offset_m: float) -> PoseStamped:
    """Translate along the pose +Z axis (tool approach direction)."""
    out = copy.deepcopy(pose)
    axis = _quat_rotate_local_z(
        out.pose.orientation.x,
        out.pose.orientation.y,
        out.pose.orientation.z,
        out.pose.orientation.w,
    )
    out.pose.position.x += offset_m * axis[0]
    out.pose.position.y += offset_m * axis[1]
    out.pose.position.z += offset_m * axis[2]
    return out


def copy_with_z_offset(pose: PoseStamped, dz: float) -> PoseStamped:
    """Translate along the parent-frame Z axis."""
    out = copy.deepcopy(pose)
    out.pose.position.z += dz
    return out
