"""Unit tests for pose offset helpers."""

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

from manipulation_actions.pose_utils import copy_with_z_offset, offset_along_approach


def _pose(x=0.0, y=0.0, z=0.5) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = 'base'
    msg.pose = Pose(
        position=Point(x=x, y=y, z=z),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    return msg


def test_offset_along_approach_identity_orientation_moves_z():
    pose = _pose(z=0.5)
    shifted = offset_along_approach(pose, 0.1)
    assert abs(shifted.pose.position.z - 0.6) < 1e-6


def test_copy_with_z_offset():
    pose = _pose(z=0.4)
    lifted = copy_with_z_offset(pose, 0.05)
    assert abs(lifted.pose.position.z - 0.45) < 1e-6
    assert lifted.pose.position.x == pose.pose.position.x
