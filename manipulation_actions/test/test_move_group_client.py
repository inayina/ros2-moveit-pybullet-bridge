"""Unit tests for MoveGroup constraint builder."""

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

from manipulation_actions.move_group_client import build_pose_constraints


def test_build_pose_constraints_populates_position_and_orientation():
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose = Pose(
        position=Point(x=0.4, y=0.0, z=0.3),
        orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
    )
    constraints = build_pose_constraints(pose, 'lbr_iiwa_link_7')
    assert len(constraints.position_constraints) == 1
    assert len(constraints.orientation_constraints) == 1
    assert constraints.position_constraints[0].link_name == 'lbr_iiwa_link_7'
    assert constraints.orientation_constraints[0].link_name == 'lbr_iiwa_link_7'
