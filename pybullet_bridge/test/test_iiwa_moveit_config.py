"""Tests for KUKA iiwa MoveIt / PyBullet joint consistency."""

from pathlib import Path
import xml.etree.ElementTree as ET

from pybullet_bridge.robot_profiles import IIWA_JOINTS, resolve_urdf_path, resolve_urdf_robot_description

REPO_ROOT = Path(__file__).resolve().parents[2]
MOVEIT_ROOT = REPO_ROOT / 'moveit_config'


def test_kuka_iiwa_srdf_parses():
    srdf = MOVEIT_ROOT / 'srdf' / 'kuka_iiwa.srdf'
    root = ET.parse(srdf).getroot()
    assert root.attrib['name'] == 'lbr_iiwa'
    groups = [g.attrib['name'] for g in root.findall('group')]
    assert 'manipulator' in groups


def test_iiwa_moveit_controller_joints_match_profile():
    controllers = MOVEIT_ROOT / 'config' / 'iiwa_moveit_controllers.yaml'
    text = controllers.read_text(encoding='utf-8')
    for joint in IIWA_JOINTS:
        assert joint in text


def test_moveit_urdf_rewrites_mesh_paths():
    xml = resolve_urdf_robot_description('iiwa7', for_moveit=True)
    assert 'package://pybullet_bridge/urdf/kuka_iiwa/meshes/' in xml
    assert 'filename="meshes/' not in xml


def test_pybullet_iiwa_joint_order():
    pytest = __import__('pytest')
    pytest.importorskip('pybullet')
    import pybullet as p

    client = p.connect(p.DIRECT)
    robot = p.loadURDF(resolve_urdf_path('iiwa7'), useFixedBase=True)
    discovered = []
    for idx in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, idx)
        if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            discovered.append(info[1].decode())
    p.disconnect()
    assert discovered == list(IIWA_JOINTS)
