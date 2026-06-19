#!/usr/bin/env python3
"""Verify PyBullet iiwa joint order matches MoveIt / robot_profiles."""

from __future__ import annotations

import sys

from pybullet_bridge.robot_profiles import IIWA_JOINTS, resolve_urdf_path


def main() -> int:
    pybullet = __import__('pybullet')
    p = pybullet
    client = p.connect(p.DIRECT)
    urdf = resolve_urdf_path('iiwa7')
    robot = p.loadURDF(urdf, useFixedBase=True)

    discovered = []
    for idx in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, idx)
        if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            discovered.append(info[1].decode())

    p.disconnect()

    if discovered != list(IIWA_JOINTS):
        print('[FAIL] Joint order mismatch')
        print('  PyBullet:', discovered)
        print('  Expected:', list(IIWA_JOINTS))
        return 1

    print('[PASS] iiwa7 joint names/order consistent')
    print('  joints:', ', '.join(discovered))
    print('  urdf:', urdf)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
