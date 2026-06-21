from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pybullet_bridge'


def _urdf_data_files():
    files = []
    for dirpath, _, filenames in os.walk('urdf'):
        for filename in filenames:
            path = os.path.join(dirpath, filename)
            files.append((os.path.join('share', package_name, dirpath), [path]))
    return files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        *_urdf_data_files(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ina',
    maintainer_email='dev@example.com',
    description='ROS 2 PyBullet bridge with dual-source simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = pybullet_bridge.bridge_node:main',
            'joint_sweep_demo = pybullet_bridge.joint_sweep_demo:main',
            'iiwa_motion_demo = pybullet_bridge.iiwa_motion_demo:main',
            'lerobot_replay_demo = pybullet_bridge.lerobot_replay_demo:main',
            'policy_runner = pybullet_bridge.learning.policy_runner:main',
            'trajectory_controller_node = pybullet_bridge.trajectory_controller_node:main',
        ],
    },
)
