from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'manipulation_actions'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ina',
    maintainer_email='dev@example.com',
    description='Pick/Place manipulation action servers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulation_node = manipulation_actions.manipulation_node:main',
            'pick_place_demo = manipulation_actions.pick_place_demo:main',
        ],
    },
)
