from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dist_monitor'

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
    install_requires=['setuptools', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='ina',
    maintainer_email='dev@example.com',
    description='Distribution shift monitor using KL divergence and MMD.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_node = dist_monitor.monitor_node:main',
            'offline_compare = dist_monitor.offline_compare:main',
        ],
    },
)
