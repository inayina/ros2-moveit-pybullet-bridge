"""Ensure launch files import shared helpers from the installed Python package."""

from launch.launch_description_sources.python_launch_file_utilities import (
    get_launch_description_from_python_launch_file,
)


def test_m1_launch_file_imports():
    from ament_index_python.packages import get_package_share_directory
    import os

    launch_path = os.path.join(
        get_package_share_directory('pybullet_bridge'),
        'launch',
        'm1_demo.launch.py',
    )
    desc = get_launch_description_from_python_launch_file(launch_path)
    assert desc is not None
