"""Launch MoveIt2 move_group with PyBullet bridge (M2 entry point)."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    moveit_pkg = FindPackageShare('moveit_config')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([moveit_pkg, 'launch', 'm2_demo.launch.py'])),
        ),
    ])
