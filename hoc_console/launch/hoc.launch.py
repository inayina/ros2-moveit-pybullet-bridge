"""Launch hoc_server node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg = FindPackageShare('hoc_console')
    return LaunchDescription([
        Node(
            package='hoc_console',
            executable='hoc_server',
            name='hoc_server',
            output='screen',
            parameters=[PathJoinSubstitution([pkg, 'config', 'hoc_config.yaml'])],
        ),
    ])
