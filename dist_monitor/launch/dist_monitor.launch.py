"""Launch dist_monitor node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg = FindPackageShare('dist_monitor')
    return LaunchDescription([
        Node(
            package='dist_monitor',
            executable='monitor_node',
            name='dist_monitor',
            output='screen',
            parameters=[PathJoinSubstitution([pkg, 'config', 'thresholds.yaml'])],
        ),
    ])
