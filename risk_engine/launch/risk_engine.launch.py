"""Launch risk_engine node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg = FindPackageShare('risk_engine')
    return LaunchDescription([
        Node(
            package='risk_engine',
            executable='risk_node',
            name='risk_engine',
            output='screen',
            parameters=[PathJoinSubstitution([pkg, 'config', 'risk_config.yaml'])],
        ),
    ])
