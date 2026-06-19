"""Launch Pick/Place action servers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('manipulation_actions'),
        'config',
        'manipulation_config.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_moveit', default_value='true'),
        Node(
            package='manipulation_actions',
            executable='manipulation_node',
            name='manipulation_actions',
            output='screen',
            parameters=[
                config,
                {'use_moveit': LaunchConfiguration('use_moveit')},
            ],
        ),
    ])
