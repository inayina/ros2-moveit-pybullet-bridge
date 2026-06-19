"""Launch pybullet_bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('pybullet_bridge')
    config = PathJoinSubstitution([pkg, 'config', 'bridge_config.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('sim_mode', default_value='DIRECT'),
        DeclareLaunchArgument('enable_dual_source', default_value='false'),
        Node(
            package='pybullet_bridge',
            executable='bridge_node',
            name='pybullet_bridge',
            output='screen',
            parameters=[
                config,
                {
                    'sim_mode': LaunchConfiguration('sim_mode'),
                    'enable_dual_source': LaunchConfiguration('enable_dual_source'),
                    'urdf_path': PathJoinSubstitution([pkg, 'urdf', 'planar_2dof.urdf']),
                },
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', PathJoinSubstitution([pkg, 'urdf', 'planar_2dof.urdf'])]),
                    value_type=str,
                ),
            }],
        ),
    ])
