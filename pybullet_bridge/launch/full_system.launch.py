"""Launch the full Sim2Real monitoring system (scaffold)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_pkg = FindPackageShare('pybullet_bridge')
    dist_pkg = FindPackageShare('dist_monitor')
    risk_pkg = FindPackageShare('risk_engine')
    hoc_pkg = FindPackageShare('hoc_console')

    urdf_path = PathJoinSubstitution([bridge_pkg, 'urdf', 'planar_2dof.urdf'])
    sim_mode = LaunchConfiguration('sim_mode')
    enable_hoc = LaunchConfiguration('enable_hoc')

    return LaunchDescription([
        DeclareLaunchArgument('sim_mode', default_value='DIRECT'),
        DeclareLaunchArgument('enable_hoc', default_value='true'),

        Node(
            package='pybullet_bridge',
            executable='bridge_node',
            name='pybullet_bridge',
            output='screen',
            parameters=[
                PathJoinSubstitution([bridge_pkg, 'config', 'bridge_config.yaml']),
                {'sim_mode': sim_mode, 'urdf_path': urdf_path},
            ],
        ),
        Node(
            package='dist_monitor',
            executable='monitor_node',
            name='dist_monitor',
            output='screen',
            parameters=[PathJoinSubstitution([dist_pkg, 'config', 'thresholds.yaml'])],
        ),
        Node(
            package='risk_engine',
            executable='risk_node',
            name='risk_engine',
            output='screen',
            parameters=[PathJoinSubstitution([risk_pkg, 'config', 'risk_config.yaml'])],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([hoc_pkg, 'launch', 'hoc.launch.py'])),
            condition=IfCondition(enable_hoc),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['cat ', urdf_path]),
                    value_type=str,
                ),
            }],
        ),
    ])
