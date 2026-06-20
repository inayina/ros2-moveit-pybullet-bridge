"""Launch pybullet_bridge node."""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from pybullet_bridge.robot_launch_utils import declare_robot_launch_arg


def _launch_setup(context, *args, **kwargs):
    from pybullet_bridge.robot_profiles import resolve_profile_config

    robot = LaunchConfiguration('robot').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    enable_dual = LaunchConfiguration('enable_dual_source').perform(context) == 'true'
    enable_camera = LaunchConfiguration('enable_camera').perform(context) == 'true'

    bridge_pkg = FindPackageShare('pybullet_bridge').perform(context)
    cfg = resolve_profile_config(robot)
    urdf_path = cfg['urdf_path']
    config = os.path.join(bridge_pkg, 'config', 'bridge_config.yaml')

    return [
        Node(
            package='pybullet_bridge',
            executable='bridge_node',
            name='pybullet_bridge',
            output='screen',
            parameters=[
                config,
                {
                    'sim_mode': sim_mode,
                    'enable_dual_source': enable_dual,
                    'enable_camera': enable_camera,
                    'robot_profile': cfg['robot_profile'],
                    'urdf_path': urdf_path,
                    'home_positions': cfg['home_positions'],
                    'end_effector_link': cfg['end_effector_link'],
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
                    Command(['cat ', urdf_path]),
                    value_type=str,
                ),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        declare_robot_launch_arg(default_profile='iiwa7'),
        DeclareLaunchArgument('sim_mode', default_value='DIRECT'),
        DeclareLaunchArgument('enable_dual_source', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
