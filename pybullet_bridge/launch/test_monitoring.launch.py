"""Integration test launch: bridge (sim-only) + dist_monitor + risk_engine."""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from robot_launch_utils import declare_robot_launch_arg


def _launch_setup(context, *args, **kwargs):
    from pybullet_bridge.robot_profiles import resolve_profile_config

    robot = LaunchConfiguration('robot').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)

    bridge_pkg = FindPackageShare('pybullet_bridge').perform(context)
    dist_pkg = FindPackageShare('dist_monitor').perform(context)
    risk_pkg = FindPackageShare('risk_engine').perform(context)

    cfg = resolve_profile_config(robot)
    urdf_path = cfg['urdf_path']

    return [
        Node(
            package='pybullet_bridge',
            executable='bridge_node',
            name='pybullet_bridge',
            output='screen',
            parameters=[
                os.path.join(bridge_pkg, 'config', 'bridge_config.yaml'),
                {
                    'sim_mode': sim_mode,
                    'enable_dual_source': False,
                    'robot_profile': cfg['robot_profile'],
                    'urdf_path': urdf_path,
                    'home_positions': cfg['home_positions'],
                    'end_effector_link': cfg['end_effector_link'],
                },
            ],
        ),
        Node(
            package='dist_monitor',
            executable='monitor_node',
            name='dist_monitor',
            output='screen',
            parameters=[os.path.join(dist_pkg, 'config', 'thresholds.yaml')],
        ),
        Node(
            package='risk_engine',
            executable='risk_node',
            name='risk_engine',
            output='screen',
            parameters=[os.path.join(risk_pkg, 'config', 'risk_config.yaml')],
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
        declare_robot_launch_arg(default_profile='planar_2dof'),
        DeclareLaunchArgument('sim_mode', default_value='DIRECT'),
        OpaqueFunction(function=_launch_setup),
    ])
