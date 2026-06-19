"""M1 one-shot demo: planar 2-DOF bridge + state publisher + sweep trajectory."""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
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
                    'enable_dual_source': False,
                    'robot_profile': cfg['robot_profile'],
                    'urdf_path': urdf_path,
                    'home_positions': cfg['home_positions'],
                    'end_effector_link': cfg['end_effector_link'],
                    'watchdog_timeout_ms': 5000,
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
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='pybullet_bridge',
                    executable='joint_sweep_demo',
                    name='joint_sweep_demo',
                    output='screen',
                    parameters=[{
                        'joint_names': ['joint1', 'joint2'],
                        'publish_delay_sec': 3.0,
                    }],
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        declare_robot_launch_arg(default_profile='planar_2dof'),
        DeclareLaunchArgument('sim_mode', default_value='DIRECT'),
        OpaqueFunction(function=_launch_setup),
    ])
