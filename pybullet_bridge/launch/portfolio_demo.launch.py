"""Portfolio demo: KUKA iiwa7 + dual-source + monitor + optional LeRobot Real replay."""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from robot_launch_utils import declare_robot_launch_arg

DEFAULT_LEROBOT_DATASET = (
    '/home/ina/robot-sim-lab/robot-arm-episode-data-lab/dataset/v1/lerobot_export'
)


def _launch_setup(context, *args, **kwargs):
    from pybullet_bridge.robot_profiles import resolve_profile_config

    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    real_source = LaunchConfiguration('real_source').perform(context)
    lerobot_path = LaunchConfiguration('lerobot_dataset_path').perform(context)

    bridge_pkg = FindPackageShare('pybullet_bridge').perform(context)
    dist_pkg = FindPackageShare('dist_monitor').perform(context)
    risk_pkg = FindPackageShare('risk_engine').perform(context)

    cfg = resolve_profile_config('iiwa7')
    urdf_path = cfg['urdf_path']
    bridge_yaml = os.path.join(bridge_pkg, 'config', 'bridge_config.yaml')
    thresholds_yaml = os.path.join(dist_pkg, 'config', 'thresholds.yaml')

    monitor_params = {
        'align_tolerance_sec': 0.05,
        'real_source': real_source,
        'lerobot_dataset_path': lerobot_path,
    }

    nodes = [
        Node(
            package='pybullet_bridge',
            executable='bridge_node',
            name='pybullet_bridge',
            output='screen',
            parameters=[
                bridge_yaml,
                {
                    'sim_mode': sim_mode,
                    'enable_dual_source': real_source == 'topic',
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
        Node(
            package='dist_monitor',
            executable='monitor_node',
            name='dist_monitor',
            output='screen',
            parameters=[thresholds_yaml, monitor_params],
        ),
        Node(
            package='risk_engine',
            executable='risk_node',
            name='risk_engine',
            output='screen',
            parameters=[os.path.join(risk_pkg, 'config', 'risk_config.yaml')],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='pybullet_bridge',
                    executable='iiwa_motion_demo',
                    name='iiwa_motion_demo',
                    output='screen',
                ),
            ],
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        declare_robot_launch_arg(default_profile='iiwa7'),
        DeclareLaunchArgument('sim_mode', default_value='GUI',
                              description='DIRECT for headless, GUI for portfolio recording'),
        DeclareLaunchArgument('real_source', default_value='topic',
                              description='topic (dual PyBullet) or lerobot'),
        DeclareLaunchArgument(
            'lerobot_dataset_path',
            default_value=DEFAULT_LEROBOT_DATASET,
            description='LeRobot export used when real_source:=lerobot',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
