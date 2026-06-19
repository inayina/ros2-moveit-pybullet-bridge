"""Shared launch helpers for robot profile selection (Plan C)."""

from __future__ import annotations

from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from pybullet_bridge.robot_profiles import DEFAULT_CI_PROFILE, DEFAULT_PORTFOLIO_PROFILE


def declare_robot_launch_arg(*, default_profile: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        'robot',
        default_value=default_profile,
        description='Robot profile: planar_2dof (CI) | iiwa7 (portfolio)',
    )


def make_bridge_and_rsp_nodes(context, bridge_params: dict | None = None) -> list[Node]:
    """Build bridge_node + robot_state_publisher for LaunchConfiguration('robot')."""
    from pybullet_bridge.robot_profiles import resolve_profile_config

    robot = LaunchConfiguration('robot').perform(context)
    cfg = resolve_profile_config(robot)
    urdf_path = cfg['urdf_path']

    params = {
        'robot_profile': cfg['robot_profile'],
        'urdf_path': urdf_path,
        'home_positions': cfg['home_positions'],
        'end_effector_link': cfg['end_effector_link'],
    }
    if bridge_params:
        params.update(bridge_params)

    bridge = Node(
        package='pybullet_bridge',
        executable='bridge_node',
        name='pybullet_bridge',
        output='screen',
        parameters=[params],
    )
    rsp = Node(
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
    )
    return [bridge, rsp]
