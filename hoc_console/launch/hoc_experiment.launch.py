"""One-click local experiment: portfolio demo + HOC console."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bridge_share = get_package_share_directory('pybullet_bridge')
    hoc_share = get_package_share_directory('hoc_console')

    portfolio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, 'launch', 'portfolio_demo.launch.py'),
        ),
        launch_arguments={
            'sim_mode': LaunchConfiguration('sim_mode'),
            'real_source': LaunchConfiguration('real_source'),
        }.items(),
    )

    hoc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hoc_share, 'launch', 'hoc.launch.py'),
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='DIRECT',
            description='DIRECT: embedded camera in HOC; GUI: also open PyBullet window',
        ),
        DeclareLaunchArgument('real_source', default_value='topic'),
        portfolio,
        hoc,
        LogInfo(msg='Local experiment: http://localhost:5173 (camera + metrics in HOC)'),
    ])
