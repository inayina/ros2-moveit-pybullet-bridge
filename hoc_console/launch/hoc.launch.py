"""Launch hoc_server and Vite frontend dev server."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hoc_console')
    config = os.path.join(pkg_share, 'config', 'hoc_config.yaml')

    frontend_dir = Path(__file__).resolve().parents[1] / 'frontend'

    vite = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'npm install && npm run dev -- --host 0.0.0.0 --port 5173',
        ],
        cwd=str(frontend_dir),
        output='screen',
        additional_env={'FORCE_COLOR': '1'},
    )

    hoc_server = Node(
        package='hoc_console',
        executable='hoc_server',
        name='hoc_server',
        output='screen',
        parameters=[
            config,
            {'serve_frontend': False},
        ],
    )

    return LaunchDescription([
        hoc_server,
        vite,
        LogInfo(msg='HOC console: open http://localhost:5173 (WebSocket ws://localhost:8765)'),
    ])
