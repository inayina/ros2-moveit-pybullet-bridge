"""Launch hoc_server with built frontend on :8080 (production demo)."""

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

    build_frontend = ExecuteProcess(
        cmd=['bash', '-c', 'npm install && npm run build'],
        cwd=str(frontend_dir),
        output='screen',
    )

    hoc_server = Node(
        package='hoc_console',
        executable='hoc_server',
        name='hoc_server',
        output='screen',
        parameters=[config, {'serve_frontend': True}],
    )

    return LaunchDescription([
        build_frontend,
        hoc_server,
        LogInfo(msg='HOC production UI: http://<host>:8080 — WebSocket ws://<host>:8080/hoc-ws'),
    ])
