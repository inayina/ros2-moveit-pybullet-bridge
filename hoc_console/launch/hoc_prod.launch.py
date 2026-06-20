"""Launch hoc_server with built frontend on :8080 (production demo)."""

import os

from ament_index_python.packages import get_package_share_directory
from hoc_console.http_static import resolve_frontend_source
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hoc_console')
    config = os.path.join(pkg_share, 'config', 'hoc_config.yaml')
    frontend_dir = resolve_frontend_source()
    if frontend_dir is None:
        raise RuntimeError(
            'HOC frontend not found. Rebuild hoc_console or set HOC_FRONTEND_DIR '
            'to hoc_console/frontend (must contain package.json).')

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
