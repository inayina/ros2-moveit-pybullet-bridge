"""Launch hoc_server and Vite frontend dev server."""

import os

from ament_index_python.packages import get_package_share_directory
from hoc_console.http_static import resolve_frontend_source
from hoc_console.launch_preflight import require_free_ports
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    require_free_ports((8765, 5173), label='hoc.launch.py')
    pkg_share = get_package_share_directory('hoc_console')
    config = os.path.join(pkg_share, 'config', 'hoc_config.yaml')

    frontend_dir = resolve_frontend_source()
    if frontend_dir is None:
        raise RuntimeError(
            'HOC frontend not found. Rebuild hoc_console or set HOC_FRONTEND_DIR '
            'to hoc_console/frontend (must contain package.json).')

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
        LogInfo(msg='HOC console: open http://<host>:5173 — WebSocket via /hoc-ws (needs hoc_server on :8765)'),
    ])
