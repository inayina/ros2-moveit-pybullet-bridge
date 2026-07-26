"""Bounded M6 wiring smoke: mock policy runtime + real Safety bridge + HOC."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    evidence_dir = LaunchConfiguration('evidence_dir')
    websocket_port = LaunchConfiguration('websocket_port')
    camera_http_port = LaunchConfiguration('camera_http_port')

    hoc = Node(
        package='hoc_console', executable='hoc_server',
        name='policy_runtime_m6_hoc', output='screen',
        parameters=[{
            'serve_frontend': False,
            'websocket_port': websocket_port,
            'camera_http_port': camera_http_port,
            'report_output_dir': evidence_dir,
            'runtime_lane_stale_after_sec': 5.0,
        }],
    )
    bridge = Node(
        package='risk_engine', executable='risk_to_safety_bridge',
        name='policy_runtime_m6_safety_bridge', output='screen',
        parameters=[{'dry_run': False, 'healthy_recovery_count': 2}],
    )
    probe = Node(
        package='hoc_console', executable='m6_wiring_probe',
        name='policy_runtime_m6_wiring_probe', output='screen',
        parameters=[{'evidence_dir': evidence_dir, 'timeout_sec': 30.0}],
    )
    shutdown_on_probe_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=probe,
            on_exit=[EmitEvent(event=Shutdown(reason='M6 probe completed'))],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'evidence_dir', default_value='/tmp/policy_runtime_m6_wiring'
        ),
        DeclareLaunchArgument('websocket_port', default_value='18765'),
        DeclareLaunchArgument('camera_http_port', default_value='18766'),
        hoc, bridge, probe, shutdown_on_probe_exit,
    ])
