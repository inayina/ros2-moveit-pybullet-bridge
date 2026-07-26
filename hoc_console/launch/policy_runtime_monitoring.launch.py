"""Launch M3 validity-first monitor, risk, and read-only four-lane HOC."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_topic = LaunchConfiguration('sim_joint_topic')
    reference_topic = LaunchConfiguration('reference_joint_topic')
    calibration_id = LaunchConfiguration('calibration_id')
    safety_dry_run = LaunchConfiguration('safety_dry_run')

    monitor = Node(
        package='dist_monitor',
        executable='monitor_node',
        name='policy_runtime_dist_monitor',
        output='screen',
        parameters=[{
            'real_source': 'topic',
            'calibration_id': calibration_id,
        }],
        remappings=[
            ('/bridge/sim/joint_states', sim_topic),
            ('/bridge/real/joint_states', reference_topic),
        ],
    )
    risk = Node(
        package='risk_engine',
        executable='risk_node',
        name='policy_runtime_risk_engine',
        output='screen',
        parameters=[{'auto_e_stop_on_r3': False}],
    )
    safety_bridge = Node(
        package='risk_engine',
        executable='risk_to_safety_bridge',
        name='policy_runtime_risk_to_safety_bridge',
        output='screen',
        parameters=[{'dry_run': safety_dry_run}],
    )
    hoc = Node(
        package='hoc_console',
        executable='hoc_server',
        name='policy_runtime_hoc_server',
        output='screen',
        parameters=[{'serve_frontend': True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_joint_topic', default_value='/joint_states'
        ),
        DeclareLaunchArgument(
            'reference_joint_topic',
            default_value='/bridge/reference/joint_states',
        ),
        DeclareLaunchArgument(
            'calibration_id',
            default_value='',
            description=(
                'Required same-scene Panda calibration identity; empty keeps '
                'KL/W1/MMD unavailable.'
            ),
        ),
        DeclareLaunchArgument(
            'safety_dry_run',
            default_value='true',
            description='Publish proposed decisions without applying Hold/E-stop.',
        ),
        monitor,
        risk,
        safety_bridge,
        hoc,
    ])
