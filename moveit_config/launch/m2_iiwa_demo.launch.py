"""M2 iiwa demo: KUKA iiwa7 + MoveIt2 + PyBullet closed loop (portfolio Plan C Phase 2)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from pybullet_bridge.robot_profiles import (
    IIWA_HOME,
    IIWA_JOINTS,
    resolve_urdf_robot_description,
)


def generate_launch_description():
    sim_mode = LaunchConfiguration('sim_mode')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    bridge_pkg = FindPackageShare('pybullet_bridge')
    moveit_pkg = FindPackageShare('moveit_config')
    urdf_path = PathJoinSubstitution([bridge_pkg, 'urdf', 'kuka_iiwa', 'model.urdf'])
    srdf_path = PathJoinSubstitution([moveit_pkg, 'srdf', 'kuka_iiwa.srdf'])

    robot_description_xml = resolve_urdf_robot_description('iiwa7', for_moveit=True)
    robot_description = {'robot_description': robot_description_xml}

    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command(['cat ', srdf_path]),
            value_type=str,
        ),
    }

    builder = MoveItConfigsBuilder('lbr_iiwa', package_name='moveit_config')
    builder._MoveItConfigsBuilder__moveit_configs.robot_description = robot_description
    builder._MoveItConfigsBuilder__moveit_configs.robot_description_semantic = (
        robot_description_semantic
    )

    moveit_config = (
        builder
        .robot_description_kinematics(file_path='config/iiwa_kinematics.yaml')
        .joint_limits(file_path='config/iiwa_joint_limits.yaml')
        .planning_pipelines(
            pipelines=['iiwa_ompl'],
            default_planning_pipeline='iiwa_ompl',
        )
        .trajectory_execution(file_path='config/iiwa_moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=False,
            publish_robot_description_semantic=False,
        )
        .to_moveit_configs()
    )

    moveit_params = moveit_config.to_dict()
    moveit_params.pop('robot_description', None)
    moveit_params.pop('robot_description_semantic', None)

    move_group_params = [
        moveit_params,
        robot_description,
        robot_description_semantic,
        {'use_sim_time': use_sim_time},
    ]

    rviz_params = [
        moveit_params,
        robot_description,
        robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        moveit_config.planning_pipelines,
        {'use_sim_time': use_sim_time},
    ]

    bridge_config = PathJoinSubstitution([bridge_pkg, 'config', 'bridge_config.yaml'])
    rviz_config = PathJoinSubstitution([moveit_pkg, 'config', 'iiwa_pybullet.rviz'])

    iiwa_home = list(IIWA_HOME)
    iiwa_joints = list(IIWA_JOINTS)

    return LaunchDescription([
        DeclareLaunchArgument('sim_mode', default_value='DIRECT'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('enable_dual_source', default_value='false'),

        Node(
            package='pybullet_bridge',
            executable='bridge_node',
            name='pybullet_bridge',
            output='screen',
            parameters=[
                bridge_config,
                {
                    'sim_mode': sim_mode,
                    'enable_dual_source': LaunchConfiguration('enable_dual_source'),
                    'robot_profile': 'iiwa7',
                    'urdf_path': urdf_path,
                    'home_positions': iiwa_home,
                    'end_effector_link': 'lbr_iiwa_link_7',
                },
            ],
        ),
        Node(
            package='pybullet_bridge',
            executable='trajectory_controller_node',
            name='arm_trajectory_controller',
            output='screen',
            parameters=[{
                'joint_names': iiwa_joints,
                'action_name': '/arm_controller/follow_joint_trajectory',
                'goal_tolerance': 0.08,
                'goal_time_margin_sec': 0.5,
                'execution_duration_scaling': 2.5,
                'succeed_on_duration': True,
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=move_group_params,
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='manipulation_actions',
                    executable='manipulation_node',
                    name='manipulation_actions',
                    output='screen',
                    parameters=[{
                        'use_moveit': True,
                        'joint_names': iiwa_joints,
                        'home_positions': iiwa_home,
                    }],
                ),
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='log',
                    arguments=['-d', rviz_config],
                    parameters=rviz_params,
                    condition=IfCondition(use_rviz),
                ),
            ],
        ),
    ])
