"""M2 demo: UR5 + MoveIt2 + PyBullet closed loop."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

UR5_HOME = [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0]
UR5_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    sim_mode = LaunchConfiguration('sim_mode')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('ur_description'),
                    'urdf',
                    'ur.urdf.xacro',
                ]),
                ' ur_type:=',
                ur_type,
                ' name:=ur tf_prefix:="" force_abs_paths:=true',
            ]),
            value_type=str,
        ),
    }

    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('ur_moveit_config'),
                    'srdf',
                    'ur.srdf.xacro',
                ]),
                ' name:=ur',
            ]),
            value_type=str,
        ),
    }

    builder = MoveItConfigsBuilder('ur', package_name='ur_moveit_config')
    builder._MoveItConfigsBuilder__moveit_configs.robot_description = robot_description
    builder._MoveItConfigsBuilder__moveit_configs.robot_description_semantic = (
        robot_description_semantic
    )

    moveit_config = (
        builder
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
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

    bridge_pkg = FindPackageShare('pybullet_bridge')
    moveit_pkg = FindPackageShare('moveit_config')
    bridge_config = PathJoinSubstitution([bridge_pkg, 'config', 'bridge_config.yaml'])
    rviz_config = PathJoinSubstitution([moveit_pkg, 'config', 'ur5_pybullet.rviz'])

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=move_group_params,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=rviz_params,
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5'),
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
                robot_description,
                {
                    'sim_mode': sim_mode,
                    'enable_dual_source': LaunchConfiguration('enable_dual_source'),
                    'urdf_path': '',
                    'home_positions': UR5_HOME,
                },
            ],
        ),
        Node(
            package='pybullet_bridge',
            executable='trajectory_controller_node',
            name='arm_trajectory_controller',
            output='screen',
            parameters=[{
                'joint_names': UR5_JOINTS,
                'action_name': '/scaled_joint_trajectory_controller/follow_joint_trajectory',
                'goal_tolerance': 0.02,
                'execution_duration_scaling': 2.0,
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        move_group_node,
        TimerAction(period=3.0, actions=[rviz_node]),
    ])
