"""M2 demo: MoveIt2 planning closed loop with PyBullet execution."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    bridge_pkg = FindPackageShare('pybullet_bridge')
    sim_mode = LaunchConfiguration('sim_mode')
    use_rviz = LaunchConfiguration('use_rviz')

    urdf_path = PathJoinSubstitution([bridge_pkg, 'urdf', 'planar_2dof.urdf'])

    moveit_config = (
        MoveItConfigsBuilder('planar_2dof', package_name='moveit_config')
        .robot_description(file_path='urdf/planar_2dof.urdf')
        .robot_description_semantic(file_path='srdf/planar_2dof.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    moveit_pkg = FindPackageShare('moveit_config')
    rviz_config = PathJoinSubstitution([moveit_pkg, 'config', 'moveit.rviz'])

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
                PathJoinSubstitution([bridge_pkg, 'config', 'bridge_config.yaml']),
                {
                    'sim_mode': sim_mode,
                    'enable_dual_source': LaunchConfiguration('enable_dual_source'),
                    'urdf_path': urdf_path,
                },
            ],
        ),
        Node(
            package='pybullet_bridge',
            executable='trajectory_controller_node',
            name='arm_trajectory_controller',
            output='screen',
            parameters=[{
                'joint_names': ['joint1', 'joint2'],
                'action_name': '/arm_controller/follow_joint_trajectory',
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[moveit_config.robot_description],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base',
            arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[moveit_config.to_dict()],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                moveit_config.planning_pipelines,
            ],
            condition=IfCondition(use_rviz),
        ),
    ])
