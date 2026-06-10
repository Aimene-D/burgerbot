"""
Launch file for rep robot visualization.
Usage:
    ros2 launch rep_description robot_viz.launch.py
    ros2 launch rep_description robot_viz.launch.py use_ekf:=false
    ros2 launch rep_description robot_viz.launch.py use_imu_filter:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('rep_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'rep.urdf.xacro')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'display.rviz')
    controllers_file = os.path.join(pkg_dir, 'config', 'ros2_controllers.yaml')
    ekf_params_file = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    ns = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')
    use_ekf = LaunchConfiguration('use_ekf')
    use_imu_filter = LaunchConfiguration('use_imu_filter')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' prefix:=', prefix]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='',
            description='ROS 2 namespace'),
        DeclareLaunchArgument('prefix', default_value='',
            description='URDF prefix'),
        DeclareLaunchArgument('use_ekf', default_value='true',
            description='Lancer le noeud EKF (robot_localization)'),
        DeclareLaunchArgument('use_imu_filter', default_value='true',
            description='Lancer imu_filter_madgwick'),

        GroupAction([
            PushRosNamespace(ns),

            # Robot state publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
                output='screen',
            ),

            # ros2_control controller manager
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                name='controller_manager',
                parameters=[{'robot_description': robot_description}, controllers_file],
                output='screen',
            ),

            # Joint state broadcaster
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawn_joint_state_broadcaster',
                arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
                output='screen',
            ),

            # Position controller
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawn_position_controller',
                arguments=['position_controller', '--controller-manager', 'controller_manager'],
                output='screen',
            ),

            # Velocity controller
            Node(
                package='controller_manager',
                executable='spawner',
                name='spawn_velocity_controller',
                arguments=['velocity_controller', '--controller-manager', 'controller_manager'],
                output='screen',
            ),

            # IMU filter Madgwick
           Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter_madgwick',
                output='screen',
                parameters=[{
                       'use_mag': False,
                       'publish_tf': False,
                       'reverse_tf': False,
                       'fixed_frame': 'base_link',
                       'gain': 0.1,
                       'zeta': 0.0,
                       'world_frame': 'enu',
    }],
          remappings=[
        ('/imu/data_raw', '/imu/raw'),
        ('/imu/mag', '/imu/mag'),
        ('/imu/data', '/imu/data'),
    ],
    condition=IfCondition(use_imu_filter),
     ),

            # EKF node
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_params_file],
                condition=IfCondition(use_ekf),
            ),

            # RViz2
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_file],
                output='screen',
            ),
        ]),
    ])
