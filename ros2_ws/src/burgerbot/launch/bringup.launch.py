from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration, Command, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('burgerbot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port  = LaunchConfiguration('serial_port')
    lidar_port   = LaunchConfiguration('lidar_port')

    urdf_xacro = PathJoinSubstitution([pkg, 'urdf', 'burgerbot.urdf.xacro'])
    robot_description = {'robot_description': Command(['xacro ', urdf_xacro])}

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='Serial port for micro-ROS agent (ESP32)'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyACM1',
            description='Serial port for LDS02RR LiDAR'),

        # ── micro-ROS agent: bridges ESP32 <-> ROS 2 ──────────────────────
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', serial_port, '-b', '115200'],
            output='screen',
        ),

        # ── Robot state publisher: URDF -> /tf static transforms ──────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # ── LiDAR driver: serial -> /scan ─────────────────────────────────
        # frame_id must match URDF link name 'lidar'
        Node(
            package='lds02rr_driver',
            executable='lds02rr_node',
            name='lidar_driver',
            parameters=[{
                'port':         lidar_port,
                'baud':         115200,
                'frame_id':     'lidar',
                'range_min':    0.12,
                'range_max':    3.5,
                'angle_offset': 0.0,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        # ── IMU filter: /imu/raw -> /imu/data (Madgwick) ──────────────────
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag':     False,
                'publish_tf':  False,
                'world_frame': 'enu',
                'use_sim_time': use_sim_time,
            }],
            remappings=[('/imu/data_raw', '/imu/raw')],
            output='screen',
        ),

        # ── EKF: /wheel_odom + /imu/data -> /odometry/filtered + odom TF ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[
                PathJoinSubstitution([pkg, 'config', 'ekf.yaml']),
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
    ])
