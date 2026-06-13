from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import (
    LaunchConfiguration, Command, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('burgerbot')

    use_sim_time = LaunchConfiguration('use_sim_time')

    world_file = PathJoinSubstitution([pkg, 'worlds', 'burgerbot_world.sdf'])
    urdf_sim   = PathJoinSubstitution([pkg, 'urdf',   'burgerbot_sim.urdf.xacro'])
    bridge_cfg = PathJoinSubstitution([pkg, 'config', 'ros_gz_bridge.yaml'])
    ekf_cfg    = PathJoinSubstitution([pkg, 'config', 'ekf.yaml'])

    robot_description = {'robot_description': Command(['xacro ', urdf_sim])}

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock (keep true for simulation)'),

        # ── 1. Gazebo Harmonic ────────────────────────────────────────────
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
        ),

        # ── 2. Robot State Publisher: URDF -> /robot_description + static TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # ── 3. Spawn robot from /robot_description (delayed 3s for Gazebo) ─
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_burgerbot',
                    arguments=[
                        '-name',  'burgerbot',
                        '-topic', 'robot_description',
                        '-z',     '0.08',
                    ],
                    output='screen',
                ),
            ],
        ),

        # ── 4. Bridge Gazebo topics <-> ROS 2 ─────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{'config_file': bridge_cfg}],
            output='screen',
        ),

        # ── 5. IMU filter: /imu/raw (Gazebo) -> /imu/data ─────────────────
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag':      False,
                'publish_tf':   False,
                'world_frame':  'enu',
                'use_sim_time': use_sim_time,
            }],
            remappings=[('/imu/data_raw', '/imu/raw')],
            output='screen',
        ),

        # ── 6. EKF: /wheel_odom + /imu/data -> /odometry/filtered + TF ───
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[ekf_cfg, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
