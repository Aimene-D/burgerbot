from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('burgerbot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file     = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'map',
            description=(
                'Full path to map WITHOUT extension. '
                'SLAM Toolbox needs .posegraph + .data files saved with: '
                'ros2 run slam_toolbox map_saver_cli -f /path/to/map'
            ),
        ),

        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                PathJoinSubstitution([pkg, 'config', 'slam_toolbox_localization.yaml']),
                {
                    'use_sim_time':      use_sim_time,
                    'map_file_name':     map_file,
                    'map_start_at_dock': True,
                },
            ],
            output='screen',
        ),
    ])
