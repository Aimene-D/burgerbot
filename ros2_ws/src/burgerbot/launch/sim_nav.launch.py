from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg        = FindPackageShare('burgerbot')
    launch_dir = PathJoinSubstitution([pkg, 'launch'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless     = LaunchConfiguration('headless')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'),
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run Gazebo server only (no GUI). Set false to open Gazebo window.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/gazebo.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'headless':     headless,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/slam.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/navigation.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
