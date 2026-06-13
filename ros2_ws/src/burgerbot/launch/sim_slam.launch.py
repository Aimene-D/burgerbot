from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg        = FindPackageShare('burgerbot')
    launch_dir = PathJoinSubstitution([pkg, 'launch'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, f'/{name}']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'),

        include('gazebo.launch.py'),
        include('slam.launch.py'),
    ])
