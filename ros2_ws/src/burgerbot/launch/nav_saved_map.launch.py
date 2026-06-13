from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg        = FindPackageShare('burgerbot')
    launch_dir = PathJoinSubstitution([pkg, 'launch'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file     = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'map',
            description=(
                'Full path to map WITHOUT extension, e.g. /home/user/maps/living_room'
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/bringup.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/localization.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map':          map_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/navigation.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
