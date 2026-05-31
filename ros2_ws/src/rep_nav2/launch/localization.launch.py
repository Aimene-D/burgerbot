from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('rep_nav2')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox_localization.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config],
        )
    ])
