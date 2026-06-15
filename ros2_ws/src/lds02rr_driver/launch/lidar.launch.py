from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/ttyACM1')

    return LaunchDescription([
        Node(
            package='lds02rr_driver',
            executable='lds02rr_node',
            name='lds02rr_driver',
            output='screen',
            parameters=[{
                'port':         port,
                'baud':         115200,
                'frame_id':     'lidar',
                'range_min':    0.12,
                'range_max':    3.5,
                'angle_offset': 0.0,
            }]
        )
    ])
