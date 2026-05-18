from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lds02rr_driver',
            executable='lds02rr_node.py',
            name='lds02rr_driver',
            output='screen',
            parameters=[{
                'port':         '/dev/ttyUSB0',
                'baud':         115200,
                'frame_id':     'lidar',
                'range_min':    0.12,
                'range_max':    3.5,
                'angle_offset': 0.0,
            }]
        )
    ])
