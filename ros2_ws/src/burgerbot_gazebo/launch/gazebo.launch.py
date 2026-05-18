from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo = get_package_share_directory("burgerbot_gazebo")
    pkg_bringup = get_package_share_directory("burgerbot_bringup")

    world_file  = os.path.join(pkg_gazebo, "worlds", "burgerbot.sdf")
    robot_file  = os.path.join(pkg_gazebo, "worlds", "burgerbot_robot.sdf")
    urdf_file   = os.path.join(pkg_bringup, "urdf", "burger_robot.urdf")

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    return LaunchDescription([

        ExecuteProcess(
            cmd=["gz", "sim", world_file],
            output="screen"
        ),

        ExecuteProcess(
            cmd=["gz", "service", "-s", "/world/burgerbot_world/create",
                 "--reqtype", "gz.msgs.EntityFactory",
                 "--reptype", "gz.msgs.Boolean",
                 "--timeout", "5000",
                 "--req", f'sdf_filename: "{robot_file}", name: "burgerbot"'],
            output="screen"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": True}]
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            output="screen",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
                "/model/burgerbot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            ],
            remappings=[
                ("/model/burgerbot/tf", "/tf"),
            ]
        ),
    ])
