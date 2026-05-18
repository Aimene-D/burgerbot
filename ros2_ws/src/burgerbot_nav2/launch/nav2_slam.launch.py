from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_nav2  = get_package_share_directory("burgerbot_nav2")
    pkg_slam  = get_package_share_directory("slam_toolbox")
    nav2_params = os.path.join(pkg_nav2, "config", "nav2_params.yaml")

    return LaunchDescription([

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "true",
                "slam_params_file": nav2_params
            }.items()
        ),

        # Controller server
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen",
             parameters=[nav2_params, {"use_sim_time": True}]),

        # Planner server
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen",
             parameters=[nav2_params, {"use_sim_time": True}]),

        # Behavior server
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen",
             parameters=[nav2_params, {"use_sim_time": True}]),

        # BT Navigator
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen",
             parameters=[nav2_params, {"use_sim_time": True}]),

        # Waypoint follower
        Node(package="nav2_waypoint_follower", executable="waypoint_follower",
             name="waypoint_follower", output="screen",
             parameters=[nav2_params, {"use_sim_time": True}]),

        # Velocity smoother
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen",
             parameters=[nav2_params, {"use_sim_time": True}]),

        # Lifecycle manager
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": True,
                          "autostart": True,
                          "node_names": [
                              "controller_server",
                              "planner_server",
                              "behavior_server",
                              "bt_navigator",
                              "waypoint_follower",
                              "velocity_smoother",
                          ]}]),
    ])
