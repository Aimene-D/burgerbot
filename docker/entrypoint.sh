#!/bin/bash
set -e

# ROS2 Jazzy base
source /opt/ros/jazzy/setup.bash

# Workspace overlay
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

# Gazebo resource path
export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/burgerbot/share/burgerbot/worlds

exec "$@"
