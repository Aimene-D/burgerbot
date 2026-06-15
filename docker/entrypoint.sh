#!/bin/bash
set -e

# ROS2 Jazzy base
source /opt/ros/jazzy/setup.bash

# micro-ROS Agent workspace (built from source in image)
if [ -f /opt/uros_ws/agent_ws/install/setup.bash ]; then
  source /opt/uros_ws/agent_ws/install/setup.bash
fi

# Workspace overlay
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

# Gazebo resource path
#   - .../share          : lets Gazebo resolve package://burgerbot/... mesh URIs
#                          (package://burgerbot/meshes/rep/*.dae in the URDF)
#   - .../share/.../worlds: models referenced from the world file
export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/burgerbot/share:/ros2_ws/install/burgerbot/share/burgerbot/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}

exec "$@"
