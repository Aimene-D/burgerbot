#!/bin/bash
# Install ROS2 Jazzy + Gazebo Harmonic + all burgerbot dependencies
# Tested on Ubuntu 24.04 (Noble)
set -e

echo "==> Adding ROS2 Jazzy apt repository..."
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "==> Adding Gazebo Harmonic apt repository..."
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update

echo "==> Installing ROS2 Jazzy packages..."
sudo apt install -y \
    ros-jazzy-ros-base \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2 \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-filter-madgwick \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-waypoint-follower \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-map-server \
    ros-jazzy-micro-ros-agent \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    gz-harmonic \
    python3-colcon-common-extensions \
    python3-pip \
    python3-serial

echo ""
echo "==> Done! Next step: bash scripts/build.sh"
