FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Add Gazebo Harmonic apt repository
# ros:jazzy-ros-base only ships the ROS2 repo; gz-harmonic lives on osrfoundation
RUN apt-get update \
 && apt-get install -y --no-install-recommends curl ca-certificates \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && rm -rf /var/lib/apt/lists/*

# ROS2 Jazzy + Gazebo Harmonic packages
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      gz-harmonic \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-sim \
      ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-controller \
      ros-jazzy-nav2-planner \
      ros-jazzy-nav2-behaviors \
      ros-jazzy-nav2-bt-navigator \
      ros-jazzy-nav2-velocity-smoother \
      ros-jazzy-nav2-waypoint-follower \
      ros-jazzy-nav2-lifecycle-manager \
      ros-jazzy-nav2-map-server \
      ros-jazzy-slam-toolbox \
      ros-jazzy-robot-localization \
      ros-jazzy-imu-filter-madgwick \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-rviz2 \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-serial \
      python3-vcstool \
      git \
 && rm -rf /var/lib/apt/lists/*

# micro-ROS Agent runs as a separate container (microros/micro-ros-agent:jazzy)
# via docker-compose — no local build needed.

# Workspace
WORKDIR /ros2_ws
COPY ros2_ws/src ./src/

RUN . /opt/ros/jazzy/setup.sh \
 && colcon build \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
