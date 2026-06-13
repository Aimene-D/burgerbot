FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ── ROS2 Jazzy packages + Gazebo Harmonic ────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Gazebo Harmonic
    gz-harmonic \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    # Navigation
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-waypoint-follower \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-map-server \
    # SLAM + localization
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-filter-madgwick \
    # Micro-ROS + tools
    ros-jazzy-micro-ros-agent \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rviz2 \
    # Build tooling
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ── Workspace ─────────────────────────────────────────────────────────────
WORKDIR /ros2_ws
COPY ros2_ws/src ./src/

RUN bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    colcon build \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release"

# ── Entrypoint ─────────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
