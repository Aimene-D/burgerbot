# BurgerBot Clean ROS2 Stack Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create `ros2_ws/src/burgerbot/` — a single clean package replacing the 5 fragmented existing packages, with well-separated launch files for easy hardware debugging, SLAM mapping, localization, and autonomous navigation.

**Architecture:** Single CMake package with distinct launch files per concern (bringup, SLAM, localization, navigation) plus compound launchers for common workflows. All parameters fixed, no hardcoded paths, `use_sim_time` is always a launch argument.

**Tech Stack:** ROS 2 Jazzy, SLAM Toolbox, Nav2, robot_localization EKF, imu_filter_madgwick, micro-ROS agent, lds02rr_driver

---

## Issues Fixed

| # | Issue | Fix |
|---|-------|-----|
| 1 | Two robot descriptions, conflicting wheel geometry | One URDF; firmware values: radius=0.0625m, sep=0.282m |
| 2 | Duplicate EKF configs | One `config/ekf.yaml` |
| 3 | Two nav packages, wrong robot radii (0.1m, 0.22m) | One nav2.yaml, robot_radius=0.105m |
| 4 | LiDAR frame `base_scan` ≠ driver `lidar` | URDF link named `lidar` |
| 5 | Hardcoded `/home/aimen/…` in localization yaml | `map_file_name` passed at launch time |
| 6 | `use_sim_time: true` hardcoded in nav package | Launch arg everywhere |
| 7 | No micro-ROS agent launch | `bringup.launch.py` starts it |
| 8 | No complete single-command bringup | `bringup.launch.py` + compound launchers |
| 9 | Mixed `rep_*` / `burgerbot_*` naming | Everything under `burgerbot` |
| 10 | lifecycle_manager timeout=0 | `bond_timeout: 30.0` |

## Launch File Map

```
bringup.launch.py           → micro-ROS agent + RSP + lidar + imu_filter + EKF
slam.launch.py              → SLAM Toolbox (mapping mode only)
localization.launch.py      → SLAM Toolbox (localization mode, map:= required)
navigation.launch.py        → Nav2 stack (controller/planner/behavior/bt/smoother)
debug.launch.py             → RViz2
slam_nav.launch.py          → bringup + slam + navigation  [compound]
nav_saved_map.launch.py     → bringup + localization + navigation  [compound]
```

## File Structure

```
ros2_ws/src/burgerbot/
├── package.xml
├── CMakeLists.txt
├── urdf/
│   └── burgerbot.urdf.xacro
├── config/
│   ├── ekf.yaml
│   ├── slam_toolbox.yaml
│   ├── slam_toolbox_localization.yaml
│   ├── nav2.yaml
│   └── rviz/
│       └── burgerbot.rviz
├── launch/
│   ├── bringup.launch.py
│   ├── slam.launch.py
│   ├── localization.launch.py
│   ├── navigation.launch.py
│   ├── debug.launch.py
│   ├── slam_nav.launch.py
│   └── nav_saved_map.launch.py
└── maps/
    └── .gitkeep
```

---

## Task 1: Package Scaffold

**Files:**
- Create: `ros2_ws/src/burgerbot/package.xml`
- Create: `ros2_ws/src/burgerbot/CMakeLists.txt`
- Create: `ros2_ws/src/burgerbot/maps/.gitkeep`

- [ ] **Step 1: Create directory tree**

```bash
mkdir -p ros2_ws/src/burgerbot/urdf
mkdir -p ros2_ws/src/burgerbot/config/rviz
mkdir -p ros2_ws/src/burgerbot/launch
mkdir -p ros2_ws/src/burgerbot/maps
touch ros2_ws/src/burgerbot/maps/.gitkeep
```

- [ ] **Step 2: Write package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>burgerbot</name>
  <version>1.0.0</version>
  <description>BurgerBot complete stack: bringup, SLAM, navigation</description>
  <maintainer email="khadraouiibrahim@gmail.com">Ibrahim</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>robot_localization</exec_depend>
  <exec_depend>imu_filter_madgwick</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_controller</exec_depend>
  <exec_depend>nav2_planner</exec_depend>
  <exec_depend>nav2_behaviors</exec_depend>
  <exec_depend>nav2_bt_navigator</exec_depend>
  <exec_depend>nav2_velocity_smoother</exec_depend>
  <exec_depend>nav2_waypoint_follower</exec_depend>
  <exec_depend>nav2_lifecycle_manager</exec_depend>
  <exec_depend>micro_ros_agent</exec_depend>
  <exec_depend>lds02rr_driver</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(burgerbot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 4: Verify build (package only)**

```bash
cd ros2_ws
colcon build --packages-select burgerbot --symlink-install
```
Expected: `Summary: 1 packages finished`

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/burgerbot/package.xml ros2_ws/src/burgerbot/CMakeLists.txt ros2_ws/src/burgerbot/maps/.gitkeep
git commit -m "feat(burgerbot): add package scaffold"
```

---

## Task 2: URDF

**Files:**
- Create: `ros2_ws/src/burgerbot/urdf/burgerbot.urdf.xacro`

Key geometry (authoritative from `firmware/src/config.h`):
- `WHEEL_RADIUS_M = 0.0625` — used for odometry
- `WHEELBASE_M = 0.282` — wheel separation for odometry
- LiDAR frame name must be `lidar` — matches `lds02rr_driver` `frame_id` parameter

- [ ] **Step 1: Write burgerbot.urdf.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="burgerbot">

  <!-- ── Geometry (firmware/src/config.h is authoritative) ── -->
  <xacro:property name="wheel_radius"     value="0.0625"/>
  <xacro:property name="wheel_sep"        value="0.282"/>
  <xacro:property name="base_radius"      value="0.0825"/>
  <xacro:property name="base_height"      value="0.090"/>
  <xacro:property name="wheel_width"      value="0.026"/>
  <xacro:property name="caster_radius"    value="0.0175"/>
  <xacro:property name="lidar_x"          value="-0.032"/>
  <xacro:property name="lidar_z"          value="0.171"/>
  <xacro:property name="imu_z"            value="0.050"/>
  <xacro:property name="pi"               value="3.14159265358979"/>

  <!-- ── Materials ── -->
  <material name="chassis_grey"><color rgba="0.65 0.65 0.65 1.0"/></material>
  <material name="wheel_black"><color rgba="0.15 0.15 0.15 1.0"/></material>
  <material name="lidar_dark"><color rgba="0.2 0.2 0.2 1.0"/></material>
  <material name="white"><color rgba="1.0 1.0 1.0 1.0"/></material>

  <!-- ── base_footprint: virtual ground-plane reference ── -->
  <link name="base_footprint"/>

  <!-- base_link sits at wheel-axle height above the floor -->
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child  link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- ── base_link: circular chassis ── -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="${base_radius}" length="${base_height}"/></geometry>
      <material name="chassis_grey"/>
    </visual>
    <collision>
      <geometry><cylinder radius="${base_radius}" length="${base_height}"/></geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <!-- solid cylinder: Ixx=Iyy = m(3r²+h²)/12, Izz = mr²/2 -->
      <inertia ixx="0.00564" ixy="0" ixz="0"
               iyy="0.00564" iyz="0"
               izz="0.00284"/>
    </inertial>
  </link>

  <!-- ── Left wheel ── -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child  link="left_wheel_link"/>
    <!-- rotate -90° around X so wheel cylinder axis → Y direction -->
    <origin xyz="0 ${wheel_sep/2} 0" rpy="${-pi/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="left_wheel_link">
    <visual>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      <material name="wheel_black"/>
    </visual>
    <collision>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <!-- thin disk: Ixx=Iyy = m(3r²+h²)/12, Izz = mr²/2 -->
      <inertia ixx="9.90e-5" ixy="0" ixz="0"
               iyy="9.90e-5" iyz="0"
               izz="1.95e-4"/>
    </inertial>
  </link>

  <!-- ── Right wheel ── -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child  link="right_wheel_link"/>
    <origin xyz="0 ${-wheel_sep/2} 0" rpy="${pi/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="right_wheel_link">
    <visual>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
      <material name="wheel_black"/>
    </visual>
    <collision>
      <geometry><cylinder radius="${wheel_radius}" length="${wheel_width}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="9.90e-5" ixy="0" ixz="0"
               iyy="9.90e-5" iyz="0"
               izz="1.95e-4"/>
    </inertial>
  </link>

  <!-- ── Front caster ── -->
  <!-- caster centre must be at floor level: z = -(wheel_radius - caster_radius) in base_link frame -->
  <joint name="front_caster_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="front_caster_link"/>
    <origin xyz="0.057 0 ${-(wheel_radius - caster_radius)}" rpy="0 0 0"/>
  </joint>

  <link name="front_caster_link">
    <visual>
      <geometry><sphere radius="${caster_radius}"/></geometry>
      <material name="wheel_black"/>
    </visual>
    <collision>
      <geometry><sphere radius="${caster_radius}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="4e-7" ixy="0" ixz="0"
               iyy="4e-7" iyz="0"
               izz="4e-7"/>
    </inertial>
  </link>

  <!-- ── Rear caster ── -->
  <joint name="rear_caster_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="rear_caster_link"/>
    <origin xyz="-0.054 0 ${-(wheel_radius - caster_radius)}" rpy="0 0 0"/>
  </joint>

  <link name="rear_caster_link">
    <visual>
      <geometry><sphere radius="${caster_radius}"/></geometry>
      <material name="wheel_black"/>
    </visual>
    <collision>
      <geometry><sphere radius="${caster_radius}"/></geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="4e-7" ixy="0" ixz="0"
               iyy="4e-7" iyz="0"
               izz="4e-7"/>
    </inertial>
  </link>

  <!-- ── LiDAR (frame name MUST match lds02rr_driver frame_id: 'lidar') ── -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="lidar"/>
    <origin xyz="${lidar_x} 0 ${lidar_z}" rpy="0 0 0"/>
  </joint>

  <link name="lidar">
    <visual>
      <geometry><cylinder radius="0.0508" length="0.055"/></geometry>
      <material name="lidar_dark"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.0508" length="0.055"/></geometry>
    </collision>
    <inertial>
      <mass value="0.114"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="9.24e-5" ixy="0" ixz="0"
               iyy="9.24e-5" iyz="0"
               izz="1.47e-4"/>
    </inertial>
  </link>

  <!-- ── IMU ── -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="imu_link"/>
    <origin xyz="0 0 ${imu_z}" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <visual>
      <geometry><box size="0.01 0.01 0.005"/></geometry>
      <material name="white"/>
    </visual>
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-8" ixy="0" ixz="0"
               iyy="1e-8" iyz="0"
               izz="1e-8"/>
    </inertial>
  </link>

</robot>
```

- [ ] **Step 2: Validate URDF with xacro**

```bash
cd ros2_ws
source install/setup.bash
xacro src/burgerbot/urdf/burgerbot.urdf.xacro > /tmp/burgerbot_check.urdf
check_urdf /tmp/burgerbot_check.urdf
```
Expected: `Successfully Parsed` with link tree showing `base_footprint → base_link → wheels/casters/lidar/imu_link`

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/burgerbot/urdf/burgerbot.urdf.xacro
git commit -m "feat(burgerbot): add clean URDF with correct wheel geometry"
```

---

## Task 3: Config Files

**Files:**
- Create: `ros2_ws/src/burgerbot/config/ekf.yaml`
- Create: `ros2_ws/src/burgerbot/config/slam_toolbox.yaml`
- Create: `ros2_ws/src/burgerbot/config/slam_toolbox_localization.yaml`
- Create: `ros2_ws/src/burgerbot/config/nav2.yaml`

### 3a — EKF Config

- [ ] **Step 1: Write config/ekf.yaml**

Fuses `/wheel_odom` (VX + VYaw) and `/imu/data` (VYaw) into the `odom→base_link` TF.
The 15-element config vector maps to: `[x,y,z, roll,pitch,yaw, vx,vy,vz, vroll,vpitch,vyaw, ax,ay,az]`

```yaml
ekf_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    publish_tf: true

    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # /wheel_odom: Odometry from firmware — use VX (idx 6) and VYaw (idx 11)
    odom0: /wheel_odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, true,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false

    # /imu/data: Madgwick-filtered IMU — use VYaw (idx 11) only
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true

    process_noise_covariance: [1.0e-3, 0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                                0,      1.0e-3, 0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                                0,      0,      1.0e-3, 0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                                0,      0,      0,      0.3,    0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                                0,      0,      0,      0,      0.3,    0,      0,      0,      0,      0,      0,      0,      0,      0,      0,
                                0,      0,      0,      0,      0,      0.01,   0,      0,      0,      0,      0,      0,      0,      0,      0,
                                0,      0,      0,      0,      0,      0,      0.5,    0,      0,      0,      0,      0,      0,      0,      0,
                                0,      0,      0,      0,      0,      0,      0,      0.5,    0,      0,      0,      0,      0,      0,      0,
                                0,      0,      0,      0,      0,      0,      0,      0,      0.1,    0,      0,      0,      0,      0,      0,
                                0,      0,      0,      0,      0,      0,      0,      0,      0,      0.3,    0,      0,      0,      0,      0,
                                0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0.3,    0,      0,      0,      0,
                                0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0.01,   0,      0,      0,
                                0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0.5,    0,      0,
                                0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0.5,    0,
                                0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0.5]
```

### 3b — SLAM Toolbox Mapping Config

- [ ] **Step 2: Write config/slam_toolbox.yaml**

```yaml
slam_toolbox_node:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    resolution: 0.05
    min_laser_range: 0.12
    max_laser_range: 3.5
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0

    do_loop_closing: true
    loop_search_maximum_distance: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
    loop_match_distance_threshold: 0.5
    loop_match_maximum_variance_coarse: 3.0
    loop_search_space_dimension: 8.0

    distance_decomposition_radius: 1.0
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
```

### 3c — SLAM Toolbox Localization Config

- [ ] **Step 3: Write config/slam_toolbox_localization.yaml**

Note: `map_file_name` is intentionally absent — it is injected at launch time via `--ros-args -p map_file_name:=<path>` so there are no hardcoded paths.

```yaml
slam_toolbox_node:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: localization

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    resolution: 0.05
    min_laser_range: 0.12
    max_laser_range: 3.5
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0

    map_start_at_dock: true
    do_loop_closing: false
    # map_file_name: passed via launch argument, e.g.:
    #   ros2 launch burgerbot localization.launch.py map:=/home/user/maps/my_map
```

### 3d — Nav2 Config

- [ ] **Step 4: Write config/nav2.yaml**

Key fixes from existing packages:
- `robot_radius: 0.105` (actual 0.0825m + 0.02m safety — previous: 0.22m/0.1m both wrong)
- `bond_timeout: 30.0` (previous: 0.0 which never times out)
- `odom_topic: /odometry/filtered` (EKF output)
- AMCL removed — SLAM Toolbox handles map→odom TF in all modes

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.05
      yaw_goal_tolerance: 0.1

    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.2
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 0.6
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_fixed_curvature_lookahead: false
      curvature_lookahead_dist: 0.25
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      allow_reversing: false
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
      max_robot_pose_search_dist: 10.0

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    wait:
      plugin: "nav2_behaviors/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_loop_duration: 10
    default_server_timeout: 20
    wait_for_service_timeout: 1000
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator/NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator/NavigateThroughPosesNavigator"

velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: "OPEN_LOOP"
    max_velocity: [0.3, 0.0, 1.0]
    min_velocity: [-0.3, 0.0, -1.0]
    max_accel: [2.5, 0.0, 3.2]
    max_decel: [-2.5, 0.0, -3.2]
    odom_topic: /odometry/filtered
    odom_duration: 0.1
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 1.0

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    action_server_result_timeout: 900.0
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.105
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.5
          raytrace_min_range: 0.0
          obstacle_max_range: 3.0
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.20
      always_send_full_costmap: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.105
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.5
          raytrace_min_range: 0.0
          obstacle_max_range: 3.0
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.20
      always_send_full_costmap: true
```

- [ ] **Step 5: Write config/rviz/burgerbot.rviz**

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
    - Class: rviz_default_plugins/RobotModel
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Topic: /robot_description
        Value: /robot_description
      Enabled: true
      Name: RobotModel
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Class: rviz_default_plugins/LaserScan
      Color: 255; 0; 0
      Enabled: true
      Name: LaserScan
      Size (m): 0.05
      Topic:
        Topic: /scan
        Value: /scan
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Enabled: true
      Name: Map
      Topic:
        Topic: /map
        Value: /map
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 0; 255; 0
      Enabled: true
      Name: GlobalPlan
      Topic:
        Topic: /plan
        Value: /plan
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 0; 0; 255
      Enabled: true
      Name: LocalPlan
      Topic:
        Topic: /local_plan
        Value: /local_plan
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: nav2_rviz_plugins/GoalTool
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
```

- [ ] **Step 6: Rebuild and verify configs are installed**

```bash
cd ros2_ws
colcon build --packages-select burgerbot --symlink-install
ls install/burgerbot/share/burgerbot/config/
```
Expected: `ekf.yaml  nav2.yaml  rviz/  slam_toolbox.yaml  slam_toolbox_localization.yaml`

- [ ] **Step 7: Commit**

```bash
git add ros2_ws/src/burgerbot/config/
git commit -m "feat(burgerbot): add EKF, SLAM, Nav2 configs with fixed params"
```

---

## Task 4: bringup.launch.py

**Files:**
- Create: `ros2_ws/src/burgerbot/launch/bringup.launch.py`

Starts the hardware layer: micro-ROS bridge, robot description, LiDAR, IMU filter, EKF.
Run this first before any SLAM or navigation launch.

- [ ] **Step 1: Write launch/bringup.launch.py**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration, Command, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('burgerbot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port  = LaunchConfiguration('serial_port')
    lidar_port   = LaunchConfiguration('lidar_port')

    urdf_xacro = PathJoinSubstitution([pkg, 'urdf', 'burgerbot.urdf.xacro'])
    robot_description = {'robot_description': Command(['xacro ', urdf_xacro])}

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='Serial port for micro-ROS agent (ESP32)'),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyACM1',
            description='Serial port for LDS02RR LiDAR'),

        # ── micro-ROS agent: bridges ESP32 ↔ ROS 2 ──────────────────────
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', serial_port, '-b', '115200'],
            output='screen',
        ),

        # ── Robot state publisher: URDF → /tf static transforms ──────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # ── LiDAR driver: serial → /scan ──────────────────────────────────
        # frame_id must match URDF link name 'lidar'
        Node(
            package='lds02rr_driver',
            executable='lds02rr_node',
            name='lidar_driver',
            parameters=[{
                'port':         lidar_port,
                'baud':         115200,
                'frame_id':     'lidar',
                'range_min':    0.12,
                'range_max':    3.5,
                'angle_offset': 0.0,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),

        # ── IMU filter: /imu/raw → /imu/data (Madgwick) ──────────────────
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag':    False,
                'publish_tf': False,
                'world_frame': 'enu',
                'use_sim_time': use_sim_time,
            }],
            remappings=[('/imu/data_raw', '/imu/raw')],
            output='screen',
        ),

        # ── EKF: /wheel_odom + /imu/data → /odometry/filtered + odom TF ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[
                PathJoinSubstitution([pkg, 'config', 'ekf.yaml']),
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
    ])
```

- [ ] **Step 2: Build and source**

```bash
cd ros2_ws
colcon build --packages-select burgerbot --symlink-install
source install/setup.bash
```

- [ ] **Step 3: Dry-run verify (no hardware needed)**

```bash
ros2 launch burgerbot bringup.launch.py --show-args
```
Expected: prints `use_sim_time`, `serial_port`, `lidar_port` with defaults. No crash.

- [ ] **Step 4: Commit**

```bash
git add ros2_ws/src/burgerbot/launch/bringup.launch.py
git commit -m "feat(burgerbot): add bringup launch (micro-ROS, RSP, lidar, IMU, EKF)"
```

---

## Task 5: slam.launch.py

**Files:**
- Create: `ros2_ws/src/burgerbot/launch/slam.launch.py`

SLAM Toolbox in mapping mode only. Requires bringup running.
Provides `map → odom` TF. Use `ros2 run nav2_map_saver map_saver_cli -f ~/maps/my_map` to save.

- [ ] **Step 1: Write launch/slam.launch.py**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('burgerbot')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                PathJoinSubstitution([pkg, 'config', 'slam_toolbox.yaml']),
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
    ])
```

- [ ] **Step 2: Verify launch args**

```bash
ros2 launch burgerbot slam.launch.py --show-args
```
Expected: `use_sim_time` listed.

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/burgerbot/launch/slam.launch.py
git commit -m "feat(burgerbot): add standalone SLAM mapping launch"
```

---

## Task 6: localization.launch.py

**Files:**
- Create: `ros2_ws/src/burgerbot/launch/localization.launch.py`

SLAM Toolbox in localization mode. Requires bringup running and a saved `.posegraph` + `.data` map pair.
The `map` argument takes the path without extension, e.g. `map:=/home/user/maps/living_room`.

- [ ] **Step 1: Write launch/localization.launch.py**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('burgerbot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file     = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'map',
            description=(
                'Full path to map WITHOUT extension. '
                'SLAM Toolbox needs the .posegraph + .data files saved '
                'with: ros2 run slam_toolbox map_saver_cli -f /path/to/map'
            ),
        ),

        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                PathJoinSubstitution([pkg, 'config', 'slam_toolbox_localization.yaml']),
                {
                    'use_sim_time':     use_sim_time,
                    'map_file_name':    map_file,
                    'map_start_at_dock': True,
                },
            ],
            output='screen',
        ),
    ])
```

- [ ] **Step 2: Verify launch args**

```bash
ros2 launch burgerbot localization.launch.py --show-args
```
Expected: `use_sim_time`, `map` listed. `map` has no default (required).

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/burgerbot/launch/localization.launch.py
git commit -m "feat(burgerbot): add localization launch (SLAM Toolbox localization mode)"
```

---

## Task 7: navigation.launch.py

**Files:**
- Create: `ros2_ws/src/burgerbot/launch/navigation.launch.py`

Nav2 planning and control stack. Requires a running map→odom TF provider (either slam.launch.py or localization.launch.py). Does NOT include AMCL — SLAM Toolbox provides the map→odom TF in all modes.

- [ ] **Step 1: Write launch/navigation.launch.py**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('burgerbot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params  = PathJoinSubstitution([pkg, 'config', 'nav2.yaml'])

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
        'waypoint_follower',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cmd_vel',          'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart':    True,
                'node_names':   lifecycle_nodes,
                'bond_timeout': 30.0,
            }],
        ),
    ])
```

- [ ] **Step 2: Verify launch args**

```bash
ros2 launch burgerbot navigation.launch.py --show-args
```
Expected: `use_sim_time` listed.

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/burgerbot/launch/navigation.launch.py
git commit -m "feat(burgerbot): add Nav2 navigation launch"
```

---

## Task 8: debug.launch.py

**Files:**
- Create: `ros2_ws/src/burgerbot/launch/debug.launch.py`

Opens RViz2 with the pre-configured display. Run alongside any other launch for visual debugging.

- [ ] **Step 1: Write launch/debug.launch.py**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('burgerbot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config  = PathJoinSubstitution([pkg, 'config', 'rviz', 'burgerbot.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
```

- [ ] **Step 2: Commit**

```bash
git add ros2_ws/src/burgerbot/launch/debug.launch.py
git commit -m "feat(burgerbot): add debug RViz2 launch"
```

---

## Task 9: Compound Launchers

**Files:**
- Create: `ros2_ws/src/burgerbot/launch/slam_nav.launch.py`
- Create: `ros2_ws/src/burgerbot/launch/nav_saved_map.launch.py`

### 9a — slam_nav.launch.py

Full autonomous mapping + navigation in one command.
Robot moves through unknown space building the map while Nav2 plans paths.

- [ ] **Step 1: Write launch/slam_nav.launch.py**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg        = FindPackageShare('burgerbot')
    launch_dir = PathJoinSubstitution([pkg, 'launch'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, f'/{name}']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),

        include('bringup.launch.py'),
        include('slam.launch.py'),
        include('navigation.launch.py'),
    ])
```

### 9b — nav_saved_map.launch.py

Navigate a previously mapped area.
Requires a map saved with SLAM Toolbox (produces `.posegraph` + `.data` pair).

- [ ] **Step 2: Write launch/nav_saved_map.launch.py**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg        = FindPackageShare('burgerbot')
    launch_dir = PathJoinSubstitution([pkg, 'launch'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file     = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'map',
            description=(
                'Full path to map file WITHOUT extension, e.g. '
                '/home/user/maps/living_room'
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/bringup.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/localization.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map':          map_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/navigation.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
```

- [ ] **Step 3: Verify all launch args**

```bash
ros2 launch burgerbot slam_nav.launch.py --show-args
ros2 launch burgerbot nav_saved_map.launch.py --show-args
```
Expected: `slam_nav` shows `use_sim_time`. `nav_saved_map` shows `use_sim_time` + `map` (required).

- [ ] **Step 4: Final build**

```bash
cd ros2_ws
colcon build --packages-select burgerbot --symlink-install
source install/setup.bash
```
Expected: `Summary: 1 packages finished`

- [ ] **Step 5: Verify all launch files load without errors**

```bash
ros2 launch burgerbot bringup.launch.py --show-args
ros2 launch burgerbot slam.launch.py --show-args
ros2 launch burgerbot localization.launch.py --show-args
ros2 launch burgerbot navigation.launch.py --show-args
ros2 launch burgerbot debug.launch.py --show-args
ros2 launch burgerbot slam_nav.launch.py --show-args
ros2 launch burgerbot nav_saved_map.launch.py --show-args
```
Expected: each command prints argument list, no `ModuleNotFoundError` or `ImportError`.

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/burgerbot/launch/
git commit -m "feat(burgerbot): add compound slam_nav and nav_saved_map launchers"
```

---

## Usage Cheatsheet

### Map a new environment
```bash
# Terminal 1
ros2 launch burgerbot bringup.launch.py

# Terminal 2
ros2 launch burgerbot slam.launch.py

# Terminal 3 (optional visualization)
ros2 launch burgerbot debug.launch.py

# Terminal 4 (drive around with keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# When done — save map (saves my_map.posegraph + my_map.data)
ros2 run slam_toolbox map_saver_cli -f ~/maps/my_map
```

### Navigate saved map
```bash
ros2 launch burgerbot nav_saved_map.launch.py map:=/home/user/maps/my_map
ros2 launch burgerbot debug.launch.py   # optional RViz
```

### Autonomous SLAM + Nav
```bash
ros2 launch burgerbot slam_nav.launch.py
ros2 launch burgerbot debug.launch.py   # optional RViz
```

### Hardware debugging only
```bash
ros2 launch burgerbot bringup.launch.py
ros2 topic echo /scan           # verify LiDAR
ros2 topic echo /imu/data       # verify filtered IMU
ros2 topic echo /odometry/filtered  # verify EKF
ros2 topic echo /tf --once      # verify TF chain
```

---

## Self-Review Notes (Hardware Stack)

- **Wheel params:** `wheel_radius=0.0625`, `wheel_sep=0.282` match `firmware/src/config.h` exactly
- **LiDAR frame:** URDF link `lidar` = `lds02rr_driver` default `frame_id` = `lidar` ✓
- **EKF TF:** `ekf_node` publishes `odom→base_link`. Firmware should NOT publish this TF simultaneously — ensure firmware conditional TF publish is disabled if micro-ROS agent is running with EKF
- **SLAM Toolbox executable names:** `async_slam_toolbox_node` (mapping), `localization_slam_toolbox_node` (localization) — verify these exist in your Jazzy install with `ros2 pkg executables slam_toolbox`
- **Nav2 node names:** `behavior_server` package is `nav2_behaviors` in Jazzy — verify with `ros2 pkg executables nav2_behaviors`
- **velocity_smoother remapping:** remaps `cmd_vel` → `cmd_vel_nav` (input from bt_navigator) and `cmd_vel_smoothed` → `cmd_vel` (output to robot) — check if this is the correct direction for your Jazzy version
- **No hardcoded paths** anywhere in this package ✓
- **`use_sim_time` is a launch arg** everywhere ✓

---

## Part 2: Gazebo Simulation + Docker + Install Scripts

Three independent additions that extend the plan:
1. **Tasks 10–15:** Gazebo Harmonic simulation (full sim parity with real robot)
2. **Task 16:** Dockerfile + docker-compose (run entire stack without ROS2 installed locally)
3. **Task 17:** Fix `installfff/` broken paths + add proper install scripts

### Updated File Structure

```
ros2_ws/src/burgerbot/
├── package.xml              (Task 10 — add ros_gz deps)
├── CMakeLists.txt           (Task 10 — install worlds/)
├── urdf/
│   ├── burgerbot.urdf.xacro           (Task 2 — base, no Gazebo tags)
│   └── burgerbot_sim.urdf.xacro       (Task 12 — includes base + Gazebo plugins)
├── worlds/
│   └── burgerbot_world.sdf            (Task 11 — 6×6m room + sensors plugins)
├── config/
│   ├── ekf.yaml
│   ├── slam_toolbox.yaml
│   ├── slam_toolbox_localization.yaml
│   ├── nav2.yaml
│   ├── ros_gz_bridge.yaml             (Task 13 — topic bridge config)
│   └── rviz/burgerbot.rviz
├── launch/
│   ├── bringup.launch.py
│   ├── slam.launch.py
│   ├── localization.launch.py
│   ├── navigation.launch.py
│   ├── debug.launch.py
│   ├── slam_nav.launch.py
│   ├── nav_saved_map.launch.py
│   ├── gazebo.launch.py               (Task 14 — sim entry point)
│   ├── sim_slam.launch.py             (Task 15 — sim + slam)
│   └── sim_nav.launch.py             (Task 15 — sim + slam + nav)
└── maps/.gitkeep

docker/
├── Dockerfile                         (Task 16)
├── entrypoint.sh                      (Task 16)
└── docker-compose.yml                 (Task 16)

scripts/
├── install_deps.sh                    (Task 17)
└── build.sh                           (Task 17)
```

### Updated Launch File Map

```
# Real hardware
bringup.launch.py           → micro-ROS agent + RSP + lidar + imu_filter + EKF
slam.launch.py              → SLAM Toolbox mapping
localization.launch.py      → SLAM Toolbox localization (map:= required)
navigation.launch.py        → Nav2 stack
debug.launch.py             → RViz2
slam_nav.launch.py          → bringup + slam + navigation  [compound]
nav_saved_map.launch.py     → bringup + localization + navigation  [compound]

# Simulation (Gazebo Harmonic, use_sim_time defaults to true)
gazebo.launch.py            → Gazebo + RSP + spawn + bridge + IMU filter + EKF
sim_slam.launch.py          → gazebo + slam  [compound]
sim_nav.launch.py           → gazebo + slam + navigation  [compound]
```

---

## Task 10: Update package.xml + CMakeLists.txt for Gazebo

**Files:**
- Modify: `ros2_ws/src/burgerbot/package.xml`
- Modify: `ros2_ws/src/burgerbot/CMakeLists.txt`

- [ ] **Step 1: Write updated package.xml (full file)**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>burgerbot</name>
  <version>1.0.0</version>
  <description>BurgerBot complete stack: bringup, SLAM, navigation, simulation</description>
  <maintainer email="khadraouiibrahim@gmail.com">Ibrahim</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>robot_localization</exec_depend>
  <exec_depend>imu_filter_madgwick</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_controller</exec_depend>
  <exec_depend>nav2_planner</exec_depend>
  <exec_depend>nav2_behaviors</exec_depend>
  <exec_depend>nav2_bt_navigator</exec_depend>
  <exec_depend>nav2_velocity_smoother</exec_depend>
  <exec_depend>nav2_waypoint_follower</exec_depend>
  <exec_depend>nav2_lifecycle_manager</exec_depend>
  <exec_depend>micro_ros_agent</exec_depend>
  <exec_depend>lds02rr_driver</exec_depend>
  <!-- Gazebo Harmonic simulation -->
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Write updated CMakeLists.txt (full file)**

```cmake
cmake_minimum_required(VERSION 3.8)
project(burgerbot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  config
  launch
  maps
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 3: Create worlds/ directory**

```bash
mkdir -p ros2_ws/src/burgerbot/worlds
```

- [ ] **Step 4: Rebuild to verify**

```bash
cd ros2_ws
colcon build --packages-select burgerbot --symlink-install
```
Expected: `Summary: 1 packages finished`

- [ ] **Step 5: Commit**

```bash
git add ros2_ws/src/burgerbot/package.xml ros2_ws/src/burgerbot/CMakeLists.txt
git commit -m "feat(burgerbot): add Gazebo deps and worlds install directory"
```

---

## Task 11: Gazebo World SDF

**Files:**
- Create: `ros2_ws/src/burgerbot/worlds/burgerbot_world.sdf`

6×6m room with walls and one obstacle. Key difference from existing `burgerbot_gazebo/worlds/burgerbot.sdf`: adds `gz-sim-sensors-system` and `gz-sim-imu-system` plugins so the LiDAR and IMU sensors actually work.

- [ ] **Step 1: Write worlds/burgerbot_world.sdf**

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="burgerbot_world">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Core sim systems -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>

    <!-- Sensor systems (REQUIRED for LiDAR + IMU to produce data) -->
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Walls: 6 × 6 m room -->
    <model name="north_wall">
      <static>true</static><pose>0 3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>6.2 0.1 1</size></box></geometry></collision>
        <visual   name="v"><geometry><box><size>6.2 0.1 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material></visual>
      </link>
    </model>

    <model name="south_wall">
      <static>true</static><pose>0 -3 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>6.2 0.1 1</size></box></geometry></collision>
        <visual   name="v"><geometry><box><size>6.2 0.1 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material></visual>
      </link>
    </model>

    <model name="east_wall">
      <static>true</static><pose>3 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.1 6.2 1</size></box></geometry></collision>
        <visual   name="v"><geometry><box><size>0.1 6.2 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material></visual>
      </link>
    </model>

    <model name="west_wall">
      <static>true</static><pose>-3 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.1 6.2 1</size></box></geometry></collision>
        <visual   name="v"><geometry><box><size>0.1 6.2 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material></visual>
      </link>
    </model>

    <!-- Box obstacle for richer SLAM maps -->
    <model name="box_obstacle">
      <static>true</static><pose>1.5 0.5 0.25 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.3 0.3 0.5</size></box></geometry></collision>
        <visual   name="v"><geometry><box><size>0.3 0.3 0.5</size></box></geometry>
          <material><ambient>0.2 0.5 0.8 1</ambient></material></visual>
      </link>
    </model>

  </world>
</sdf>
```

- [ ] **Step 2: Commit**

```bash
git add ros2_ws/src/burgerbot/worlds/burgerbot_world.sdf
git commit -m "feat(burgerbot): add Gazebo Harmonic world SDF (6x6m room + sensors plugins)"
```

---

## Task 12: Simulation URDF with Gazebo Plugins

**Files:**
- Create: `ros2_ws/src/burgerbot/urdf/burgerbot_sim.urdf.xacro`

Includes the base URDF then adds `<gazebo>` plugin tags. The base `burgerbot.urdf.xacro` stays clean (no Gazebo specifics). Gazebo Harmonic processes `<gazebo>` elements when spawning via `ros_gz_sim create`.

Key design:
- Diff drive publishes `/wheel_odom` at 50 Hz; **`<publish_odom_tf>false`** — EKF owns `odom→base_link`
- LiDAR publishes Gazebo-internal topic `scan` — bridged to ROS `/scan`
- IMU publishes Gazebo-internal topic `imu/raw` — bridged to ROS `/imu/raw`
- Wheel params match firmware: `wheel_separation=0.282`, `wheel_radius=0.0625`

- [ ] **Step 1: Write urdf/burgerbot_sim.urdf.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="burgerbot">

  <!-- Base geometry: links, joints, visuals, inertials -->
  <xacro:include filename="$(find burgerbot)/urdf/burgerbot.urdf.xacro"/>

  <!-- ── Differential Drive ────────────────────────────────────────────────
       Subscribes: /cmd_vel
       Publishes:  /wheel_odom (Odometry, 50 Hz)
       TF:         publish_odom_tf=false → EKF publishes odom→base_link
  ──────────────────────────────────────────────────────────────────────── -->
  <gazebo>
    <plugin name="gz::sim::systems::DiffDrive"
            filename="gz-sim-diff-drive-system">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.282</wheel_separation>
      <wheel_radius>0.0625</wheel_radius>
      <odom_publish_frequency>50</odom_publish_frequency>
      <topic>/cmd_vel</topic>
      <odom_topic>/wheel_odom</odom_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>false</publish_odom_tf>
    </plugin>
  </gazebo>

  <!-- ── GPU LiDAR on 'lidar' link ────────────────────────────────────────
       gz_frame_id matches URDF link name → scan.header.frame_id = 'lidar'
       Bridged: Gazebo 'scan' → ROS '/scan'
  ──────────────────────────────────────────────────────────────────────── -->
  <gazebo reference="lidar">
    <sensor name="gpu_lidar" type="gpu_lidar">
      <always_on>1</always_on>
      <update_rate>10</update_rate>
      <topic>scan</topic>
      <gz_frame_id>lidar</gz_frame_id>
      <visualize>true</visualize>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159265</min_angle>
            <max_angle>3.14159265</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.12</min>
          <max>3.5</max>
          <resolution>0.015</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </lidar>
    </sensor>
  </gazebo>

  <!-- ── IMU on 'imu_link' ─────────────────────────────────────────────────
       Bridged: Gazebo 'imu/raw' → ROS '/imu/raw'
       imu_filter_madgwick filters → /imu/data → EKF
  ──────────────────────────────────────────────────────────────────────── -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>1</always_on>
      <update_rate>50</update_rate>
      <topic>imu/raw</topic>
      <gz_frame_id>imu_link</gz_frame_id>
      <imu>
        <angular_velocity>
          <x><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev><bias_mean>7.5e-6</bias_mean><bias_stddev>8e-7</bias_stddev></noise></x>
          <y><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev><bias_mean>7.5e-6</bias_mean><bias_stddev>8e-7</bias_stddev></noise></y>
          <z><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev><bias_mean>7.5e-6</bias_mean><bias_stddev>8e-7</bias_stddev></noise></z>
        </angular_velocity>
        <linear_acceleration>
          <x><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev><bias_mean>0.1</bias_mean><bias_stddev>0.001</bias_stddev></noise></x>
          <y><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev><bias_mean>0.1</bias_mean><bias_stddev>0.001</bias_stddev></noise></y>
          <z><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev><bias_mean>0.1</bias_mean><bias_stddev>0.001</bias_stddev></noise></z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>

</robot>
```

- [ ] **Step 2: Validate xacro expansion**

```bash
cd ros2_ws && source install/setup.bash
xacro src/burgerbot/urdf/burgerbot_sim.urdf.xacro > /tmp/sim_check.urdf
check_urdf /tmp/sim_check.urdf
```
Expected: `Successfully Parsed` — same 7-link tree as base URDF.

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/burgerbot/urdf/burgerbot_sim.urdf.xacro
git commit -m "feat(burgerbot): add sim URDF with Gazebo Harmonic diff-drive + lidar + IMU plugins"
```

---

## Task 13: ros_gz_bridge Config

**Files:**
- Create: `ros2_ws/src/burgerbot/config/ros_gz_bridge.yaml`

Maps Gazebo-internal topics to ROS 2 topics. Used by `gazebo.launch.py`.

- [ ] **Step 1: Write config/ros_gz_bridge.yaml**

```yaml
# ros_gz_bridge topic bridge configuration
# GZ_TO_ROS: Gazebo internal → ROS 2 topic
# ROS_TO_GZ: ROS 2 topic → Gazebo internal

- topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS

- topic_name: /scan
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS

- topic_name: /imu/raw
  ros_type_name: sensor_msgs/msg/Imu
  gz_type_name: gz.msgs.IMU
  direction: GZ_TO_ROS

- topic_name: /wheel_odom
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS

- topic_name: /cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: gz.msgs.Twist
  direction: ROS_TO_GZ

- topic_name: /tf
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
```

- [ ] **Step 2: Rebuild and verify file installed**

```bash
cd ros2_ws
colcon build --packages-select burgerbot --symlink-install
ls install/burgerbot/share/burgerbot/config/ros_gz_bridge.yaml
```
Expected: file present.

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/burgerbot/config/ros_gz_bridge.yaml
git commit -m "feat(burgerbot): add ros_gz_bridge YAML config for sim topic bridging"
```

---

## Task 14: gazebo.launch.py

**Files:**
- Create: `ros2_ws/src/burgerbot/launch/gazebo.launch.py`

Starts Gazebo, spawns robot, bridges topics, starts IMU filter + EKF. `use_sim_time` defaults `true`. Standalone — does not include SLAM or Nav2.

- [ ] **Step 1: Write launch/gazebo.launch.py**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import (
    LaunchConfiguration, Command, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('burgerbot')

    use_sim_time = LaunchConfiguration('use_sim_time')

    world_file = PathJoinSubstitution([pkg, 'worlds', 'burgerbot_world.sdf'])
    urdf_sim   = PathJoinSubstitution([pkg, 'urdf',   'burgerbot_sim.urdf.xacro'])
    bridge_cfg = PathJoinSubstitution([pkg, 'config', 'ros_gz_bridge.yaml'])
    ekf_cfg    = PathJoinSubstitution([pkg, 'config', 'ekf.yaml'])

    robot_description = {'robot_description': Command(['xacro ', urdf_sim])}

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock (always true for sim)'),

        # ── 1. Gazebo Harmonic ────────────────────────────────────────────
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
        ),

        # ── 2. Robot State Publisher ──────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # ── 3. Spawn robot (delayed 3s to let Gazebo load the world) ─────
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_burgerbot',
                    arguments=[
                        '-name',  'burgerbot',
                        '-topic', 'robot_description',
                        '-z',     '0.08',
                    ],
                    output='screen',
                ),
            ],
        ),

        # ── 4. Bridge Gazebo topics ↔ ROS 2 ──────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            parameters=[{'config_file': bridge_cfg}],
            output='screen',
        ),

        # ── 5. IMU filter: /imu/raw (Gazebo) → /imu/data ─────────────────
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag':      False,
                'publish_tf':   False,
                'world_frame':  'enu',
                'use_sim_time': use_sim_time,
            }],
            remappings=[('/imu/data_raw', '/imu/raw')],
            output='screen',
        ),

        # ── 6. EKF: /wheel_odom + /imu/data → /odometry/filtered + TF ───
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[ekf_cfg, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
```

- [ ] **Step 2: Verify launch args**

```bash
ros2 launch burgerbot gazebo.launch.py --show-args
```
Expected: `use_sim_time` listed with default `true`.

- [ ] **Step 3: Commit**

```bash
git add ros2_ws/src/burgerbot/launch/gazebo.launch.py
git commit -m "feat(burgerbot): add Gazebo simulation base launch"
```

---

## Task 15: Simulation Compound Launchers

**Files:**
- Create: `ros2_ws/src/burgerbot/launch/sim_slam.launch.py`
- Create: `ros2_ws/src/burgerbot/launch/sim_nav.launch.py`

### 15a — sim_slam.launch.py

Gazebo + SLAM mapping. Drive with teleop to build a map, then save with `slam_toolbox map_saver_cli`.

- [ ] **Step 1: Write launch/sim_slam.launch.py**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg        = FindPackageShare('burgerbot')
    launch_dir = PathJoinSubstitution([pkg, 'launch'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, f'/{name}']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'),

        include('gazebo.launch.py'),
        include('slam.launch.py'),
    ])
```

### 15b — sim_nav.launch.py

Gazebo + SLAM + Nav2. Full autonomous navigation in simulation.

- [ ] **Step 2: Write launch/sim_nav.launch.py**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg        = FindPackageShare('burgerbot')
    launch_dir = PathJoinSubstitution([pkg, 'launch'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, f'/{name}']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Gazebo simulation clock'),

        include('gazebo.launch.py'),
        include('slam.launch.py'),
        include('navigation.launch.py'),
    ])
```

- [ ] **Step 3: Final sim build + verify all launch args**

```bash
cd ros2_ws
colcon build --packages-select burgerbot --symlink-install
source install/setup.bash
ros2 launch burgerbot sim_slam.launch.py --show-args
ros2 launch burgerbot sim_nav.launch.py --show-args
```
Expected: both show `use_sim_time` with default `true`.

- [ ] **Step 4: Commit**

```bash
git add ros2_ws/src/burgerbot/launch/sim_slam.launch.py \
        ros2_ws/src/burgerbot/launch/sim_nav.launch.py
git commit -m "feat(burgerbot): add sim compound launchers (sim_slam, sim_nav)"
```

---

## Task 16: Dockerfile + Docker Compose

**Files:**
- Create: `docker/Dockerfile`
- Create: `docker/entrypoint.sh`
- Create: `docker/docker-compose.yml`

Based on `ros:jazzy-ros-base` (Ubuntu 24.04 Noble). Builds the full workspace inside the image. Three compose services: `headless` (CLI only), `sim` (Gazebo with X11), `robot` (real hardware with serial device access).

- [ ] **Step 1: Create docker/ directory**

```bash
mkdir -p docker
```

- [ ] **Step 2: Write docker/Dockerfile**

```dockerfile
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ── ROS2 packages + Gazebo Harmonic ──────────────────────────────────────
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

# ── Workspace ────────────────────────────────────────────────────────────
WORKDIR /ros2_ws
COPY ros2_ws/src ./src/

RUN bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    colcon build \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release"

# ── Entrypoint ────────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

- [ ] **Step 3: Write docker/entrypoint.sh**

```bash
#!/bin/bash
set -e

# ROS2 base
source /opt/ros/jazzy/setup.bash

# Workspace overlay
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

# Gazebo resource path
export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/burgerbot/share/burgerbot/worlds

exec "$@"
```

- [ ] **Step 4: Write docker/docker-compose.yml**

```yaml
version: "3.8"

x-burgerbot: &base
  build:
    context: ..
    dockerfile: docker/Dockerfile
  network_mode: host
  ipc: host
  environment:
    - ROS_DOMAIN_ID=0

services:

  # ── CLI shell, no display ─────────────────────────────────────────────
  headless:
    <<: *base
    container_name: burgerbot_headless
    stdin_open: true
    tty: true

  # ── Gazebo + RViz2, requires X11 on host ─────────────────────────────
  # Linux: run `xhost +local:docker` before starting
  sim:
    <<: *base
    container_name: burgerbot_sim
    stdin_open: true
    tty: true
    environment:
      - ROS_DOMAIN_ID=0
      - DISPLAY=${DISPLAY:-:0}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ${HOME}/maps:/root/maps
    command: >
      bash -c "source /ros2_ws/install/setup.bash &&
               ros2 launch burgerbot sim_nav.launch.py"

  # ── Real robot: access /dev/ttyACM* for ESP32 + LiDAR ────────────────
  robot:
    <<: *base
    container_name: burgerbot_robot
    stdin_open: true
    tty: true
    privileged: true
    devices:
      - /dev/ttyACM0:/dev/ttyACM0
      - /dev/ttyACM1:/dev/ttyACM1
    volumes:
      - ${HOME}/maps:/root/maps
    command: >
      bash -c "source /ros2_ws/install/setup.bash &&
               ros2 launch burgerbot slam_nav.launch.py"
```

- [ ] **Step 5: Commit**

```bash
git add docker/
git commit -m "feat: add Dockerfile + docker-compose (headless, sim, robot services)"
```

---

## Task 17: Fix installfff + Add Install Scripts

**Context:** `installfff/setup.bash` line 25 has a hardcoded path to
`/home/aimen/Documents/PlatformIO/Projects/burgerbot/ros2_ws/install`.
This breaks on every machine except the original. Fix: use relative path from script location.

`installfff/` is a colcon install output — normally gitignored. It was committed intentionally as a convenience setup overlay. The fix preserves that intent while making it portable.

### 17a — Fix installfff setup scripts

- [ ] **Step 1: Fix installfff/setup.bash (line 25)**

Change:
```bash
COLCON_CURRENT_PREFIX="/home/aimen/Documents/PlatformIO/Projects/burgerbot/ros2_ws/install"
_colcon_prefix_chain_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"
```

To:
```bash
# Resolve ros2_ws/install relative to this script's location
_bb_ws_install="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")/../ros2_ws/install" > /dev/null 2>&1 && pwd)"
if [ -n "$_bb_ws_install" ] && [ -d "$_bb_ws_install" ]; then
  COLCON_CURRENT_PREFIX="$_bb_ws_install"
  _colcon_prefix_chain_bash_source_script "$COLCON_CURRENT_PREFIX/local_setup.bash"
fi
unset _bb_ws_install
```

- [ ] **Step 2: Fix installfff/setup.sh (same pattern, use $0)**

Change the same hardcoded block to:
```sh
_bb_ws_install="$(cd "$(dirname "$0")/../ros2_ws/install" > /dev/null 2>&1 && pwd)"
if [ -n "$_bb_ws_install" ] && [ -d "$_bb_ws_install" ]; then
  COLCON_CURRENT_PREFIX="$_bb_ws_install"
  _colcon_prefix_chain_sh_source_script "$COLCON_CURRENT_PREFIX/local_setup.sh"
fi
unset _bb_ws_install
```

- [ ] **Step 3: Fix installfff/setup.zsh (use %x for zsh)**

Change the hardcoded block to:
```zsh
_bb_ws_install="$(builtin cd "$(dirname "${(%):-%x}")/../ros2_ws/install" > /dev/null 2>&1 && pwd)"
if [[ -n "$_bb_ws_install" && -d "$_bb_ws_install" ]]; then
  COLCON_CURRENT_PREFIX="$_bb_ws_install"
  _colcon_prefix_chain_zsh_source_script "$COLCON_CURRENT_PREFIX/local_setup.zsh"
fi
unset _bb_ws_install
```

- [ ] **Step 4: Fix installfff/setup.ps1 (use $PSScriptRoot)**

Find the hardcoded path line and replace with:
```powershell
$_bbWsInstall = Join-Path $PSScriptRoot ".." "ros2_ws" "install"
$_bbWsInstall = try { (Resolve-Path $_bbWsInstall).Path } catch { $null }
if ($_bbWsInstall -and (Test-Path $_bbWsInstall)) {
  $COLCON_CURRENT_PREFIX = $_bbWsInstall
  . "$COLCON_CURRENT_PREFIX/local_setup.ps1"
}
Remove-Variable _bbWsInstall
```

### 17b — Install scripts

- [ ] **Step 5: Create scripts/ directory**

```bash
mkdir -p scripts
```

- [ ] **Step 6: Write scripts/install_deps.sh**

Fresh Ubuntu 24.04 setup — installs ROS2 Jazzy + Gazebo Harmonic + all burgerbot deps:

```bash
#!/bin/bash
# Install ROS2 Jazzy + Gazebo Harmonic + burgerbot dependencies
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
echo "==> Done! Next step:"
echo "      bash scripts/build.sh"
```

- [ ] **Step 7: Write scripts/build.sh**

```bash
#!/bin/bash
# Build the burgerbot ros2_ws workspace
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../ros2_ws"

echo "==> Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "==> Building workspace: $WS_DIR"
cd "$WS_DIR"
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "==> Build complete. Source with:"
echo "      source $WS_DIR/install/setup.bash"
echo ""
echo "==> Or use the fixed convenience script:"
echo "      source $SCRIPT_DIR/../installfff/setup.bash"
```

- [ ] **Step 8: Make scripts executable**

```bash
chmod +x scripts/install_deps.sh scripts/build.sh
```

- [ ] **Step 9: Commit**

```bash
git add installfff/setup.bash installfff/setup.sh installfff/setup.zsh installfff/setup.ps1
git add scripts/install_deps.sh scripts/build.sh
git commit -m "fix(installfff): replace hardcoded /home/aimen paths with relative; add install + build scripts"
```

---

## Complete Usage Cheatsheet

### Docker (no ROS2 install needed)

```bash
# Build image (~10 min first time)
docker compose -f docker/docker-compose.yml build

# Simulation: Gazebo + SLAM + Nav2
# Linux only: xhost +local:docker first
docker compose -f docker/docker-compose.yml run --rm sim

# Get a shell for step-by-step work
docker compose -f docker/docker-compose.yml run --rm headless bash
# Inside container:
ros2 launch burgerbot sim_nav.launch.py
```

### Simulation (Ubuntu 24.04, ROS2 Jazzy installed)

```bash
# First time only
bash scripts/install_deps.sh
bash scripts/build.sh
source ros2_ws/install/setup.bash

# One-command: Gazebo + SLAM + Nav2
ros2 launch burgerbot sim_nav.launch.py

# Step-by-step (4 terminals):
# T1: ros2 launch burgerbot gazebo.launch.py
# T2: ros2 launch burgerbot slam.launch.py use_sim_time:=true
# T3: ros2 launch burgerbot navigation.launch.py use_sim_time:=true
# T4: ros2 launch burgerbot debug.launch.py use_sim_time:=true
# T5: ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map
ros2 run slam_toolbox map_saver_cli -f ~/maps/my_map
```

### Real robot

```bash
source ros2_ws/install/setup.bash
# or: source installfff/setup.bash  ← now portable (relative paths)

ros2 launch burgerbot slam_nav.launch.py

# Debug hardware:
ros2 topic echo /scan
ros2 topic echo /imu/data
ros2 topic echo /odometry/filtered
```

---

## Self-Review Notes (Simulation + Docker)

- **Diff drive TF conflict:** `<publish_odom_tf>false</publish_odom_tf>` in sim URDF → EKF is sole `odom→base_link` publisher in both sim and real ✓
- **LiDAR frame in sim:** `gz_frame_id>lidar</gz_frame_id>` → LaserScan `frame_id=lidar` → matches URDF → SLAM TF chain works ✓
- **Clock sync:** `/clock` bridged + `use_sim_time=true` on all nodes → all timestamps aligned ✓
- **Sim parity:** Both sim and real use same EKF, SLAM, Nav2 configs — only bringup differs ✓
- **Docker display:** `sim` service passes `DISPLAY` + X11 socket — run `xhost +local:docker` on Linux host; WSL2 needs VcXsrv or WSLg configured
- **installfff portability:** Fixed to compute `ros2_ws/install` relative to script location — requires workspace to be built first with `scripts/build.sh`
- **Verify Gazebo executables:** `ros2 pkg executables ros_gz_sim` → `create`; `ros2 pkg executables ros_gz_bridge` → `parameter_bridge`
- **Gazebo Harmonic package name:** `gz-harmonic` in apt — if not available, try `gz-sim8`
