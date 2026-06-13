# burgerbot Architecture

How every component fits together — hardware path, simulation path, and the full autonomous navigation stack.

---

## 1. System Overview

Two runtime modes share the same SLAM, Nav2, and EKF configs. Only the sensor bringup differs.

```mermaid
flowchart TB
    subgraph HW["Hardware Mode (bringup.launch.py)"]
        ESP["ESP32-S3\nFirmware"] -->|"serial /dev/ttyACM0"| uROS["micro_ros_agent"]
        LIDAR["LDS02RR\nLiDAR"] -->|"serial /dev/ttyACM1"| LDS["lds02rr_node"]
    end

    subgraph SIM["Simulation Mode (gazebo.launch.py)"]
        GZ["Gazebo Harmonic\n(burgerbot_world.sdf)"]
        GZ -->|"gz topics"| BRIDGE["ros_gz_bridge"]
    end

    uROS -->|"/wheel_odom\n/imu/raw"| FUSE
    LDS -->|"/scan"| SLAM
    BRIDGE -->|"/wheel_odom\n/imu/raw\n/scan\n/cmd_vel"| FUSE

    subgraph FUSE["Sensor Fusion"]
        IMU["imu_filter_madgwick\n/imu/raw → /imu/data"]
        EKF["robot_localization EKF\n/wheel_odom + /imu/data → /odometry/filtered"]
        IMU --> EKF
    end

    EKF -->|"odom → base_link TF\n/odometry/filtered"| SLAM

    subgraph SLAM["Mapping / Localization"]
        SLAMT["slam_toolbox\n/scan + TF → /map\nmap → odom TF"]
    end

    SLAMT -->|"/map\nmap→odom TF"| NAV2

    subgraph NAV2["Nav2 Autonomous Navigation"]
        PLAN["planner_server\n(NavFn — global path)"]
        CTRL["controller_server\n(RegulatedPurePursuit)"]
        BT["bt_navigator\n(BehaviorTree executor)"]
        VS["velocity_smoother"]
        BT --> PLAN
        BT --> CTRL
        CTRL --> VS
    end

    VS -->|"/cmd_vel"| HW
    VS -->|"/cmd_vel"| BRIDGE
```

---

## 2. Coordinate Frames (TF Tree)

```mermaid
flowchart LR
    MAP["map"] -->|"slam_toolbox\npublishes map→odom"| ODOM["odom"]
    ODOM -->|"EKF ekf_node\npublishes odom→base_link"| BL["base_link"]
    BL -->|"robot_state_publisher\nstatic from URDF"| BF["base_footprint"]
    BL --> IMU["imu_link"]
    BL --> LL["lidar\n(LiDAR frame)"]
    BL --> WL["wheel_left_link"]
    BL --> WR["wheel_right_link"]
```

Key rules:
- **EKF owns `odom → base_link`** — Gazebo DiffDrive plugin has `publish_odom_tf=false` to avoid conflict.
- **slam_toolbox owns `map → odom`** — no AMCL in this stack.
- **`lidar` frame** matches the `frame_id` published by `lds02rr_driver` (was `base_scan` in old URDF — mismatch fixed).

---

## 3. Sensor Fusion (EKF)

```mermaid
flowchart LR
    FW["ESP32 Firmware\n(wheel encoders + MPU6050)"]

    FW -->|"/wheel_odom\nOdometry msg\n@ firmware rate"| MA["micro_ros_agent"]
    FW -->|"/imu/raw\nImu msg\n@ firmware rate"| MA

    MA -->|"ROS 2 DDS"| IMF["imu_filter_madgwick\nRemoves gyro drift\nworld_frame: ENU"]
    MA -->|"ROS 2 DDS"| EKF

    IMF -->|"/imu/data\nFiltered orientation\n+ angular velocity"| EKF

    EKF["robot_localization\nekf_node\n50 Hz\ntwo_d_mode: true"]

    EKF -->|"/odometry/filtered\nodom→base_link TF"| NAV2["Nav2 + slam_toolbox"]

    note1["Fused states:\n• VX from wheel_odom\n• VYaw from wheel_odom + imu/data"]
```

**What the EKF fuses:**

| Source | Fields used | Why |
|--------|-------------|-----|
| `/wheel_odom` | VX, VYaw | Forward velocity + turn rate from encoders |
| `/imu/data` | VYaw only | Gyro cross-checks turn rate, reduces encoder slip error |

Position (X, Y) is integrated from velocities — not measured directly.

---

## 4. SLAM Toolbox Modes

```mermaid
stateDiagram-v2
    [*] --> Mapping : slam.launch.py
    Mapping : async_slam_toolbox_node\nBuilds /map from /scan\nPublishes map→odom TF

    Mapping --> SaveMap : ros2 service call\n/slam_toolbox/save_map
    SaveMap --> [*]

    [*] --> Localization : localization.launch.py\nmap:=<path>
    Localization : localization_slam_toolbox_node\nLoads saved .yaml + .posegraph\nLocalizes without modifying map
```

- **Mapping mode**: scans continuously added to map, map grows.
- **Localization mode**: map is frozen, robot localizes within it. `map_file_name` injected at launch time — no hardcoded paths.

---

## 5. Nav2 Stack Internals

```mermaid
flowchart TB
    GOAL["/goal_pose\nPoseStamped"] --> BT

    subgraph Nav2["Nav2 (lifecycle-managed)"]
        BT["bt_navigator\nBehaviorTree executor\nNavigateToPose BT"]
        PLAN["planner_server\nNavFn (Dijkstra)\nGlobal costmap ← /map + /scan"]
        CTRL["controller_server\nRegulatedPurePursuit\nLocal costmap ← /scan"]
        BEHAV["behavior_server\nSpin / BackUp / Wait\n(recovery behaviors)"]
        VS["velocity_smoother\ncmd_vel_nav → cmd_vel"]

        BT -->|"ComputePathToPose"| PLAN
        BT -->|"FollowPath"| CTRL
        BT -->|"recovery"| BEHAV
        CTRL --> VS
    end

    VS -->|"/cmd_vel"| ROBOT["Robot / Gazebo"]
    PLAN -->|"global path /plan"| BT
    CTRL -->|"local path /local_plan"| VS

    MAP["/map"] --> PLAN
    MAP --> CTRL
    ODOM["/odometry/filtered"] --> CTRL
    SCAN["/scan"] --> CTRL
```

`velocity_smoother` remaps: `cmd_vel_nav` (Nav2 output) → smoothed → `cmd_vel` (robot input). Prevents wheel slip from abrupt velocity commands.

---

## 6. Gazebo Simulation Path

```mermaid
flowchart LR
    subgraph GZ["Gazebo Harmonic"]
        WORLD["burgerbot_world.sdf\n6×6 m room\n4 walls + 1 obstacle"]
        ROBOT["burgerbot_sim.urdf.xacro\nDiffDrive plugin\nGPU LiDAR plugin\nIMU plugin"]
    end

    WORLD --> ROBOT

    ROBOT -->|"gz/topic/wheel_odom"| BRIDGE
    ROBOT -->|"gz/topic/lidar/scan"| BRIDGE
    ROBOT -->|"gz/topic/imu/raw"| BRIDGE
    BRIDGE -->|"/wheel_odom\n/scan\n/imu/raw\n/clock"| ROS["ROS 2 topics"]
    ROS -->|"/cmd_vel"| BRIDGE
    BRIDGE -->|"gz/topic/cmd_vel"| ROBOT

    BRIDGE["ros_gz_bridge\n(ros_gz_bridge.yaml)"]
```

**Critical SDF plugins** (missing from original — sensors produced no data):
```xml
<plugin filename="gz-sim-sensors-system" .../>
<plugin filename="gz-sim-imu-system" .../>
```

DiffDrive plugin sets `<publish_odom_tf>false</publish_odom_tf>` so EKF remains the sole publisher of `odom → base_link`.

---

## 7. Launch File Composition

```mermaid
flowchart TB
    subgraph HW_COMPOUND["Hardware Compound Launchers"]
        SN["slam_nav.launch.py"]
        NSM["nav_saved_map.launch.py"]
    end

    subgraph SIM_COMPOUND["Simulation Compound Launchers"]
        SS["sim_slam.launch.py"]
        SNAV["sim_nav.launch.py"]
    end

    subgraph ATOMIC["Atomic Launchers"]
        BU["bringup.launch.py\nmicro-ROS + LiDAR + IMU + EKF"]
        SL["slam.launch.py"]
        LOC["localization.launch.py"]
        NAV["navigation.launch.py"]
        DBG["debug.launch.py\nRViz2"]
        GAZ["gazebo.launch.py\nGazebo + bridge + IMU + EKF"]
    end

    SN --> BU
    SN --> SL
    SN --> NAV

    NSM --> BU
    NSM --> LOC
    NSM --> NAV

    SS --> GAZ
    SS --> SL

    SNAV --> GAZ
    SNAV --> SL
    SNAV --> NAV
```

Each atomic launcher is independently runnable for debugging a single subsystem. Compound launchers just include the atomics via `IncludeLaunchDescription`.

---

## 8. Docker Services

```mermaid
flowchart LR
    subgraph DC["docker-compose.yml"]
        H["headless\nCLI shell\nno display"]
        S["sim\nsim_nav.launch.py\nX11 passthrough\nGazebo GUI"]
        R["robot\nslam_nav.launch.py\nprivileged\n/dev/ttyACM0 + ACM1"]
    end

    IMG["Dockerfile\nros:jazzy-ros-base\n+ gz-harmonic\n+ full Nav2 suite\n+ micro-ros-agent\n+ colcon build"]

    IMG --> H
    IMG --> S
    IMG --> R
```

All three services share the same image. The difference is the command and device/display passthrough.

---

## 9. Design Decisions

| Decision | Rationale |
|----------|-----------|
| Single package replacing 5 | Eliminates cross-package path fragility and version skew |
| EKF owns `odom→base_link` (not firmware) | Fused estimate is more accurate; firmware odometry used as input only |
| No AMCL | SLAM Toolbox localization mode provides `map→odom` TF directly, no separate localizer needed |
| `bond_timeout: 30.0` in Nav2 | Old value was `0.0` which hides lifecycle failures silently |
| `robot_radius: 0.105 m` | Actual chassis radius 0.0825 m + 0.02 m safety margin |
| `lidar` frame (not `base_scan`) | Matches `lds02rr_driver` frame_id; old URDF mismatch caused empty costmaps |
| Wheel geometry from firmware | `radius=0.0625, sep=0.282` — old URDFs had `0.065/0.24` causing odometry drift |
| `publish_odom_tf=false` in Gazebo DiffDrive | Prevents TF conflict with EKF on the same `odom→base_link` edge |
