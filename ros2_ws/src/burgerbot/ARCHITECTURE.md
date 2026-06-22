# burgerbot Architecture

How every component fits together — hardware path, simulation path, and the full autonomous navigation stack.

---

## 1. System Overview

Two runtime modes share identical EKF, SLAM, and Nav2 configs.
Only the sensor bringup layer differs.

```mermaid
flowchart TB
    subgraph HW["Hardware Mode (bringup.launch.py)"]
        ESP["ESP32-S3\nFirmware"] -->|"serial /dev/ttyACM0"| uROS["micro_ros_agent"]
        LIDAR["LDS02RR\nLiDAR"] -->|"serial /dev/ttyACM1"| LDS["lds02rr_node"]
    end

    subgraph SIM["Simulation Mode (gazebo.launch.py)"]
        GZ["Gazebo Harmonic\n(burgerbot_world.sdf)\nheadless or GUI"]
        GZ -->|"gz topics"| BRIDGE["ros_gz_bridge"]
    end

    uROS -->|"/wheel_odom\n/imu/raw"| FUSE
    LDS -->|"/scan"| SLAM
    BRIDGE -->|"/wheel_odom\n/imu/raw\n/scan\n/cmd_vel"| FUSE

    subgraph FUSE["Sensor Fusion"]
        IMU["imu_filter_madgwick\n/imu/raw -> /imu/data"]
        EKF["robot_localization EKF\n/wheel_odom + /imu/data -> /odometry/filtered"]
        IMU --> EKF
    end

    EKF -->|"odom -> base_link TF\n/odometry/filtered"| SLAM

    subgraph SLAM["Mapping / Localization"]
        SLAMT["slam_toolbox\n/scan + TF -> /map\nmap -> odom TF"]
    end

    SLAMT -->|"/map\nmap->odom TF"| NAV2

    subgraph NAV2["Nav2 Autonomous Navigation"]
        PLAN["planner_server\n(NavfnPlanner global path)"]
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
    MAP["map"] -->|"slam_toolbox\nmap->odom"| ODOM["odom"]
    ODOM -->|"EKF ekf_node\nodom->base_link"| BL["base_link"]
    BL -->|"robot_state_publisher\nstatic from URDF"| BF["base_footprint"]
    BL --> IMU["imu_link"]
    BL --> LL["lidar"]
    BL --> WL["wheel_left_link"]
    BL --> WR["wheel_right_link"]
```

Key rules:

- **EKF owns `odom -> base_link`** — Gazebo DiffDrive plugin has `publish_odom_tf=false` to avoid conflict.
- **slam_toolbox owns `map -> odom`** — no AMCL in this stack.
- **`lidar` frame** matches the `frame_id` published by `lds02rr_driver`.

---

## 3. Sensor Fusion (EKF)

```mermaid
flowchart LR
    FW["ESP32 Firmware\n(wheel encoders + MPU6050)"]

    FW -->|"/wheel_odom\nOdometry"| MA["micro_ros_agent"]
    FW -->|"/imu/raw\nImu"| MA

    MA -->|"ROS 2 DDS"| IMF["imu_filter_madgwick\nRemoves gyro drift\nworld_frame: ENU"]
    MA -->|"ROS 2 DDS"| EKF

    IMF -->|"/imu/data"| EKF

    EKF["robot_localization ekf_node\n50 Hz, two_d_mode: true"]

    EKF -->|"/odometry/filtered\nodom->base_link TF"| NAV2["Nav2 + slam_toolbox"]
```

| Source | Fields fused | Reason |
| ------ | ------------ | ------ |
| `/wheel_odom` | VX, VYaw | Forward velocity + turn rate from encoders |
| `/imu/data` | VYaw only | Gyro cross-checks turn rate, reduces encoder slip error |

All values in the `process_noise_covariance` matrix must be floats (`0.0` not `0`).
The RCL YAML parser rejects mixed integer/float sequences.

---

## 4. SLAM Toolbox Modes

```mermaid
stateDiagram-v2
    [*] --> Mapping : slam.launch.py
    Mapping : async_slam_toolbox_node
    Mapping : Builds /map from /scan
    Mapping : Publishes map to odom TF

    Mapping --> SaveMap : save_map service
    SaveMap --> [*]

    [*] --> Localization : localization.launch.py
    Localization : localization_slam_toolbox_node
    Localization : map arg sets the map path
    Localization : Loads saved .yaml + .posegraph
    Localization : Localizes without modifying map
```

`map_file_name` is injected at launch time via the `map:=` arg — no hardcoded paths in any config file.

---

## 5. Nav2 Stack Internals

```mermaid
flowchart TB
    GOAL["/goal_pose\nPoseStamped"] --> BT

    subgraph Nav2["Nav2 (lifecycle-managed, bond_timeout=30s)"]
        BT["bt_navigator\nBehaviorTree executor\nNavigateToPose BT"]
        PLAN["planner_server\nnav2_navfn_planner::NavfnPlanner\nGlobal costmap <- /map + /scan"]
        CTRL["controller_server\nRegulatedPurePursuit\nLocal costmap <- /scan"]
        BEHAV["behavior_server\nnav2_behaviors::Spin/BackUp/Wait"]
        VS["velocity_smoother\ncmd_vel_nav -> cmd_vel"]

        BT -->|"ComputePathToPose"| PLAN
        BT -->|"FollowPath"| CTRL
        BT -->|"recovery"| BEHAV
        CTRL --> VS
    end

    VS -->|"/cmd_vel"| ROBOT["Robot / Gazebo"]

    MAP["/map"] --> PLAN
    MAP --> CTRL
    ODOM["/odometry/filtered"] --> CTRL
    SCAN["/scan"] --> CTRL
```

`velocity_smoother` remaps `cmd_vel_nav` (Nav2 raw output) to `cmd_vel` (robot input)
to prevent wheel slip from abrupt velocity commands.

Plugin names use `::` separator (pluginlib convention in ROS 2 Jazzy):

- `nav2_navfn_planner::NavfnPlanner`
- `nav2_behaviors::Spin`, `nav2_behaviors::BackUp`, `nav2_behaviors::Wait`
- `nav2_bt_navigator::NavigateToPoseNavigator`

---

## 6. Gazebo Simulation Path

```mermaid
flowchart LR
    subgraph GZ["Gazebo Harmonic"]
        WORLD["burgerbot_world.sdf\n6x6 m room\n4 walls + 1 obstacle"]
        ROBOT["burgerbot_sim.urdf.xacro\nDiffDrive plugin\nGPU LiDAR plugin\nIMU plugin"]
    end

    WORLD --> ROBOT

    ROBOT -->|"gz/topic/wheel_odom"| BRIDGE
    ROBOT -->|"gz/topic/scan"| BRIDGE
    ROBOT -->|"gz/topic/imu/raw"| BRIDGE
    BRIDGE -->|"/wheel_odom\n/scan\n/imu/raw\n/clock"| ROS["ROS 2 topics"]
    ROS -->|"/cmd_vel"| BRIDGE
    BRIDGE -->|"gz/topic/cmd_vel"| ROBOT

    BRIDGE["ros_gz_bridge\n(ros_gz_bridge.yaml)"]
```

### Headless vs GUI mode

`gazebo.launch.py` supports a `headless` argument (default `true`):

```text
headless:=true   →  gz sim -s -r world.sdf   (server only, no Qt/X11)
headless:=false  →  gz sim -r world.sdf       (GUI, requires DISPLAY)
```

For GUI mode on Ubuntu, run `xhost +local:docker` before starting the container.

### Critical SDF plugins

These plugins must be in the world SDF or sensors produce no data:

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
```

DiffDrive plugin sets `<publish_odom_tf>false</publish_odom_tf>` so EKF
remains the sole publisher of `odom -> base_link`.

### Startup timing

The sim stack takes ~20 s to reach steady state:

1. Gazebo starts (headless or GUI)
2. Bridge connects → `/clock` flows
3. EKF receives clock → starts publishing `odom -> base_link`
4. SLAM receives `/scan` + TF → publishes `map -> odom`
5. Nav2 costmaps get both TFs → lifecycle activation completes

`Timed out waiting for transform` warnings during this window are normal.

---

## 7. Launch File Composition

```mermaid
flowchart TB
    subgraph HW_COMPOUND["Hardware Compound"]
        SN["slam_nav.launch.py"]
        NSM["nav_saved_map.launch.py"]
    end

    subgraph SIM_COMPOUND["Simulation Compound"]
        SS["sim_slam.launch.py"]
        SNAV["sim_nav.launch.py"]
    end

    subgraph ATOMIC["Atomic"]
        BU["bringup.launch.py"]
        SL["slam.launch.py"]
        LOC["localization.launch.py"]
        NAV["navigation.launch.py"]
        DBG["debug.launch.py"]
        GAZ["gazebo.launch.py"]
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

---

## 8. Docker Services

```mermaid
flowchart LR
    subgraph DC["docker-compose.yml"]
        H["headless\nCLI shell\nno display"]
        S["sim\nsim_nav.launch.py\nheadless:=true by default\nX11 optional"]
        R["robot\nslam_nav.launch.py\nprivileged\n/dev/ttyACM0 + ACM1"]
    end

    IMG["Dockerfile\nros:jazzy-ros-base\n+ Gazebo Harmonic repo\n+ gz-harmonic\n+ full Nav2 suite\n+ micro-ros-agent (source build)\n+ colcon build"]

    IMG --> H
    IMG --> S
    IMG --> R
```

`network_mode: host` means all ROS 2 topics (DDS) are visible on the host machine
without any port mapping.

---

## 9. Design Decisions

| Decision | Rationale |
| -------- | --------- |
| Single `burgerbot` package replacing 5 | Eliminates cross-package path fragility and version skew |
| EKF owns `odom->base_link` (not firmware) | Fused estimate more accurate; firmware odometry is input only |
| No AMCL | SLAM Toolbox localization mode provides `map->odom` directly |
| `bond_timeout: 30.0` in Nav2 | Old value was `0.0` which silently hides lifecycle failures |
| `robot_radius: 0.105 m` | Actual chassis 0.0825 m + 0.02 m safety margin |
| `lidar` frame (not `base_scan`) | Matches `lds02rr_driver` frame_id; wrong name → empty costmaps |
| `reverse_scan=true` in LiDAR driver | LDS02RR scans CW; LaserScan convention is CCW. The LDS library outputs raw CW angles (`cw=true`). Index is reversed via `(360 - raw) % 360` so CCW-increasing LaserScan angles map to correct physical directions. |
| Wheel geometry from firmware | `radius=0.0625, sep=0.282`; old URDFs had wrong values causing odometry drift |
| `publish_odom_tf=false` in Gazebo DiffDrive | Prevents TF conflict with EKF on `odom->base_link` |
| `ParameterValue(value_type=str)` on `robot_description` | ROS 2 Jazzy launch auto-parses `Command()` output as YAML; URDF XML breaks YAML parsing |
| `pi` removed from URDF properties | xacro in ROS 2 Jazzy provides `pi` as built-in; redefining it emits stderr → `Command()` throws |
| All covariance values as floats (`0.0`) | RCL YAML parser rejects mixed integer/float sequences in arrays |
| Plugin names use `::` separator | pluginlib in ROS 2 Jazzy uses `::` not `/` for class IDs |
| `headless:=true` default in gazebo.launch.py | Allows Docker usage without X11/display server |
| micro-ros-agent built from source | No deb package exists for ROS 2 Jazzy; snap not usable in Docker |
| Gazebo Harmonic apt repo added separately | `ros:jazzy-ros-base` only ships the ROS 2 repo; `gz-harmonic` is on osrfoundation |
