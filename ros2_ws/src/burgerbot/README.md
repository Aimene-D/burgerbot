# burgerbot

ROS 2 Jazzy package for the burgerbot differential-drive robot. Runs on Ubuntu 24.04 natively or in Docker (no local ROS 2 required).

> See [ARCHITECTURE.md](ARCHITECTURE.md) for a full explanation of how the stack works.

---

## Quick-Start Options

| Situation | Path |
|-----------|------|
| No ROS 2 installed | [Docker](#docker) |
| Ubuntu 24.04, bare metal | [Native Install](#native-install) |
| Just want to simulate | [Gazebo Simulation](#gazebo-simulation) |
| Physical robot | [Hardware Bringup](#hardware-bringup) |

---

## Docker

### Prerequisites
- Docker Desktop (Windows/Mac) or Docker Engine (Linux)
- For GUI (Gazebo/RViz) on Linux: X11 running

### Build the image

```bash
cd burgerbot
docker compose -f docker/docker-compose.yml build
```

### Run

```bash
# Gazebo + SLAM + Nav2 (simulation, GUI via X11)
docker compose -f docker/docker-compose.yml run sim

# Physical robot with SLAM (requires /dev/ttyACM0 + /dev/ttyACM1)
docker compose -f docker/docker-compose.yml run robot

# Headless CLI session
docker compose -f docker/docker-compose.yml run headless bash
```

> **Windows GUI note:** Docker GUI (Gazebo/RViz) requires an X server such as VcXsrv or WSLg. Set `DISPLAY=host.docker.internal:0` before running the `sim` service.

---

## Native Install

### 1. Install dependencies (Ubuntu 24.04 only)

```bash
bash scripts/install_deps.sh
```

This adds the ROS 2 Jazzy and Gazebo Harmonic apt repos and installs all required packages.

### 2. Build the workspace

```bash
bash scripts/build.sh
```

### 3. Source the workspace

```bash
# bash
source installfff/setup.bash

# zsh
source installfff/setup.zsh

# plain sh
. installfff/setup.sh
```

> The `installfff/` scripts are portable — they resolve the workspace path relative to their own location, so they work from any clone location.

---

## Launch Files

All launch files live in `launch/`. Each one accepts `use_sim_time:=true/false`.

### Atomic launchers (single concern)

| File | What it starts |
|------|----------------|
| `bringup.launch.py` | micro-ROS agent, robot_state_publisher, LiDAR driver, IMU filter, EKF |
| `slam.launch.py` | SLAM Toolbox (async mapping mode) |
| `localization.launch.py` | SLAM Toolbox (localization mode) — requires `map:=<path>` |
| `navigation.launch.py` | Full Nav2 stack |
| `debug.launch.py` | RViz2 with pre-configured display |
| `gazebo.launch.py` | Gazebo Harmonic, robot spawn, ros_gz_bridge, IMU filter, EKF |

### Compound launchers (full workflows)

| File | What it includes | Use case |
|------|-----------------|----------|
| `slam_nav.launch.py` | bringup + slam + navigation | Hardware: map while navigating |
| `nav_saved_map.launch.py` | bringup + localization + navigation | Hardware: navigate on a saved map |
| `sim_slam.launch.py` | gazebo + slam | Simulation: mapping run |
| `sim_nav.launch.py` | gazebo + slam + navigation | Simulation: full autonomous nav |

---

## Hardware Bringup

### Connections

| Device | Default port | Launch arg |
|--------|-------------|-----------|
| ESP32-S3 (firmware) | `/dev/ttyACM0` | `serial_port:=` |
| LDS02RR LiDAR | `/dev/ttyACM1` | `lidar_port:=` |

### Map while navigating (SLAM + Nav2)

```bash
ros2 launch burgerbot slam_nav.launch.py
```

Open RViz in a second terminal:

```bash
ros2 launch burgerbot debug.launch.py
```

Save map when done:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Navigate on a saved map

```bash
ros2 launch burgerbot nav_saved_map.launch.py map:=~/maps/my_map
```

> `map` is the path **without extension** — SLAM Toolbox appends `.yaml` and `.posegraph` automatically.

### Hardware-only bringup (no autonomy)

```bash
ros2 launch burgerbot bringup.launch.py
```

Drive manually:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Gazebo Simulation

No physical hardware needed. Simulates LiDAR, IMU, and differential drive inside a 6×6 m room.

### SLAM mapping in simulation

```bash
ros2 launch burgerbot sim_slam.launch.py
```

### Full autonomous navigation in simulation

```bash
ros2 launch burgerbot sim_nav.launch.py
```

### Debug overlay (RViz)

```bash
ros2 launch burgerbot debug.launch.py use_sim_time:=true
```

Send a goal via RViz **2D Nav Goal** tool or:

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}" \
  --once
```

---

## Package Structure

```
ros2_ws/src/burgerbot/
├── config/
│   ├── ekf.yaml                    # robot_localization EKF (wheel_odom + imu/data)
│   ├── nav2.yaml                   # Nav2 stack (controller, planner, BT navigator)
│   ├── slam_toolbox.yaml           # SLAM Toolbox — async mapping
│   ├── slam_toolbox_localization.yaml  # SLAM Toolbox — localization mode
│   ├── ros_gz_bridge.yaml          # Gazebo ↔ ROS 2 topic bridge
│   └── rviz/burgerbot.rviz         # RViz display config
├── launch/
│   ├── bringup.launch.py           # hardware sensors + EKF
│   ├── slam.launch.py
│   ├── localization.launch.py
│   ├── navigation.launch.py
│   ├── debug.launch.py
│   ├── gazebo.launch.py            # Gazebo sim equivalent of bringup
│   ├── slam_nav.launch.py          # compound: bringup + slam + nav
│   ├── nav_saved_map.launch.py     # compound: bringup + localization + nav
│   ├── sim_slam.launch.py          # compound: gazebo + slam
│   └── sim_nav.launch.py           # compound: gazebo + slam + nav
├── maps/                           # drop saved maps here
├── urdf/
│   ├── burgerbot.urdf.xacro        # physical robot description
│   └── burgerbot_sim.urdf.xacro   # + Gazebo plugin elements
└── worlds/
    └── burgerbot_world.sdf         # 6×6 m test room
```

---

## Robot Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Wheel radius | 0.0625 m | firmware |
| Wheel separation | 0.282 m | firmware |
| Robot radius | 0.105 m | nav2.yaml |
| LiDAR frame | `lidar` | lds02rr_driver |
| LiDAR max range | 3.5 m | slam_toolbox.yaml |
| EKF frequency | 50 Hz | ekf.yaml |
| Nav2 max linear vel | 0.2 m/s | nav2.yaml |

---

## Troubleshooting

**micro-ROS agent can't connect**
```bash
# Check which port the ESP32 is on
ls /dev/ttyACM*
ros2 launch burgerbot bringup.launch.py serial_port:=/dev/ttyACM0
```

**No LiDAR scans**
```bash
ros2 topic echo /scan --once
# If empty, check port and baud rate
ros2 launch burgerbot bringup.launch.py lidar_port:=/dev/ttyACM1
```

**SLAM map not building**
```bash
# Verify scan arriving and TF tree is complete
ros2 topic hz /scan
ros2 run tf2_tools view_frames
```

**Nav2 nodes failing to activate**
- Check `bond_timeout` — set to 30 s in nav2.yaml, gives nodes time to start on slow hardware.
- Run `ros2 lifecycle list` to see which nodes are stuck.

**Gazebo sensors produce no data**
- Ensure world SDF includes `gz-sim-sensors-system` and `gz-sim-imu-system` plugins (already in `burgerbot_world.sdf`).
