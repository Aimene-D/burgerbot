# burgerbot

ROS 2 Jazzy package for the burgerbot differential-drive robot.
Tested on Ubuntu 24.04 (Noble) — native install or Docker.

> See [ARCHITECTURE.md](ARCHITECTURE.md) for system design and Mermaid diagrams.

---

## Quick-Start Options

| Situation | Path |
| --------- | ---- |
| Ubuntu 24.04, bare metal | [Native Install](#native-install) |
| Any machine, no ROS 2 needed | [Docker](#docker) |
| Just want to simulate | [Gazebo Simulation](#gazebo-simulation) |
| Physical robot | [Hardware Bringup](#hardware-bringup) |

---

## Native Install

### 1. Install dependencies

```bash
bash scripts/install_deps.sh
```

Adds ROS 2 Jazzy and Gazebo Harmonic apt repos and installs all required packages.
Run once on a fresh Ubuntu 24.04 machine.

### 2. Build the workspace

```bash
bash scripts/build.sh
```

### 3. Source the workspace

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

Add both lines to `~/.bashrc` to avoid re-sourcing every terminal.

---

## Docker

Runs on Ubuntu (tested on Ubuntu 24.04 with Docker Engine).

### Prerequisites

- Docker Engine: `sudo apt install docker.io docker-compose-plugin`
- Add yourself to the docker group: `sudo usermod -aG docker $USER` (then log out/in)

### Build

```bash
cd burgerbot
docker compose -f docker/docker-compose.yml build
```

> First build takes ~15 minutes — micro-ROS agent is compiled from source.
> Subsequent builds are cached. All services share one image (`burgerbot:latest`).

### Modular services

Every service runs with `network_mode: host` and the same `ROS_DOMAIN_ID`, so all
containers join **one ROS 2 graph**. Start pieces in separate terminals and they
talk to each other — bring up one component at a time when debugging.

| Service | What it runs | Display |
| ------- | ------------ | ------- |
| `shell` | Interactive CLI with workspace sourced (`ros2 topic …`, builds) | — |
| `gazebo` | Gazebo world + robot spawn + bridge + IMU filter + EKF only | headless |
| `gazebo-gui` | Same as `gazebo`, with the Gazebo GUI window | X11 |
| `rviz` | RViz2 only, pre-configured layout — visualization client | X11 |
| `sim-slam` | Gazebo + SLAM Toolbox (mapping) | headless |
| `sim-nav` | Gazebo + SLAM + Nav2 (full autonomous sim) | headless |
| `bringup` | Hardware sensors only (micro-ROS, LiDAR, IMU, EKF) | needs `/dev/ttyACM*` |
| `robot` | Hardware + SLAM + Nav2 (map while navigating) | needs `/dev/ttyACM*` |

Run any service (`--rm` cleans up the container on exit):

```bash
docker compose -f docker/docker-compose.yml run --rm <service>
```

### Debug one thing at a time

```bash
# Terminal 1 — just the simulated world, nothing else
docker compose -f docker/docker-compose.yml run --rm gazebo

# Terminal 2 — visualize what the sim is publishing
xhost +local:docker
docker compose -f docker/docker-compose.yml run --rm rviz

# Terminal 3 — inspect the live ROS 2 graph
docker compose -f docker/docker-compose.yml run --rm shell
# inside: ros2 topic list / ros2 topic hz /scan / ros2 topic echo /clock
```

### GUI services on Ubuntu

GUI services (`gazebo-gui`, `rviz`) forward the host X server. Allow local docker
clients once per login session:

```bash
xhost +local:docker
```

### Run: physical robot

```bash
docker compose -f docker/docker-compose.yml run --rm robot
```

Requires `/dev/ttyACM0` (ESP32) and `/dev/ttyACM1` (LiDAR) connected. Use the
`bringup` service first to confirm sensors work before adding SLAM/Nav2.

---

## Launch Files

All launch files are in `launch/`. Each accepts `use_sim_time:=true/false`.

### Atomic launchers

| File | What it starts |
| ---- | -------------- |
| `bringup.launch.py` | micro-ROS agent, robot_state_publisher, LiDAR driver, IMU filter, EKF |
| `slam.launch.py` | SLAM Toolbox (async mapping mode) |
| `localization.launch.py` | SLAM Toolbox (localization on saved map) — requires `map:=<path>` |
| `navigation.launch.py` | Full Nav2 stack |
| `debug.launch.py` | RViz2 with pre-configured display |
| `gazebo.launch.py` | Gazebo Harmonic, robot spawn, ros_gz_bridge, IMU filter, EKF |

### Compound launchers

| File | Includes | Use case |
| ---- | -------- | -------- |
| `slam_nav.launch.py` | bringup + slam + navigation | Hardware: map while navigating |
| `nav_saved_map.launch.py` | bringup + localization + navigation | Hardware: navigate saved map |
| `sim_slam.launch.py` | gazebo + slam | Sim: mapping run |
| `sim_nav.launch.py` | gazebo + slam + navigation | Sim: full autonomous navigation |

### Gazebo headless vs GUI

`gazebo.launch.py` (and all `sim_*.launch.py`) accept a `headless` argument:

```bash
# Default: server only, no display required
ros2 launch burgerbot sim_nav.launch.py headless:=true

# Open Gazebo GUI (requires DISPLAY)
ros2 launch burgerbot sim_nav.launch.py headless:=false
```

---

## Hardware Bringup

### Connections

| Device | Default port | Override arg |
| ------ | ------------ | ------------ |
| ESP32-S3 (micro-ROS firmware) | `/dev/ttyACM0` | `serial_port:=` |
| LDS02RR LiDAR | `/dev/ttyACM1` | `lidar_port:=` |

Add your user to the dialout group if you get serial port permission errors:

```bash
sudo usermod -aG dialout $USER
```

### Map while navigating (SLAM + Nav2)

```bash
ros2 launch burgerbot slam_nav.launch.py
```

Open RViz in a second terminal:

```bash
ros2 launch burgerbot debug.launch.py
```

Save the map when done exploring:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Navigate on a saved map

```bash
ros2 launch burgerbot nav_saved_map.launch.py map:=~/maps/my_map
```

`map` is the path **without extension** — SLAM Toolbox appends `.yaml` and `.posegraph`.

### Hardware-only bringup (no autonomy)

```bash
ros2 launch burgerbot bringup.launch.py
```

Manual driving:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Gazebo Simulation

No hardware needed. Simulates LiDAR, IMU, and differential drive in a 6×6 m room.

### Mapping in simulation

```bash
ros2 launch burgerbot sim_slam.launch.py
```

### Full autonomous navigation in simulation

```bash
ros2 launch burgerbot sim_nav.launch.py
```

### Expected startup sequence

The stack takes ~20 seconds to fully initialize:

1. Gazebo world loads and starts physics
2. Robot spawns, clock starts flowing via bridge
3. EKF starts once `/clock` arrives → publishes `odom→base_link`
4. SLAM Toolbox processes first `/scan` → publishes `map→odom`
5. Nav2 global costmap gets `map` TF — all warnings clear

`Timed out waiting for transform` warnings during the first ~20 s are normal.
If they persist past 60 s, check the topics below.

### Send a navigation goal

Via RViz **2D Nav Goal** tool, or:

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}" \
  --once
```

---

## Package Structure

```text
ros2_ws/src/burgerbot/
├── config/
│   ├── ekf.yaml                       # EKF (fuses /wheel_odom + /imu/data)
│   ├── nav2.yaml                      # Nav2 controller, planner, costmaps
│   ├── slam_toolbox.yaml              # SLAM Toolbox — mapping mode
│   ├── slam_toolbox_localization.yaml # SLAM Toolbox — localization mode
│   ├── ros_gz_bridge.yaml             # Gazebo <-> ROS 2 topic bridge
│   └── rviz/burgerbot.rviz            # RViz layout
├── launch/
│   ├── bringup.launch.py
│   ├── slam.launch.py
│   ├── localization.launch.py
│   ├── navigation.launch.py
│   ├── debug.launch.py
│   ├── gazebo.launch.py               # headless arg (default true)
│   ├── slam_nav.launch.py
│   ├── nav_saved_map.launch.py
│   ├── sim_slam.launch.py
│   └── sim_nav.launch.py
├── maps/                              # drop saved maps here
├── urdf/
│   ├── burgerbot.urdf.xacro           # physical robot description
│   └── burgerbot_sim.urdf.xacro      # + Gazebo plugin elements
└── worlds/
    └── burgerbot_world.sdf            # 6x6 m test room
```

---

## Robot Parameters

| Parameter | Value | Source |
| --------- | ----- | ------ |
| Wheel radius | 0.0625 m | firmware config.h |
| Wheel separation | 0.282 m | firmware config.h |
| Robot radius (Nav2) | 0.105 m | nav2.yaml |
| LiDAR frame | `lidar` | matches lds02rr_driver |
| LiDAR max range | 3.5 m | slam_toolbox.yaml |
| EKF frequency | 50 Hz | ekf.yaml |
| Controller max linear vel | 0.2 m/s | nav2.yaml |

---

## Troubleshooting

#### micro-ROS agent can't connect

```bash
ls /dev/ttyACM*
ros2 launch burgerbot bringup.launch.py serial_port:=/dev/ttyACM0
```

#### No LiDAR scans

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

#### SLAM map not building

```bash
ros2 run tf2_tools view_frames
ros2 topic hz /scan
```

#### Nav2 nodes failing to activate

```bash
ros2 lifecycle list
```

`bond_timeout` is 30 s in nav2.yaml — nodes on slow hardware need time to start.

#### Gazebo sensors produce no data in simulation

Ensure world SDF contains `gz-sim-sensors-system` and `gz-sim-imu-system`
plugins (already present in `burgerbot_world.sdf`).

#### `transform from base_link to map` warnings never clear

```bash
ros2 topic hz /scan
ros2 topic hz /clock
ros2 topic hz /wheel_odom
```

If `/clock` is silent, the Gazebo bridge failed to connect to the sim.
