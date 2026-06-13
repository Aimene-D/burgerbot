# burgerbot

Differential-drive robot built on ROS 2 Jazzy + Gazebo Harmonic.

## Repository Layout

```
burgerbot/
├── ros2_ws/
│   └── src/
│       ├── burgerbot/          ← main ROS 2 package (launch, config, URDF, worlds)
│       └── lds02rr_driver/     ← LDS02RR LiDAR driver
├── firmware/                   ← ESP32-S3 PlatformIO firmware (micro-ROS)
├── docker/                     ← Dockerfile + docker-compose
├── scripts/
│   ├── install_deps.sh         ← bootstrap Ubuntu 24.04 with all apt deps
│   └── build.sh                ← colcon build
└── tools/
    └── imu_visualizer.py       ← debug utility
```

## Getting Started

See **[ros2_ws/src/burgerbot/README.md](ros2_ws/src/burgerbot/README.md)** for full build, install, and run instructions.

See **[ros2_ws/src/burgerbot/ARCHITECTURE.md](ros2_ws/src/burgerbot/ARCHITECTURE.md)** for system architecture and Mermaid diagrams.
