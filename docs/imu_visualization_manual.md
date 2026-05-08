# IMU visualization in RViz (manual workflow)

This project publishes raw IMU data on /imu/raw and magnetometer data on /imu/mag.
Host-side filtering with imu_filter_madgwick generates /imu/data.
This guide adds host-side visualization only.

## Constraints respected

- Reuse existing launch file.
- Manual commands only.

## New node

File: tools/imu_visualizer.py

- Input topic: /imu/data (sensor_msgs/msg/Imu)
- Output topic: /imu_visualization (visualization_msgs/msg/MarkerArray)
- Frame used: msg.header.frame_id (fallback: imu_link)

Markers published:

- Orientation block (cube)
- Linear acceleration arrow (red)
- Angular velocity arrow (blue)
- Local axes (X, Y, Z)
- Text with norms |a| and |w|

## Manual startup

Run each command in a separate terminal after sourcing ROS 2.

1. Start micro-ROS agent (adapt serial port):

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

2. Start existing robot visualization stack with IMU filter enabled:

```bash
ros2 launch moteur robot_viz.launch.py use_ekf:=false use_imu_filter:=true
```

3. Start IMU visualizer node:

```bash
python3 tools/imu_visualizer.py
```

4. Start RViz:

```bash
rviz2
```

## RViz setup

Add these displays:

- RobotModel
- TF
- MarkerArray, topic: /imu_visualization
- Odometry, topic: /wheel_odom (optional)

Suggested Fixed Frame:

- odom (if available)
- otherwise base_link

## Quick checks

```bash
ros2 topic list | grep -E '^/imu/raw$|^/imu/mag$|^/imu/data$|^/imu_visualization$|^/wheel_odom$'
ros2 topic hz /imu/raw
ros2 topic hz /imu/data
ros2 topic hz /wheel_odom
ros2 topic echo /imu/data --once
```

Expected:

- /imu/raw and /imu/data available
- /imu/data header frame_id is imu_link
- markers visible and updated in RViz

## Parameters (optional)

You can tune scaling values at runtime:

```bash
python3 tools/imu_visualizer.py --ros-args \
  -p accel_gain:=0.02 \
  -p gyro_gain:=0.12 \
  -p max_arrow_length:=0.25
```
