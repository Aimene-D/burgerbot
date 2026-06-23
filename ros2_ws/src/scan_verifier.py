#!/usr/bin/env python3
"""LiDAR scan orientation verifier.

Subscribes to /scan, prints range values at key angles,
and draws a simple ASCII polar plot. Use this to confirm
the scan data is oriented correctly relative to the robot.

How to use:
  1. Place robot facing a wall (~1m away), no other obstacles nearby
  2. Run:  ros2 run burgerbot scan_verifier.py
  3. Verify output:
     - Wall in front → peak range at angle 0° = robot BACKWARD
     = Wall should appear at 180° (forward in robot frame)
     with shortest range reading at ~180°
     - If wall appears at 0° → scan is flipped 180°
     - If wall appears at 90° or 270° → scan is rotated 90°

Lidar is mounted backward (URDF: rpy="0 0 3.14159"):
  - lidar +X = robot -X (backward)
  - lidar +Y = robot -Y (rightward)
  - So wall in FRONT of robot → scan angle 180° (lidar -X)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanVerifier(Node):
    def __init__(self):
        super().__init__('scan_verifier')
        self.sub = self.create_subscription(
            LaserScan, '/scan', self._cb, 10)
        self._count = 0
        self.get_logger().info(
            'Scan verifier started — waiting for /scan...')

    def _cb(self, msg):
        self._count += 1
        if self._count > 5:
            return  # enough

        n = len(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        inc = msg.angle_increment

        self.get_logger().info(
            f'--- Scan #{self._count} ({n} pts, '
            f'min={angle_min:.3f}, max={angle_max:.3f}, '
            f'inc={inc:.5f}) ---')

        # Report ranges at cardinal directions in the lidar frame
        # lidar frame: 0° = +X, 90° = +Y, 180° = -X, 270° = -Y
        # URDF rotation: lidar +X = robot -X (backward)
        key_angles = {
            'lidar +X (robot BACK)': 0.0,
            'lidar +Y (robot RIGHT)': math.pi / 2,
            'lidar -X (robot FRONT)': math.pi,
            'lidar -Y (robot LEFT)': 3 * math.pi / 2,
        }
        for label, angle_rad in key_angles.items():
            # Find nearest index
            idx = round((angle_rad - angle_min) / inc)
            if 0 <= idx < n:
                val = msg.ranges[idx]
                val_str = f'{val:.3f} m' if math.isfinite(val) else 'INF'
                self.get_logger().info(f'  {label:30s} [{idx:3d}] = {val_str}')

        # ASCII polar plot (simplified 16-ray radar)
        rays = 16
        radar = [['  '] * rays for _ in range(rays // 2 + 1)]
        for i in range(rays):
            angle = angle_min + i * (2 * math.pi / rays)
            idx = round((angle - angle_min) / inc)
            if 0 <= idx < n and math.isfinite(msg.ranges[idx]):
                r = msg.ranges[idx]
                # Normalize: 0..3.5 m → radius 0..(rays//2)
                max_r = msg.range_max if msg.range_max > 0 else 3.5
                radius = min(int(r / max_r * (rays // 2)), rays // 2)
                x = int(rays // 2 + radius * math.sin(angle))
                y = int(rays // 2 - radius * math.cos(angle))
                if 0 <= x < rays and 0 <= y < len(radar):
                    radar[y][x] = '##'

        # Mark center
        cx, cy = rays // 2, rays // 2
        radar[cy][cx] = '++'

        self.get_logger().info('  Polar radar (top = forward):')
        for row in radar:
            line = '  ' + ''.join(row)
            self.get_logger().info(line)


def main():
    rclpy.init()
    node = ScanVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
