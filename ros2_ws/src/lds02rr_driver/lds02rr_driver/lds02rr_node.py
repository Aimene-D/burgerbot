#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math

TOTAL_SAMPLES = 360


class LDS02RRDriver(Node):
    """ROS2 driver for LDS02RR LiDAR via ESP32-C3 Super Mini (USB CDC).

    Reads text lines from the ESP32-C3 serial port:
        <angle_deg> <distance_mm> <quality>
        # scan <n> complete, freq=X.XX Hz

    Accumulates a full 360-point scan and publishes sensor_msgs/LaserScan.
    """

    def __init__(self):
        super().__init__('lds02rr_driver')
        self.declare_parameter('port',         '/dev/ttyACM0')
        self.declare_parameter('baud',         115200)
        self.declare_parameter('frame_id',     'lidar')
        self.declare_parameter('range_min',    0.12)
        self.declare_parameter('range_max',    3.5)
        self.declare_parameter('angle_offset', 0.0)

        port       = self.get_parameter('port').value
        baud       = self.get_parameter('baud').value
        self.frame_id      = self.get_parameter('frame_id').value
        self.range_min     = self.get_parameter('range_min').value
        self.range_max     = self.get_parameter('range_max').value
        self.angle_offset  = self.get_parameter('angle_offset').value

        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        # Scan buffer — filled rotation by rotation
        self.ranges       = [float('inf')] * TOTAL_SAMPLES
        self.intensities  = [0.0] * TOTAL_SAMPLES

        # Latest frequency reported by the LiDAR firmware
        self.scan_freq_hz = 5.0

        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'ESP32-C3 connecté sur {port} @ {baud} baud')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Impossible d\'ouvrir {port}: {e}')
            self.get_logger().fatal('Vérifie le périphérique (ls /dev/ttyACM* /dev/ttyUSB*)')
            raise SystemExit(1)

        # Read buffer — we read lines from serial
        self._line_buf = b''

        self.create_timer(0.001, self._read_cb)

    # ── Serial reader ─────────────────────────────────────────

    def _read_cb(self):
        try:
            nb = self.ser.in_waiting
            if nb:
                self._line_buf += self.ser.read(nb)
            self._process_lines()
        except serial.SerialException as e:
            self.get_logger().error(f'Erreur série: {e}')

    def _process_lines(self):
        while b'\n' in self._line_buf:
            line_bytes, self._line_buf = self._line_buf.split(b'\n', 1)
            line = line_bytes.decode('utf-8', errors='replace').strip()
            if line:
                self._parse_line(line)

    # ── Text parser ───────────────────────────────────────────

    def _parse_line(self, line):
        # Comment / status line
        if line.startswith('#'):
            # Detect end-of-scan marker
            if 'scan' in line and 'complete' in line:
                # Extract frequency if present
                if 'freq=' in line:
                    try:
                        freq_str = line.split('freq=')[1].split('Hz')[0].strip()
                        self.scan_freq_hz = float(freq_str)
                    except (ValueError, IndexError):
                        pass
                self._publish_scan()
            return

        # Data line:  angle_deg  dist_mm  quality
        parts = line.split()
        if len(parts) < 3:
            return

        try:
            angle_deg  = float(parts[0])
            dist_mm    = float(parts[1])
            quality    = float(parts[2])
        except ValueError:
            return

        idx = int(round(angle_deg)) % TOTAL_SAMPLES
        dist_m = dist_mm / 1000.0

        if dist_mm <= 0 or dist_m < self.range_min or dist_m > self.range_max:
            self.ranges[idx] = float('inf')
        else:
            self.ranges[idx] = dist_m
        self.intensities[idx] = quality

    # ── LaserScan publisher ───────────────────────────────────

    def _publish_scan(self):
        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min       = self.angle_offset
        msg.angle_max       = self.angle_offset + 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / TOTAL_SAMPLES
        msg.time_increment  = (1.0 / self.scan_freq_hz) / TOTAL_SAMPLES
        msg.scan_time       = 1.0 / self.scan_freq_hz
        msg.range_min       = self.range_min
        msg.range_max       = self.range_max

        msg.ranges      = list(self.ranges)
        msg.intensities = list(self.intensities)

        self.pub.publish(msg)

    # ── Cleanup ───────────────────────────────────────────────

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LDS02RRDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
