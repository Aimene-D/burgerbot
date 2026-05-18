#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math
import struct

PACKET_SIZE    = 42
START_BYTE     = 0xFA
INDEX_MIN      = 0xA0
INDEX_MAX      = 0xF9
TOTAL_PACKETS  = 60
TOTAL_SAMPLES  = 360

class LDS02RRDriver(Node):
    def __init__(self):
        super().__init__('lds02rr_driver')
        self.declare_parameter('port',         '/dev/ttyACM0')
        self.declare_parameter('baud',         115200)
        self.declare_parameter('frame_id',     'lidar')
        self.declare_parameter('range_min',    0.12)
        self.declare_parameter('range_max',    3.5)
        self.declare_parameter('angle_offset', 0.0)

        port             = self.get_parameter('port').value
        baud             = self.get_parameter('baud').value
        self.frame_id    = self.get_parameter('frame_id').value
        self.range_min   = self.get_parameter('range_min').value
        self.range_max   = self.get_parameter('range_max').value
        self.angle_offset= self.get_parameter('angle_offset').value

        self.pub         = self.create_publisher(LaserScan, '/scan', 10)
        self.distances   = [float('inf')] * TOTAL_SAMPLES
        self.intensities = [0.0]          * TOTAL_SAMPLES
        self.buf         = bytearray()

        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'Port {port} ouvert @ {baud} baud')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Impossible ouvrir {port}: {e}')
            raise SystemExit(1)

        self.create_timer(0.001, self._read_cb)

    def _read_cb(self):
        try:
            nb = self.ser.in_waiting
            if nb:
                self.buf.extend(self.ser.read(nb))
            self._process_buf()
        except serial.SerialException as e:
            self.get_logger().error(f'Erreur série: {e}')

    def _process_buf(self):
        while len(self.buf) >= PACKET_SIZE:
            if self.buf[0] != START_BYTE:
                self.buf.pop(0)
                continue
            idx = self.buf[1]
            if idx < INDEX_MIN or idx > INDEX_MAX:
                self.buf.pop(0)
                continue
            pkt = bytes(self.buf[:PACKET_SIZE])
            del self.buf[:PACKET_SIZE]
            cs       = sum(pkt[:40]) & 0xFFFF
            cs_recv  = struct.unpack_from('<H', pkt, 40)[0]
            if cs != cs_recv:
                continue
            self._parse_packet(pkt)

    def _parse_packet(self, pkt):
        idx = pkt[1] - INDEX_MIN
        for i in range(6):
            angle_idx      = idx * 6 + i
            dist_mm, intensity = struct.unpack_from('<HH', pkt, 4 + i * 4)
            dist_m         = dist_mm / 1000.0
            if dist_mm == 0 or dist_m < self.range_min or dist_m > self.range_max:
                dist_m     = float('inf')
            self.distances[angle_idx]   = dist_m
            self.intensities[angle_idx] = float(intensity)
        if idx == TOTAL_PACKETS - 1:
            self._publish_scan()

    def _publish_scan(self):
        msg                 = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min       = self.angle_offset
        msg.angle_max       = self.angle_offset + 2 * math.pi
        msg.angle_increment = (2 * math.pi) / TOTAL_SAMPLES
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / 5.0
        msg.range_min       = self.range_min
        msg.range_max       = self.range_max
        msg.ranges          = self.distances[:]
        msg.intensities     = self.intensities[:]
        self.pub.publish(msg)

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
