#!/usr/bin/env python3

"""RViz IMU visualizer node for the moteur project.

Subscribes to /imu/data by default and publishes MarkerArray on /imu_visualization.
This is intentionally host-side and launch-file independent.
"""

import math

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class ImuVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("imu_visualizer")

        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("marker_topic", "/imu_visualization")
        self.declare_parameter("frame_fallback", "imu_link")
        self.declare_parameter("accel_gain", 0.02)
        self.declare_parameter("gyro_gain", 0.12)
        self.declare_parameter("max_arrow_length", 0.25)

        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value

        marker_qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, marker_qos)
        self._imu_sub = self.create_subscription(Imu, imu_topic, self._imu_callback, qos_profile_sensor_data)

        self._got_first_msg = False
        self.get_logger().info(f"IMU visualizer started: {imu_topic} -> {marker_topic}")

    def _imu_callback(self, msg: Imu) -> None:
        frame_id = msg.header.frame_id.strip() or self._param_str("frame_fallback")
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        markers = MarkerArray()

        markers.markers.append(self._orientation_marker(frame_id, stamp, msg))
        markers.markers.append(
            self._vector_marker(
                frame_id=frame_id,
                stamp=stamp,
                marker_id=1,
                ns="accel",
                vector=(
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ),
                gain=self._param_float("accel_gain"),
                max_len=self._param_float("max_arrow_length"),
                color=ColorRGBA(r=1.0, g=0.3, b=0.2, a=0.9),
            )
        )
        markers.markers.append(
            self._vector_marker(
                frame_id=frame_id,
                stamp=stamp,
                marker_id=2,
                ns="gyro",
                vector=(
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ),
                gain=self._param_float("gyro_gain"),
                max_len=self._param_float("max_arrow_length"),
                color=ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.9),
            )
        )

        markers.markers.extend(self._axis_markers(frame_id, stamp))
        markers.markers.append(self._text_marker(frame_id, stamp, msg))

        self._marker_pub.publish(markers)

        if not self._got_first_msg:
            self._got_first_msg = True
            self.get_logger().info("First /imu message received. Publishing markers.")

    def _orientation_marker(self, frame_id: str, stamp, msg: Imu) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "orientation"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation = msg.orientation
        marker.scale = Vector3(x=0.04, y=0.025, z=0.015)
        marker.color = ColorRGBA(r=0.1, g=0.95, b=0.35, a=0.75)
        marker.frame_locked = True
        return marker

    def _vector_marker(
        self,
        frame_id: str,
        stamp,
        marker_id: int,
        ns: str,
        vector: tuple,
        gain: float,
        max_len: float,
        color: ColorRGBA,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.frame_locked = True

        x, y, z = vector
        mag = math.sqrt((x * x) + (y * y) + (z * z))
        if mag < 1e-6:
            marker.action = Marker.DELETE
            return marker

        end_x = x * gain
        end_y = y * gain
        end_z = z * gain
        end_mag = math.sqrt((end_x * end_x) + (end_y * end_y) + (end_z * end_z))
        if end_mag > max_len:
            k = max_len / end_mag
            end_x *= k
            end_y *= k
            end_z *= k

        marker.action = Marker.ADD
        marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=end_x, y=end_y, z=end_z)]
        marker.scale = Vector3(x=0.005, y=0.01, z=0.0)
        marker.color = color
        return marker

    def _axis_markers(self, frame_id: str, stamp) -> list:
        axis_data = [
            (3, "axis_x", Point(x=0.06, y=0.0, z=0.0), ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)),
            (4, "axis_y", Point(x=0.0, y=0.06, z=0.0), ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)),
            (5, "axis_z", Point(x=0.0, y=0.0, z=0.06), ColorRGBA(r=0.0, g=0.3, b=1.0, a=0.5)),
        ]

        markers = []
        for marker_id, ns, end, color in axis_data:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = ns
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.frame_locked = True
            marker.points = [Point(x=0.0, y=0.0, z=0.0), end]
            marker.scale = Vector3(x=0.0025, y=0.005, z=0.0)
            marker.color = color
            markers.append(marker)
        return markers

    def _text_marker(self, frame_id: str, stamp, msg: Imu) -> Marker:
        accel_norm = math.sqrt(
            (msg.linear_acceleration.x * msg.linear_acceleration.x)
            + (msg.linear_acceleration.y * msg.linear_acceleration.y)
            + (msg.linear_acceleration.z * msg.linear_acceleration.z)
        )
        gyro_norm = math.sqrt(
            (msg.angular_velocity.x * msg.angular_velocity.x)
            + (msg.angular_velocity.y * msg.angular_velocity.y)
            + (msg.angular_velocity.z * msg.angular_velocity.z)
        )

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "norm_text"
        marker.id = 6
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.09
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.025
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        marker.frame_locked = True
        marker.text = f"|a|={accel_norm:.2f} m/s^2 |w|={gyro_norm:.2f} rad/s"
        return marker

    def _param_str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _param_float(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
