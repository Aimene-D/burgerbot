import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry

class ImuTfPublisher(Node):
    def __init__(self):
        super().__init__('imu_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def imu_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = ImuTfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
