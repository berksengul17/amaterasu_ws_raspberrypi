import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from amaterasu.kalman import Kalman
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.kalman = Kalman()

        # Subscribers
        self.imu_subscription = self.create_subscription(
            Float32,
            '/imu/raw_data',
            self.imu_callback,
            10
        )
        self.mag_subscription = self.create_subscription(
            Float32,
            '/magnetometer/smoothed',
            self.mag_callback,
            10
        )

        # Publisher
        self.fused_publisher = self.create_publisher(Float32, '/imu/fused', 10)

        # State variables
        self.imu_data = None
        self.mag_heading = None

        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.fuse_data)

        self.tf_broadcaster = TransformBroadcaster(self)

    def imu_callback(self, msg: Float32):
        """Handle IMU data."""
        self.imu_data = msg.data # degree

    def mag_callback(self, msg: Float32):
        """Handle Magnetometer data."""
        self.mag_heading = msg.data  # degree

    def fuse_data(self):
        """Fuse IMU and Magnetometer data when both are available."""
        if self.imu_data is None or self.mag_heading is None:
            return

        # Time step
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.kalman.update_yaw_angle(self.imu_data, dt, self.mag_heading)

        # Debug output
        self.get_logger().info(
            f"Fused Yaw: {self.kalman.yaw:.2f}°"
        )
        
        pub_yaw = Float32()
        pub_yaw.data = self.kalman.yaw
        self.fused_publisher.publish(pub_yaw)
        self.publish_tf(self.kalman.yaw)

    def publish_tf(self, yaw):
        """Provide the magnetometer's transform as a TransformStamped."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"  # Match robot's frame
        t.child_frame_id = "yaw"

        quaternion = quaternion_from_euler(0, 0, math.radians(yaw))
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
