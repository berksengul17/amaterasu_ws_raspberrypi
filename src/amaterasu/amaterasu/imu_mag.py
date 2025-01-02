import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from amaterasu.kalman import Kalman

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

    def imu_callback(self, msg: Float32):
        """Handle IMU data."""
        self.imu_data = msg.data

    def mag_callback(self, msg: Float32):
        """Handle Magnetometer data."""
        self.mag_heading = msg.data  # Use smoothed heading

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

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
