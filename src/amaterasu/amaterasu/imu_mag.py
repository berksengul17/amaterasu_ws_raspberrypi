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

        # Kalman filter state
        self.kalman_state = {'angle': 0, 'uncertainty': 2 * 2}  # Example initial state
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.try_fuse_data)

    def imu_callback(self, msg: Float32):
        """Handle IMU data."""
        self.imu_data = msg.data

    def mag_callback(self, msg: Float32):
        """Handle Magnetometer data."""
        self.mag_heading = msg.data  # Use smoothed heading

    def try_fuse_data(self):
        """Fuse IMU and Magnetometer data when both are available."""
        if self.imu_data is None or self.mag_heading is None:
            return

        # Time step
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.kalman.update_yaw_angle(self.imu_data, dt, self.mag_heading)

        # # Fuse yaw using Kalman filter
        # self.kalman_state = self.kalman_filter(
        #     self.kalman_state,
        #     self.imu_data,  # Gyroscope yaw rate
        #     self.mag_heading,  # Magnetometer absolute yaw
        #     dt
        # )


        # Debug output
        self.get_logger().info(
            f"Fused Yaw: {self.kalman.yaw:.2f}°"
        )


    def kalman_filter(self, state, gyro_rate, mag_angle, dt):
        """Apply Kalman filter for yaw estimation."""
        # Predict step: update angle using gyroscope
        state['angle'] += gyro_rate * dt  # Integrate angular velocity
        state['uncertainty'] += 0.001  # Add process noise (tune as needed)

        # Normalize predicted angle to [-π, π]
        state['angle'] = state['angle']

        # Update step: incorporate magnetometer measurement
        kalman_gain = state['uncertainty'] / (state['uncertainty'] + 0.03)  # Measurement noise (tune as needed)
        angle_difference = mag_angle - state['angle']
        state['angle'] += kalman_gain * angle_difference
        state['uncertainty'] *= (1 - kalman_gain)

        return state
    
    # def normalize_angle(self, angle):
    #     """Normalize angle to [-π, π]."""
    #     while angle > math.pi:
    #         angle -= 2 * math.pi
    #     while angle < -math.pi:
    #         angle += 2 * math.pi
    #     return angle

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()
    rclpy.spin(fusion_node)
    fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
