import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050

class Imu(Node):
    def __init__(self):
        super().__init__('imu')

        self.mpu = mpu6050(0x68)

        self.gyro_z_bias = 0.0

        self.imu_publisher = self.create_publisher(Imu, '/imu/z', 10)

        self.calibrate_mpu6050()

        # 10ms
        self.timer = self.create_timer(0.01, self.publish_imu_data)

    def calibrate_mpu6050(self, num_samples=2000):
        self.get_logger().info('Calibrating MPU6050... Please keep the sensor still.')

        gyro_z_values = []

        for _ in range(num_samples):
            gyro_data = self.mpu.get_gyro_data()
            gyro_z_values.append(gyro_data['z'])
            time.sleep(0.01)

        self.gyro_z_bias = np.mean(gyro_z_values)
        self.get_logger().info(f'Calibration complete. Gyro Z bias: {self.gyro_z_bias:.6f} deg/s')

    def publish_imu_data(self):
        gyro_data = self.mpu.get_gyro_data()
        gyro_z_corrected = gyro_data['z'] - self.gyro_z_bias  # Apply bias correction

        # Create Imu message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Gyroscope data (only Z-axis is used)
        imu_msg.angular_velocity.z = gyro_z_corrected

        # Publish message
        self.imu_publisher.publish(imu_msg)
        self.get_logger().info(f'Published Gyro Z: {gyro_z_corrected:.6f} deg/s')

def main(args=None):
    rclpy.init(args=args)

    imu_publisher = Imu()
    rclpy.spin(imu_publisher)

    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
