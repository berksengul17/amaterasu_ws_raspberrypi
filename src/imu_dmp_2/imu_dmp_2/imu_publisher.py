import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, Float32
from imu_dmp_2.MPU6050 import MPU6050  # Assuming MPU6050 library is available


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Declare and get parameters
        self.declare_parameter('topic_name', 'imu_data')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', 0x68)
        self.declare_parameter('x_accel_offset', 0.0)
        self.declare_parameter('y_accel_offset', 0.0)
        self.declare_parameter('z_accel_offset', 0.0)
        self.declare_parameter('x_gyro_offset', 0.0)
        self.declare_parameter('y_gyro_offset', 0.0)
        self.declare_parameter('z_gyro_offset', 0.0)

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        device_address = self.get_parameter('device_address').get_parameter_value().integer_value
        x_accel_offset = self.get_parameter('x_accel_offset').get_parameter_value().integer_value
        y_accel_offset = self.get_parameter('y_accel_offset').get_parameter_value().integer_value
        z_accel_offset = self.get_parameter('z_accel_offset').get_parameter_value().integer_value
        x_gyro_offset = self.get_parameter('x_gyro_offset').get_parameter_value().integer_value
        y_gyro_offset = self.get_parameter('y_gyro_offset').get_parameter_value().integer_value
        z_gyro_offset = self.get_parameter('z_gyro_offset').get_parameter_value().integer_value

        # Create publisher
        self.publisher_ = self.create_publisher(Float32, topic_name, 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish at 10 Hz

        # Initialize MPU6050
        self.mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
                           z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset, False)
        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()

    def publish_imu_data(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        try:
            FIFO_count = self.mpu.get_FIFO_count()
            mpu_int_status = self.mpu.get_int_status()

            if (FIFO_count == 1024) or (mpu_int_status & 0x10):
                self.mpu.reset_FIFO()
            elif (mpu_int_status & 0x02):
                while FIFO_count < self.packet_size:
                    FIFO_count = self.mpu.get_FIFO_count()

                FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
                quat = self.mpu.DMP_get_quaternion(FIFO_buffer)
                grav = self.mpu.DMP_get_gravity(quat)
                gyro = self.mpu.get_rotation()
                accel = self.mpu.DMP_get_linear_accel(
                    self.mpu.DMP_get_acceleration_int16(FIFO_buffer), grav)

                msg.orientation.x = quat.x
                msg.orientation.y = quat.y
                msg.orientation.z = quat.z
                msg.orientation.w = quat.w

                msg.angular_velocity.x = float(gyro[0])
                msg.angular_velocity.y = float(gyro[1])
                msg.angular_velocity.z = float(gyro[2])

                msg.linear_acceleration.x = float(accel.x)
                msg.linear_acceleration.y = float(accel.y)
                msg.linear_acceleration.z = float(accel.z)

                # Set covariance matrices
                msg.orientation_covariance = [float(1e-6), 0.0, 0.0, 0.0, float(1e-6), 0.0, 0.0, 0.0, float(1e-6)]
                msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
                msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

                rpy = self.mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
                self.get_logger().info(f"R: {rpy.x} P: {rpy.y} Y: {rpy.z}")

                yaw = Float32()
                yaw.data = rpy.z

                self.publisher_.publish(yaw)
        except Exception as e:
            self.get_logger().error(f"Error in publishing IMU data: {e}")


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()

    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
