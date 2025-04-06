import serial
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Start serial comm with arduino
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
        time.sleep(20)
        self.get_logger().info('Countdown ended.')
        self.ser.reset_input_buffer()

        self.last_yaw = 0.0

        self.imu_publisher = self.create_publisher(Imu, '/imu/z', 10)

        # 10ms
        self.timer = self.create_timer(0.01, self.publish_imu_data)

    def publish_imu_data(self):
        yaw = self.last_yaw
        if (self.ser.in_waiting > 0):
            yaw = float(self.ser.readline().decode('utf-8').rstrip()) * (np.pi / 180.0) # in rad/s
            self.last_yaw = yaw

        # Create Imu message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        q = quaternion_from_euler(0, 0, yaw)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        # Publish message
        self.imu_publisher.publish(imu_msg)
        self.get_logger().info(f'Published yaw: {yaw:.6f} rad/s')

def main(args=None):
    rclpy.init(args=args)

    imu_publisher = ImuPublisher()
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        imu_publisher.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
