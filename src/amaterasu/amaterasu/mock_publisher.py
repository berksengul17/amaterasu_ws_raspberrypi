import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import random

class MockPublisher(Node):
    def __init__(self):
        super().__init__('mock_publisher')
        self.imu_pub = self.create_publisher(Imu, '/imu/mock_data', 10)
        self.timer = self.create_timer(0.1, self.publish_mock_data)

    def publish_mock_data(self):
        imu_msg = Imu()
        imu_msg.orientation.x = random.uniform(-1, 1)
        imu_msg.orientation.y = random.uniform(-1, 1)
        imu_msg.orientation.z = random.uniform(-1, 1)
        imu_msg.orientation.w = random.uniform(-1, 1)
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
