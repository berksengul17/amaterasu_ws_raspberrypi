import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class CameraPublisher(Node):
    
    def __init__(self):
        super().__init__("camera_publisher")
        self.declare_parameter("period", 0.02)
        self.publisher = self.create_publisher(CompressedImage, "/camera/image_raw/compressed", 10)
        self.camera_device_number = 2
        self.camera = cv2.VideoCapture(self.camera_device_number)
        self.bridge_object = CvBridge()

        self.timer = self.create_timer(self.get_parameter("period").get_parameter_value().double_value, self.timer_callback_function)

        self.i = 0

    def timer_callback_function(self):
        success, frame = self.camera.read()

        if success:
            ros2_image_message = self.bridge_object.cv2_to_compressed_imgmsg(frame)
            self.publisher.publish(ros2_image_message)

        self.get_logger().info(f'Publishing image number {self.i}')

        self.i += 1
        


def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraPublisher()
    rclpy.spin(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

