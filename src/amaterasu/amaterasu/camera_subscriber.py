import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraSubscriber(Node):
    
    def __init__(self):
        super().__init__("camera_subscriber")
        self.subscritption = self.create_subscription(Image, "/camera/image_raw", self.listener_callback_function, 10)
        self.bridge_object = CvBridge()

    def listener_callback_function(self, image_msg):
        self.get_logger().info('The image frame is received')

        open_cv_image = self.bridge_object.imgmsg_to_cv2(image_msg)

        cv2.imshow("Camera", open_cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraSubscriber()
    rclpy.spin(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

