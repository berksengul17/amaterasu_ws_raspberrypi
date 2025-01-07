import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class ObjectDetectorSubscriber(Node):
    def __init__(self):
        super().__init__("object_detector_subscriber")
        self.bridge_object = CvBridge()

        self.subscription = self.create_subscription(CompressedImage, "/camera/detected_image/compressed", self.show_image, 10)


    def show_image(self, image_msg):
        self.get_logger().info("The processed image is received")
        frame = self.bridge_object.compressed_imgmsg_to_cv2(image_msg)

        cv2.imshow("Detected Image", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    object_detector_subscriber = ObjectDetectorSubscriber()
    rclpy.spin(object_detector_subscriber)

    object_detector_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()