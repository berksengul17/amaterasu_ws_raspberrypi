from ultralytics import YOLO
import numpy as np
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from sklearn.cluster import KMeans
import supervision as sv

CONF_THRESHOLD = 0.7
ROBOT_CLS = "Robot"

class ObjectDetectorPublisher(Node):
    def __init__(self):
        super().__init__('object_detector_publisher')
        self.model = YOLO("/home/berk/amaterasu_ws/src/amaterasu/amaterasu/yolo11nbest.pt")
        self.bridge_object = CvBridge()

        self.declare_parameter("conf_threshold", CONF_THRESHOLD)
        self.subscription = self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.detect_from_image, 10)
        self.publisher = self.create_publisher(CompressedImage, "/camera/detected_image/compressed", 10)
        self.ball_publisher = self.create_publisher(Float32MultiArray, "/ball/bounding_box", 10)
        self.robot_publisher = self.create_publisher(Float32MultiArray, "/robot/bounding_box", 10)

        # Added a publisher for robot's initial position
        self.robot_position_publisher = self.create_publisher(Float32MultiArray, "/robot/initial_position", 10)

        # Internal state
        self.robot_position_initialized = False  # Track if robot position is set
        self.robot_position = Float32MultiArray()  # To store initial robot position

    def detect_from_image(self, image_msg):
        frame = self.bridge_object.compressed_imgmsg_to_cv2(image_msg)
        results = self.model(frame)

        ball_bounding_box_list = Float32MultiArray() 
        robot_bounding_box = Float32MultiArray()

        # Process each detection
        for detection in results[0].boxes:
            x1, y1, x2, y2 = map(int, detection.xyxy[0])  # Bounding box coordinates
            conf = detection.conf[0]  # Confidence score
            cls = detection.cls[0]  # Class index

            # Filter detections by confidence threshold
            if conf < self.get_parameter("conf_threshold").value:
                continue

            # Get the class name
            class_name = self.model.names[int(cls)]

            if class_name == "orange ball":
                # Add to ball bounding boxes
                ball_bounding_box_list.data.extend([x1, y1, x2, y2])

                # Annotate the frame
                label = f"{class_name} ({conf:.2f})"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)  # Orange bounding box
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

            elif class_name == "robot" and not self.robot_position_initialized:
                # Set robot's initial position only if not already initialized
                self.robot_position.data = [x1, y1, x2, y2]
                self.robot_position_initialized = True

                # Publish the robot's initial position
                self.robot_position_publisher.publish(self.robot_position)

                # Annotate the frame
                label = f"{class_name} ({conf:.2f})"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue bounding box
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        print(f"Bounding box list: {ball_bounding_box_list}")
        self.ball_publisher.publish(ball_bounding_box_list)
        self.publisher.publish(self.bridge_object.cv2_to_compressed_imgmsg(frame))
        # return bounding_box_list


def main(args=None):
    rclpy.init(args=args)

    object_detector = ObjectDetectorPublisher()
    rclpy.spin(object_detector)

    object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

