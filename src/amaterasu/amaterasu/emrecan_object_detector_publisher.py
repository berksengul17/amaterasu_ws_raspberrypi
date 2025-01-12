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

    def overlay_grid(self, frame, grid_size, platform_dimensions):
        img_h, img_w = frame.shape[:2]
        grid_rows, grid_cols = grid_size
        platform_width, platform_height = platform_dimensions  # in meters

        # Cell dimensions in pixels and meters
        cell_w_px = img_w // grid_cols
        cell_h_px = img_h // grid_rows
        cell_w_m = platform_width / grid_cols
        cell_h_m = platform_height / grid_rows

        # Draw grid
        for col in range(1, grid_cols):
            x = col * cell_w_px
            cv2.line(frame, (x, 0), (x, img_h), (255, 255, 255), 1)  # Vertical lines

        for row in range(1, grid_rows):
            y = row * cell_h_px
            cv2.line(frame, (0, y), (img_w, y), (255, 255, 255), 1)  # Horizontal lines

        return frame, cell_w_m, cell_h_m

    def map_to_real_world(self, x, y, cell_w_m, cell_h_m, platform_origin=(0, 0)):
        real_x = x * cell_w_m + platform_origin[0]
        real_y = y * cell_h_m + platform_origin[1]
        return real_x, real_y


    def detect_from_image(self, image_msg):
        frame = self.bridge_object.compressed_imgmsg_to_cv2(image_msg)

        # Define platform dimensions and grid
        platform_dimensions = (10.0, 10.0)  # Real-world size in meters
        grid_size = (10, 10)  # 10x10 grid

        # Overlay grid and get cell dimensions
        frame, cell_w_m, cell_h_m = self.overlay_grid(frame, grid_size, platform_dimensions)

        # Detect objects
        results = self.model(frame)

        ball_positions = []

        for detection in results[0].boxes:
            x1, y1, x2, y2 = map(int, detection.xyxy[0])
            conf = detection.conf[0]
            cls = detection.cls[0]

            if conf < self.get_parameter("conf_threshold").value:
                continue

            class_name = self.model.names[int(cls)]
            if class_name == "orange ball":
                # Calculate the ball's real-world position
                ball_x_px = (x1 + x2) // 2
                ball_y_px = (y1 + y2) // 2
                real_x, real_y = self.map_to_real_world(ball_x_px, ball_y_px, cell_w_m, cell_h_m)

                ball_positions.append((real_x, real_y))

                # Annotate the grid and frame
                label = f"Ball ({real_x:.2f}, {real_y:.2f})"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        # Publish ball positions in real-world coordinates
        ball_positions_msg = Float32MultiArray(data=[coord for pos in ball_positions for coord in pos])
        self.ball_publisher.publish(ball_positions_msg)

        # Publish annotated image
        self.publisher.publish(self.bridge_object.cv2_to_compressed_imgmsg(frame))



def main(args=None):
    rclpy.init(args=args)

    object_detector = ObjectDetectorPublisher()
    rclpy.spin(object_detector)

    object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

