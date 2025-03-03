import cv2
import numpy as np
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import Vector3

CONF_THRESHOLD = 0.5

class ObjectDetectorPublisher(Node):
    def __init__(self):
        super().__init__('object_detector_publisher')
        self.model = YOLO("/home/amaterasu/amaterasu_ws/src/amaterasu/amaterasu/çok_iyi_duran_best.pt")
        self.bridge_object = CvBridge()

        # Parameters for confidence and grid dimensions.
        self.declare_parameter("conf_threshold", CONF_THRESHOLD)
        self.declare_parameter("grid_rows", 10)
        self.declare_parameter("grid_cols", 10)

        # Subscriptions and publishers.
        self.subscription = self.create_subscription(
            CompressedImage, 
            "/camera/image_raw/compressed", 
            self.detect_from_image, 
            10
        )
        self.publisher = self.create_publisher(
            CompressedImage, 
            "/camera/detected_image/compressed", 
            10
        )
        self.ball_publisher = self.create_publisher(
            Float32MultiArray, 
            "/ball/bounding_box", 
            10
        )
        self.robot_publisher = self.create_publisher(
            Vector3, 
            "/robot/bounding_box", 
            10
        )
        self.grid_publisher = self.create_publisher(
            Int32MultiArray, 
            "/grid/fire_count", 
            10
        )

        # Camera calibration matrices.
        self.camera_matrix = np.array([
            [1381.31470, 0.0, 648.521209], 
            [0.0, 1406.74184, 462.773793], 
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([0.145705008, -0.191785570, -0.0342241896, 0.00089729895, 1.11870175])

    def undistort_frame(self, frame):
        """Remove camera distortion from the frame."""
        h, w = frame.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)
        x, y, w, h = roi
        return undistorted_frame[y:y+h, x:x+w]

    def map_to_real_world(self, x_px, y_px, img_w, img_h, platform_dimensions):
        """Map pixel coordinates to real-world coordinates."""
        platform_width, platform_height = platform_dimensions
        real_x = (x_px / img_w) * platform_width
        real_y = (y_px / img_h) * platform_height
        return real_x, real_y

    def detect_from_image(self, image_msg):
        # Convert ROS image message to OpenCV image.
        frame = self.bridge_object.compressed_imgmsg_to_cv2(image_msg)
        # Undistort (and optionally crop) the frame.
        frame = self.undistort_frame(frame)
        # Run YOLO detection.
        results = self.model(frame)

        # Get image dimensions and define real-world platform dimensions (meters).
        img_h, img_w = frame.shape[:2]
        platform_dimensions = (2.05, 1.48)

        # Retrieve grid parameters.
        grid_rows = self.get_parameter("grid_rows").value
        grid_cols = self.get_parameter("grid_cols").value
        # Initialize a grid count matrix.
        grid_counts = [[0 for _ in range(grid_cols)] for _ in range(grid_rows)]

        ball_positions = []
        robot_position = (0.0, 0.0)

        # Process each detected object.
        for detection in results[0].boxes:
            x1, y1, x2, y2 = map(int, detection.xyxy[0])
            conf = detection.conf[0]
            cls = detection.cls[0]

            if conf < self.get_parameter("conf_threshold").value:
                continue

            class_name = self.model.names[int(cls)]
            center_x_px = (x1 + x2) // 2
            center_y_px = (y1 + y2) // 2

            # Map the center pixel to real-world coordinates.
            real_x, real_y = self.map_to_real_world(center_x_px, center_y_px, img_w, img_h, platform_dimensions)

            if class_name == "orange ball":
                ball_positions.append((real_x, real_y, conf))
                label = f"Ball ({real_x:.2f}, {real_y:.2f}, Conf: {conf:.2f})"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                # Determine grid cell based on real-world coordinates.
                cell_width = platform_dimensions[0] / grid_cols
                cell_height = platform_dimensions[1] / grid_rows
                grid_col = min(grid_cols - 1, int(real_x / cell_width))
                grid_row = min(grid_rows - 1, int(real_y / cell_height))
                grid_counts[grid_row][grid_col] += 1
            elif class_name == "robot":
                robot_position = (real_x, real_y)
                label = f"Robot ({real_x:.2f}, {real_y:.2f}, Conf: {conf:.2f})"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # Print and publish the grid counts.
        print("Detected Ball Positions (Real World, Confidence):", ball_positions)
        print("Detected Robot Position (Real World):", robot_position)
        print("Grid Counts (fire count per cell):", grid_counts)

        # Publish ball positions (flattened real-world coordinates).
        ball_positions_msg = Float32MultiArray(
            data=[coord for pos in ball_positions for coord in pos[:2]]
        )
        self.ball_publisher.publish(ball_positions_msg)
        # Publish robot position.
        self.robot_publisher.publish(Vector3(x=robot_position[0], y=robot_position[1], z=0.0))
        # Flatten the grid counts in row-major order and publish.
        flattened_grid = [count for row in grid_counts for count in row]
        grid_msg = Int32MultiArray(data=flattened_grid)
        self.grid_publisher.publish(grid_msg)

        # (Optional) Draw grid lines on the image for visualization.
        for i in range(1, grid_rows):
            y = int((i / grid_rows) * img_h)
            cv2.line(frame, (0, y), (img_w, y), (255, 255, 255), 1)
        for j in range(1, grid_cols):
            x = int((j / grid_cols) * img_w)
            cv2.line(frame, (x, 0), (x, img_h), (255, 255, 255), 1)

        # Publish the annotated image.
        self.publisher.publish(self.bridge_object.cv2_to_compressed_imgmsg(frame))

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetectorPublisher()
    rclpy.spin(object_detector)
    object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
