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

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def make_histogram(self, cluster):
        """ Count the number of pixels in each cluster """
        numLabels = np.arange(0, len(np.unique(cluster.labels_)) + 1)
        hist, _ = np.histogram(cluster.labels_, bins=numLabels)
        hist = hist.astype('float32')
        hist /= hist.sum()
        return hist

    def make_bar(self, color):
        """ Convert BGR color to HSV """
        bar = np.zeros((100, 100, 3), np.uint8)
        bar[:] = color
        hsv_bar = cv2.cvtColor(bar, cv2.COLOR_BGR2HSV)
        hue, sat, val = hsv_bar[0][0]
        return hue, sat, val

    def dominant_color_kmeans(self, roi, k=5):
        """ Extract dominant color using KMeans clustering """
        roi_resized = cv2.resize(roi, (50, 50), interpolation=cv2.INTER_AREA)
        roi_reshaped = roi_resized.reshape((-1, 3))  # Flatten ROI
        clusters = KMeans(n_clusters=k, random_state=42)
        clusters.fit(roi_reshaped)

        histogram = self.make_histogram(clusters)
        combined = zip(histogram, clusters.cluster_centers_)
        combined = sorted(combined, key=lambda x: x[0], reverse=True)

        # Return the dominant color in HSV
        dominant_bgr = combined[0][1]
        hue, sat, val = self.make_bar(dominant_bgr)
        return hue, sat, val

    def classify_color_from_hsv(self, h, s, v):
        """ Classify color based on HSV ranges """
        if s < 100 and v > 140:
            return "white", (255, 255, 255)
        elif v < 50:
            return "black", (0, 0, 0)
        elif 0 <= h < 10 or 160 <= h <= 180:
            return "red", (0, 0, 255)
        elif 10 <= h < 25:
            return "orange", (0, 165, 255)
        elif 25 <= h < 35:
            return "yellow", (0, 255, 255)
        elif 40 <= h < 70:
            return "green", (0, 255, 0)
        elif 80 <= h < 130:
            return "blue", (255, 0, 0)
        elif 130 <= h < 170:
            return "purple", (128, 0, 128)
        else:
            return "unknown", (128, 128, 128)
        
    def slicer_callback(self, image_slice):
        results = self.model(image_slice)[0]
        print(f"Results: {results}")
        return sv.Detections.from_ultralytics(results)

    def detect_from_image(self, image_msg):
        frame = self.bridge_object.compressed_imgmsg_to_cv2(image_msg)
        # Blur
        #frame = cv2.GaussianBlur(frame, (5, 5), 0)
        #frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, self.kernel)
        # Grayscale
        grayscale_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Histogram equalization
        #grayscale_frame = self.clahe.apply(grayscale_frame)
        grayscale_frame = cv2.cvtColor(grayscale_frame, cv2.COLOR_GRAY2BGR)
        slicer = sv.InferenceSlicer(callback=self.slicer_callback)
        results = self.model(frame)

        ball_bounding_box_list = Float32MultiArray() 
        robot_bounding_box = Float32MultiArray()

        # Loop through detections
        for detection in results[0].boxes:
            x1, y1, x2, y2 = map(int, detection.xyxy[0])  # Bounding box coordinates
            conf = detection.conf[0]  # Confidence score
            cls = detection.cls[0]  # Class index

            if conf < 0.5:  # Confidence threshold
                continue

            # Define class name and box color (adjust based on your class names)
            class_name = self.model.names[int(cls)]  # Replace model.names with class names if predefined
            color = (0, 255, 0)  # Green for bounding boxes, you can define colors for each class

            # Annotate the frame with bounding box and label
            label = f"{class_name} ({conf:.2f})"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)  # Draw bounding box
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)  # Draw label

            if (self.model.names[int(cls)] == "robot"):
                robot_bounding_box.data.extend([x1, y1, x2, y2])
            elif (self.model.names[int(cls)] == "orange ball"):
                ball_bounding_box_list.data.extend([x1, y1, x2, y2])

        # for i in range(len(results.xyxy)):
        #     x1, y1, x2, y2 = map(int, results.xyxy[i])  # Bounding box coordinates
        #     conf = results.confidence[i]  # Confidence score
        #     cls = results.class_id[i] # Class ID

        #     # Draw the bounding box and label
        #     if (conf < self.get_parameter("conf_threshold").value):
        #         continue

        #     if self.model.names[int(cls)] == ROBOT_CLS:
        #         label = f'{self.model.names[int(cls)]}: {float(conf):.2f}'
        #         box_color = (255, 0, 0)
        #         text_color = (255, 0, 0)
        #         robot_bounding_box.data.extend(x1, y1, x2, y2)

        #     else:
        #         # Extract ROI (Region of Interest) and determine dominant color
        #         margin_x = int((x2 - x1) * 0.1)  # 10% margin
        #         margin_y = int((y2 - y1) * 0.1)

        #         roi = frame[y1 + margin_y:y2 - margin_y, x1 + margin_x:x2 - margin_x]

        #         if roi.size > 0:
        #             h, s, v = self.dominant_color_kmeans(roi)
        #             detected_color, bgr_color = self.classify_color_from_hsv(h, s, v)
        #         else:
        #             detected_color, bgr_color = "unknown", (128, 128, 128)
                
        #         # Annotate the frame with bounding box and color label
        #         label = f"{detected_color} ({float(conf):.2f})"
        #         box_color = (0, 0, 0) if detected_color == "white" else bgr_color
        #         text_color = (0, 0, 0) if detected_color == "white" else bgr_color

        #         ball_bounding_box_list.data.extend([x1, y1, x2, y2])
        #     cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
        #     cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
        
        print(f"Bounding box list: {ball_bounding_box_list}")
        self.ball_publisher.publish(ball_bounding_box_list)
        self.robot_publisher.publish(robot_bounding_box)
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

