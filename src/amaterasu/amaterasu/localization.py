import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from tf_transformations import (
    quaternion_matrix,
    euler_from_quaternion,
    translation_matrix,
    quaternion_from_matrix
)
import numpy as np

class AprilTagLocalizationNode(Node):
    def __init__(self):
        super().__init__("apriltag_localization")

        self.odom_pub = self.create_publisher(Odometry, "/localization", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.T_A_to_robot_start = None  # Initial camera-to-robot transform (position only)

        self.timer = self.create_timer(0.01, self.lookup_transform)

    def lookup_transform(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "camera",  # target_frame (camera)
                "tag36h11_0",  # source_frame (robot's AprilTag)
                rclpy.time.Time())

            # Current transform (T_A_to_robot_new)
            trans = tf.transform.translation
            quat = tf.transform.rotation

            T_A_to_robot_new = np.eye(4)
            T_A_to_robot_new[:3, :3] = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])[:3, :3]
            T_A_to_robot_new[:3, 3] = [trans.x, trans.y, trans.z]

            if self.T_A_to_robot_start is None:
                # Save initial pose as T_A_to_robot_start
                self.T_A_to_robot_start = T_A_to_robot_new
                self.get_logger().info('Initial pose saved.')
                return

            # Calculate T_B_to_robot_new
            T_B_to_robot_new = np.linalg.inv(self.T_A_to_robot_start) @ T_A_to_robot_new

            # Extract translation and rotation
            trans_rel = T_B_to_robot_new[:3, 3]
            rot_rel = quaternion_from_matrix(T_B_to_robot_new)

            _, _, yaw = euler_from_quaternion(rot_rel)
            yaw_deg = np.degrees(yaw)

            robot_length = 0.15 # meters
            robot_width = 0.13

            trans_rel[0] = trans_rel[0] + (np.cos(yaw) * robot_length / 2) + (np.sin(yaw) * robot_width / 2)
            trans_rel[1] = trans_rel[1] + (np.sin(yaw) * robot_length / 2) + (np.cos(yaw) * robot_width / 2)
            
            self.get_logger().info(f"x: {trans_rel[0]}, y: {trans_rel[1]}, z: {trans_rel[2]}, yaw: {yaw_deg}")

            # Publish Odometry in frame B
            relative_odom = Odometry()
            relative_odom.header.stamp = self.get_clock().now().to_msg()
            relative_odom.header.frame_id = 'tag36h11_0'

            relative_odom.pose.pose.position.x = trans_rel[0]
            relative_odom.pose.pose.position.y = trans_rel[1]
            relative_odom.pose.pose.position.z = trans_rel[2]

            relative_odom.pose.pose.orientation.x = rot_rel[0]
            relative_odom.pose.pose.orientation.y = rot_rel[1]
            relative_odom.pose.pose.orientation.z = rot_rel[2]
            relative_odom.pose.pose.orientation.w = rot_rel[3]

            self.odom_pub.publish(relative_odom)

        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
