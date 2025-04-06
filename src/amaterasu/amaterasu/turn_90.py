import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import numpy as np
import math

class Turn90(Node):
    def __init__(self):
        super().__init__('turn_90_node')

        self.current_yaw = 0.0
        self.start_yaw = None
        self.target_reached = False

        self.kp = 0.02
        self.ki = 0.0005
        self.kd = 0.001
        self.max_integral = 100

        self.error_sum = 0.0
        self.prev_error = 0.0

        self.sample_time = 0.05  # 10ms
        self.declare_parameter("target_angle_deg", 90.0)

        self.target_angle_deg = self.get_parameter("target_angle_deg").get_parameter_value().double_value

        self.odom_sub = self.create_subscription(Odometry, "/ekf_odom", self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(self.sample_time, self.update)

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw_rad = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.current_yaw = yaw_rad * (180.0 / np.pi)

    def normalize_angle(self, angle):
        return (angle + 180.0) % 360.0 - 180.0

    def update(self):
        if self.target_reached:
            return

        if self.start_yaw is None:
            self.start_yaw = self.current_yaw
            self.goal_yaw = self.normalize_angle(self.start_yaw + self.target_angle_deg)
            self.get_logger().info(f"Turning from {self.start_yaw:.2f}° to {self.goal_yaw:.2f}°")

        error = self.normalize_angle(self.current_yaw - self.goal_yaw)

        # PID terms
        self.error_sum += error * self.sample_time
        self.error_sum = max(min(self.error_sum, self.max_integral), -self.max_integral)
        d_error = (error - self.prev_error) / self.sample_time
        self.prev_error = error

        angular_z = (
            self.kp * error +
            self.ki * self.error_sum +
            self.kd * d_error
        )

        # Clamp angular speed
        angular_z = max(min(angular_z, 1.5), -1.5)

        if abs(error) > 5.0:
            twist = Twist()
            twist.angular.z = -angular_z
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"Yaw error: {error:.2f}°, turning...")
        else:
            self.cmd_vel_pub.publish(Twist())  # Stop
            self.get_logger().info("Turn complete!")
            self.target_reached = True


def main(args=None):
    rclpy.init(args=args)
    node = Turn90()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
