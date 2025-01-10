import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32

class OrientationFilterNode(Node):
    def __init__(self):
        super().__init__('orientation_filter_node')

        # ROS2 Subscriptions
        self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)

        # ROS2 Publishers
        self.quaternion_publisher = self.create_publisher(Quaternion, '/orientation', 10)
        self.yaw_publisher = self.create_publisher(Float32, '/yaw_angle', 10)

        # Constants
        self.deltat = 0.001  # Sampling period in seconds (1 ms)
        gyro_meas_error = np.pi * (5.0 / 180.0)  # Gyroscope measurement error in rad/s (5 deg/s)
        self.beta = np.sqrt(3.0 / 4.0) * gyro_meas_error

        # Global state variables
        self.SEq = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion [w, x, y, z]

    def imu_callback(self, msg):
        # Extract accelerometer and gyroscope data
        a_x, a_y, a_z = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        w_x, w_y, w_z = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        # Update orientation using the filter
        self.filter_update(w_x, w_y, w_z, a_x, a_y, a_z)

        # Publish the updated quaternion
        quaternion_msg = Quaternion()
        quaternion_msg.w = self.SEq[0]
        quaternion_msg.x = self.SEq[1]
        quaternion_msg.y = self.SEq[2]
        quaternion_msg.z = self.SEq[3]
        self.quaternion_publisher.publish(quaternion_msg)

        # Calculate and publish the yaw angle
        yaw_angle = self.calculate_yaw()
        yaw_msg = Float32()
        yaw_msg.data = yaw_angle
        self.yaw_publisher.publish(yaw_msg)

    def filter_update(self, w_x, w_y, w_z, a_x, a_y, a_z):
        # Normalize accelerometer measurement
        norm = np.sqrt(a_x**2 + a_y**2 + a_z**2)
        if norm == 0:
            self.get_logger().warning('Accelerometer norm is zero; skipping update.')
            return

        a_x, a_y, a_z = a_x / norm, a_y / norm, a_z / norm

        # Auxiliary variables
        halfSEq = 0.5 * self.SEq
        twoSEq = 2.0 * self.SEq

        # Compute the objective function and Jacobian
        f = np.array([
            twoSEq[1] * self.SEq[3] - twoSEq[0] * self.SEq[2] - a_x,
            twoSEq[0] * self.SEq[1] + twoSEq[2] * self.SEq[3] - a_y,
            1.0 - twoSEq[1]**2 - twoSEq[2]**2 - a_z
        ])

        J = np.array([
            [0, twoSEq[2], twoSEq[3], -twoSEq[1]],
            [-twoSEq[3], twoSEq[1], twoSEq[0], twoSEq[2]],
            [twoSEq[1], twoSEq[2], 0, -twoSEq[3]]
        ])

        # Compute the gradient
        SEqHatDot = J.T @ f

        # Normalize the gradient
        norm = np.linalg.norm(SEqHatDot)
        if norm != 0:
            SEqHatDot /= norm

        # Compute the quaternion derivative measured by gyroscopes
        SEqDot_omega = 0.5 * np.array([
            -self.SEq[1] * w_x - self.SEq[2] * w_y - self.SEq[3] * w_z,
            self.SEq[0] * w_x + self.SEq[2] * w_z - self.SEq[3] * w_y,
            self.SEq[0] * w_y - self.SEq[1] * w_z + self.SEq[3] * w_x,
            self.SEq[0] * w_z + self.SEq[1] * w_y - self.SEq[2] * w_x
        ])

        # Integrate the estimated quaternion derivative
        self.SEq += (SEqDot_omega - self.beta * SEqHatDot) * self.deltat

        # Normalize quaternion
        self.SEq /= np.linalg.norm(self.SEq)

    def calculate_yaw(self):
        # Extract yaw from quaternion
        _, _, yaw = self.quaternion_to_euler(self.SEq)
        return yaw

    def quaternion_to_euler(self, quaternion):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        w, x, y, z = quaternion
        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x**2 + y**2))
        pitch = np.arcsin(2.0 * (w * y - z * x))
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = OrientationFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
