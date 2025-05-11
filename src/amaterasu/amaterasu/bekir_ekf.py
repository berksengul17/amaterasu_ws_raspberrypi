#!/usr/bin/env python3
import numpy as np
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster

class ExtendedKalmanFilter(Node):
    def __init__(self):
        super().__init__('ekf')

        # State vector: [x, y, theta, v, w]
        self.mu = np.zeros(5)

        # Covariances
        self.P = np.diag([0.1, 0.1, 0.1, 0.1, 0.1])  # state covariance
        self.Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1])  # process noise

        # Measurement model for encoder-only odometry: [v, w]
        self.H_odom = np.array([
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])
        self.R_odom = np.diag([0.1, 0.02])  # odometry noise

        # Measurement model for AprilTag pose: [x, y, theta]
        self.H_tag = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0]
        ])
        self.R_tag = np.diag([0.02, 0.02, np.deg2rad(2.0)**2])  # tag pose noise

        # Last encoder readings
        self.v_odom = 0.0
        self.w_odom = 0.0

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/localization', self.tag_callback, 10)

        # Publisher and TF broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timing
        self.dt = 0.01
        self.last_time = time.time()
        self.create_timer(self.dt, self.run)

    def normalize_angle(self, angle: float) -> float:
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def odom_callback(self, msg: Odometry):
        # Encoder-based velocities
        self.v_odom = msg.twist.twist.linear.x
        self.w_odom = msg.twist.twist.angular.z

    def tag_callback(self, msg: Odometry):
        # Absolute pose from AprilTag
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw_tag = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw_tag = self.normalize_angle(yaw_tag)
        z_tag = np.array([px, py, yaw_tag])

        # EKF update with tag pose
        y = z_tag - self.H_tag.dot(self.mu)
        y[2] = self.normalize_angle(y[2])
        S = self.H_tag.dot(self.P).dot(self.H_tag.T) + self.R_tag
        K = self.P.dot(self.H_tag.T).dot(np.linalg.inv(S))
        self.mu += K.dot(y)
        self.mu[2] = self.normalize_angle(self.mu[2])
        self.P = (np.eye(5) - K.dot(self.H_tag)).dot(self.P)

    def run(self):
        # Time update
        curr_time = time.time()
        dt = curr_time - self.last_time
        self.last_time = curr_time

        # Predict
        x, y, theta, v, w = self.mu
        self.mu = np.array([
            x + v * dt * np.cos(theta),
            y + v * dt * np.sin(theta),
            theta + w * dt,
            v,
            w
        ])

        F = np.array([
            [1, 0, -v * dt * np.sin(theta), dt * np.cos(theta), 0],
            [0, 1,  v * dt * np.cos(theta), dt * np.sin(theta), 0],
            [0, 0, 1,                       0,                dt],
            [0, 0, 0,                       1,                0],
            [0, 0, 0,                       0,                1]
        ])
        self.P = F.dot(self.P).dot(F.T) + self.Q

        # EKF update with encoder odometry
        z = np.array([self.v_odom, self.w_odom])
        y = z - self.H_odom.dot(self.mu)
        S = self.H_odom.dot(self.P).dot(self.H_odom.T) + self.R_odom
        K = self.P.dot(self.H_odom.T).dot(np.linalg.inv(S))
        self.mu += K.dot(y)
        self.P = (np.eye(5) - K.dot(self.H_odom)).dot(self.P)

        # Publish fused odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(self.mu[0])
        odom.pose.pose.position.y = float(self.mu[1])
        odom.pose.pose.position.z = 0.0

        self.get_logger().info(f"x: {self.mu[0]} - y: {self.mu[1]}")

        qt = quaternion_from_euler(0, 0, self.mu[2])
        odom.pose.pose.orientation = Quaternion(
            x=float(qt[0]), y=float(qt[1]), z=float(qt[2]), w=float(qt[3])
        )

        odom.pose.covariance = [
            self.P[0,0], 0, 0, 0, 0, 0,
            0, self.P[1,1], 0, 0, 0, 0,
            0, 0, self.P[2,2], 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0
        ]

        odom.twist.twist.linear.x = float(self.mu[3])
        odom.twist.twist.angular.z = float(self.mu[4])
        odom.twist.covariance = [
            self.P[3,3], 0, 0, 0, 0, 0,
            0, self.P[4,4], 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0
        ]

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = float(self.mu[0])
        tf_msg.transform.translation.y = float(self.mu[1])
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    ekf = ExtendedKalmanFilter()
    rclpy.spin(ekf)
    ekf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
