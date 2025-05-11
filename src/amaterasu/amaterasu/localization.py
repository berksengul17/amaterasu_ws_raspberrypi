#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
from tf_transformations import (
    quaternion_matrix,
    euler_from_quaternion,
    quaternion_from_matrix
)
import numpy as np

class AprilTagLocalizationNode(Node):
    def __init__(self):
        super().__init__('apriltag_localization')

        # ─── 1) PARAMETERS ────────────────────────────────────────────────
        self.declare_parameter('robot_ns', '')              # e.g. "robot1"
        self.declare_parameter('tag_frame', 'tag36h11_0')   # this node’s AprilTag
        self.declare_parameter('global_tag_frame', 'tag36h11_0')  
        self.declare_parameter('origin_frame', 'camera')    # fixed world frame

        ns_val           = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.tag_frame   = self.get_parameter('tag_frame').get_parameter_value().string_value
        self.global_tag  = self.get_parameter('global_tag_frame').get_parameter_value().string_value
        self.origin_frame= self.get_parameter('origin_frame').get_parameter_value().string_value

        prefix = f"/{ns_val}" if ns_val else ""

        # ─── 2) PUBLISHERS ─────────────────────────────────────────────
        self.odom_pub     = self.create_publisher(
            Odometry, f"{prefix}/localization", 10)
        self.sim_odom_pub     = self.create_publisher(
            Odometry, f"{prefix}/sim_localization", 10)

        # ─── 3) TF SETUP ────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ─── 4) STORAGE ─────────────────────────────────────────────────
        # This will hold the camera→global_tag transform at t₀
        self.T_cam_to_global_start = None

        # ─── 5) SPIN ────────────────────────────────────────────────────
        self.timer = self.create_timer(0.01, self.lookup_transform)


    def lookup_transform(self):
        try:
            # 1) grab camera→thisTag right now
            tf_tag = self.tf_buffer.lookup_transform(
                self.origin_frame,
                self.tag_frame,
                rclpy.time.Time())

            # build 4×4 matrix for camera→tag
            t = tf_tag.transform.translation
            q = tf_tag.transform.rotation

            T_cam_to_tag = np.eye(4)
            T_cam_to_tag[:3,:3] = quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
            T_cam_to_tag[:3,3]  = [t.x, t.y, t.z]

            # 2) if we haven’t frozen our global origin yet, do it now
            if self.T_cam_to_global_start is None:
                # always use the global_tag_frame’s transform
                tf_global = self.tf_buffer.lookup_transform(
                    self.origin_frame,
                    self.global_tag,
                    rclpy.time.Time())

                tg = tf_global.transform.translation
                qg = tf_global.transform.rotation

                G = np.eye(4)
                G[:3,:3] = quaternion_matrix([qg.x,qg.y,qg.z,qg.w])[:3,:3]
                G[:3,3]  = [tg.x, tg.y, tg.z]

                self.T_cam_to_global_start = G
                self.get_logger().info(f"⭑ Global origin set by '{self.global_tag}'")
                return

            # 3) compute this robot’s pose relative to that common origin:
            #     T_origin_to_robot = (T_cam→global_start)⁻¹ × (T_cam→thisTag)
            T0_inv = np.linalg.inv(self.T_cam_to_global_start)
            T_origin_to_robot = T0_inv @ T_cam_to_tag

            # extract translation & rotation
            pos = T_origin_to_robot[:3,3]
            rot = quaternion_from_matrix(T_origin_to_robot)
            _,_,yaw = euler_from_quaternion(rot)

            # apply your chassis‐offset
            L, W = 0.15, 0.13
            offset = np.array([
                np.cos(yaw)*L/2 + np.sin(yaw)*W/2,
                np.sin(yaw)*L/2 + np.cos(yaw)*W/2,
                0.0
            ])
            pos += offset

            # 4) PUBLISH ODOM
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = self.origin_frame
            odom.pose.pose.position.x = float(pos[0])
            odom.pose.pose.position.y = float(pos[1])
            odom.pose.pose.position.z = float(pos[2])
            odom.pose.pose.orientation.x = float(rot[0])
            odom.pose.pose.orientation.y = float(rot[1])
            odom.pose.pose.orientation.z = float(rot[2])
            odom.pose.pose.orientation.w = float(rot[3])
            self.odom_pub.publish(odom)

            odom.pose.pose.position.x = float(pos[0] * 10)
            odom.pose.pose.position.y = float(pos[1] * 10)
            odom.pose.pose.position.z = 0.005
            self.sim_odom_pub.publish(odom)

            self.get_logger().info(
                f"[{self.tag_frame}] → x:{pos[0]:.3f} y:{pos[1]:.3f} yaw:{np.degrees(yaw):.1f}°"
            )

        except Exception as e:
            self.get_logger().warn(
                f"[{self.tag_frame}] transform unavailable: {e}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
