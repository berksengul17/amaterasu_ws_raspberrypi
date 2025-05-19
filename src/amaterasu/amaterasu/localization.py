import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from tf_transformations import (
    quaternion_matrix,
    euler_from_quaternion,
    quaternion_from_matrix,
    quaternion_from_euler
)
import numpy as np


def alpha(cutoff, dt):
    return 1.0 / (1.0 + 1.0/(2*np.pi*cutoff*dt))

class OneEuro:
    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta       = beta
        self.d_cutoff   = d_cutoff
        self.x_prev     = None
        self.dx_low     = None
        self.x_low      = None

    def filt(self, x, dt):
        if self.x_prev is None:
            self.x_prev = x.copy()
            self.x_low  = x.copy()
            self.dx_low = np.zeros_like(x)
            return x
        dx = (x - self.x_prev) / dt
        a_d = alpha(self.d_cutoff, dt)
        self.dx_low = a_d*dx + (1-a_d)*self.dx_low
        cutoff = self.min_cutoff + self.beta*np.abs(self.dx_low)
        a = alpha(cutoff, dt)
        self.x_low = a*x + (1-a)*self.x_low
        self.x_prev = x.copy()
        return self.x_low

class AprilTagLocalizationNode(Node):
    def __init__(self):
        super().__init__('apriltag_localization')

        # parameters
        ns_val = self.declare_parameter('robot_ns', '').value
        self.tag_frame = self.declare_parameter('tag_frame', 'tag36h11_0').value
        self.global_tag = self.declare_parameter('global_tag_frame', 'tag36h11_0').value
        self.origin_frame = self.declare_parameter('origin_frame', 'camera').value
        prefix = f"/{ns_val}" if ns_val else ""

        # publishers & subscriptions
        self.odom_pub = self.create_publisher(Odometry, f"{prefix}/localization", 10)
        self.odom_sub = self.create_subscription(Odometry, f'{prefix}/odom', self.odom_callback, 10)

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # state
        self.T0 = None
        self.v = 0.0; self.w = 0.0
        self.last_stamp = None
        self.prev_pos = np.zeros(3)
        self.first_update = True

        self.T_x180 = np.array([
            [ 1,  0,  0, 0],
            [ 0, -1,  0, 0],
            [ 0,  0, -1, 0],
            [ 0,  0,  0, 1],
        ])

        # filters
        self.pos_filter = OneEuro(min_cutoff=2.0, beta=4.0, d_cutoff=5.0)
        self.yaw_filter = OneEuro(min_cutoff=1.0, beta=2.0, d_cutoff=5.0)
        self.v0, self.alpha_min, self.alpha_max = 0.2, 0.1, 0.9
        self.max_jump = 0.2

        self.timer = self.create_timer(0.04, self.lookup_transform)

    def odom_callback(self, msg: Odometry):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def compute_alpha(self, v):
        return self.alpha_min + (self.alpha_max-self.alpha_min)*(abs(v)/(abs(v)+self.v0))

    def lookup_transform(self):
        now = self.get_clock().now()
        if self.last_stamp is None:
            self.last_stamp = now
            return
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now

        try:
            tf_tag = self.tf_buffer.lookup_transform(self.origin_frame, self.tag_frame, rclpy.time.Time())
            t, q = tf_tag.transform.translation, tf_tag.transform.rotation
            T_ct = np.eye(4)
            T_ct[:3,:3] = quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
            T_ct[:3,3] = [t.x, t.y, t.z]

            T_ct = self.T_x180 @ T_ct

            # if self.T0 is None:
            #     tf_ref = self.tf_buffer.lookup_transform(self.origin_frame, self.global_tag, rclpy.time.Time())
            #     tr, qr = tf_ref.transform.translation, tf_ref.transform.rotation
            #     G = np.eye(4)
            #     G[:3,:3] = quaternion_matrix([qr.x,qr.y,qr.z,qr.w])[:3,:3]
            #     G[:3,3]  = [tr.x, tr.y, tr.z]
            #     self.T0 = G
            #     self.get_logger().info(f"Global origin set by {self.global_tag}")
            #     return

            # T0_inv = np.linalg.inv(self.T0)
            # Tor = T0_inv @ T_ct
            pos = T_ct[:3,3]
            yaw = euler_from_quaternion(quaternion_from_matrix(T_ct))[2]
            # yaw -= math.pi/2
            # yaw = math.atan2(math.sin(yaw), math.cos(yaw))

            # offset to center
            L,W = 0.15,0.13
            offset = np.array([np.cos(yaw)*L/2 + np.sin(yaw)*W/2,
                               np.sin(yaw)*L/2 + np.cos(yaw)*W/2, 0.0])
            pos[:2] += offset[:2]

            if self.first_update:
                self.prev_pos = np.array([pos[0], pos[1], yaw])
                self.first_update = False

            pred = self.prev_pos + self.v*np.array([np.cos(yaw), np.sin(yaw),0])*dt
            a = self.compute_alpha(self.v)
            if np.linalg.norm(pos[:2]-pred[:2]) > self.max_jump:
                fused = pred
            else:
                fused = a*pos + (1-a)*pred
            fused[:2] = self.pos_filter.filt(fused[:2], dt)
            fused[2]  = self.yaw_filter.filt(np.array([yaw]), dt)[0]
            self.prev_pos = fused.copy()

            # publish real odom
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.origin_frame
            odom.pose.pose.position.x = float(pos[0])
            odom.pose.pose.position.y = float(pos[1])
            q2 = quaternion_from_euler(0,0,float(yaw))
            odom.pose.pose.orientation.x = q2[0]; odom.pose.pose.orientation.y = q2[1]
            odom.pose.pose.orientation.z = q2[2]; odom.pose.pose.orientation.w = q2[3]
            self.odom_pub.publish(odom)

            self.get_logger().info(f"x: {pos[0]} y: {pos[1]} yaw: {np.rad2deg(yaw)}")

        except Exception as e:
            self.get_logger().warn(f"Transform unavailable: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()