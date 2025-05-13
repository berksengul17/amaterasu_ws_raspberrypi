#!/usr/bin/env python3
import rclpy
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
from collections import deque


def alpha(cutoff, dt):
    return 1.0 / (1.0 + 1.0/(2*np.pi*cutoff*dt))

class OneEuro:
    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta       = beta
        self.d_cutoff   = d_cutoff
        self.x_prev     = None
        self.dx_prev    = None
        self.x_low      = None
        self.dx_low     = None

    def filt(self, x, dt):
        if self.x_prev is None:
            self.x_prev, self.x_low = x.copy(), x.copy()
            self.dx_prev, self.dx_low = np.zeros_like(x), np.zeros_like(x)
            return x
        dx = (x - self.x_prev) / dt
        a_d = alpha(self.d_cutoff, dt)
        self.dx_low = a_d * dx + (1 - a_d) * self.dx_low
        cutoff = self.min_cutoff + self.beta * np.abs(self.dx_low)
        a = alpha(cutoff, dt)
        self.x_low = a * x + (1 - a) * self.x_low
        self.x_prev = x.copy()
        return self.x_low

class AprilTagLocalizationNode(Node):
    def __init__(self):
        super().__init__('apriltag_localization')

        # PARAMETERS
        self.declare_parameter('robot_ns', '')
        self.declare_parameter('tag_frame', 'tag36h11_0')
        self.declare_parameter('global_tag_frame', 'tag36h11_0')
        self.declare_parameter('origin_frame', 'camera')

        ns_val            = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.tag_frame    = self.get_parameter('tag_frame').get_parameter_value().string_value
        self.global_tag   = self.get_parameter('global_tag_frame').get_parameter_value().string_value
        self.origin_frame = self.get_parameter('origin_frame').get_parameter_value().string_value
        prefix           = f"/{ns_val}" if ns_val else ""

        # PUB & SUB
        self.odom_pub     = self.create_publisher(Odometry, f"{prefix}/localization", 10)
        self.sim_odom_pub = self.create_publisher(Odometry, f"{prefix}/simlocalization", 10)
        self.odom_sub     = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # TF SETUP
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # STATE
        self.T_cam_to_global_start = None
        self.v = 0.0; self.w = 0.0
        self.last_stamp = None
        self.prev_pos   = np.zeros(3)

        # SMOOTHING
        self.v0        = 0.2
        self.alpha_min = 0.1
        self.alpha_max = 0.9
        self.max_jump  = 0.2
        self.pos_filter = OneEuro(min_cutoff=2.0, beta=4.0, d_cutoff=5.0)
        self.yaw_filter = OneEuro(min_cutoff=1.0, beta=2.0, d_cutoff=5.0)

        # DELAY + INTERP QUEUES
        self.delay_buffer = []               # store fused poses for 1s
        self.delay_steps  = int(round(1.0 / 0.04))
        self.sim_queue    = deque()          # interpolated sim poses

        # TIMER
        self.timer = self.create_timer(0.04, self.lookup_transform)

    def odom_callback(self, msg: Odometry):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def compute_alpha(self, v):
        return self.alpha_min + (self.alpha_max - self.alpha_min) * (abs(v)/(abs(v)+self.v0))

    def lookup_transform(self):
        # first drain sim_queue if available
        if self.sim_queue:
            pose = self.sim_queue.popleft()
            self._publish_sim(pose)
            return

        # timestamp & dt
        now = self.get_clock().now()
        if self.last_stamp is None: self.last_stamp = now
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now

        try:
            # VISION TF
            tf_tag = self.tf_buffer.lookup_transform(self.origin_frame, self.tag_frame, rclpy.time.Time())
            t, q = tf_tag.transform.translation, tf_tag.transform.rotation
            T_ct = np.eye(4)
            T_ct[:3,:3] = quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
            T_ct[:3,3]  = [t.x, t.y, t.z]

            # GLOBAL ORIGIN
            if self.T_cam_to_global_start is None:
                tf_g = self.tf_buffer.lookup_transform(self.origin_frame, self.global_tag, rclpy.time.Time())
                tg, qg = tf_g.transform.translation, tf_g.transform.rotation
                G = np.eye(4)
                G[:3,:3] = quaternion_matrix([qg.x,qg.y,qg.z,qg.w])[:3,:3]
                G[:3,3]  = [tg.x, tg.y, tg.z]
                self.T_cam_to_global_start = G
                self.get_logger().info(f"⭑ Global origin set by '{self.global_tag}'")
                return

            # COMPUTE FUSED POSE
            T0_inv  = np.linalg.inv(self.T_cam_to_global_start)
            T_or    = T0_inv @ T_ct
            pos_v   = T_or[:3,3]
            rot_mat = quaternion_from_matrix(T_or)
            _,_,yaw = euler_from_quaternion(rot_mat)
            # robot center offset
            L, W = 0.15, 0.13
            off = np.array([np.cos(yaw)*L/2 + np.sin(yaw)*W/2,
                            np.sin(yaw)*L/2 + np.cos(yaw)*W/2, 0.0])
            pos_v[:2] += off[:2]

            # PREDICT & FUSE
            pred = self.prev_pos + self.v * np.array([np.cos(yaw), np.sin(yaw), 0.0]) * dt
            a    = self.compute_alpha(self.v)
            if np.linalg.norm(pos_v[:2]-pred[:2])>self.max_jump:
                fused = pred
            else:
                fused = a*pos_v + (1-a)*pred
            fused[:2] = self.pos_filter.filt(fused[:2], dt)
            fused[2]  = self.yaw_filter.filt(np.array([yaw]), dt)[0]
            self.prev_pos = fused.copy()

            # PUBLISH REAL ODOM
            self._publish_real(fused)

            # BUFFER for 1s
            self.delay_buffer.append(fused.copy())
            if len(self.delay_buffer)>=self.delay_steps:
                # take first & last, then clear buffer
                start = np.array(self.delay_buffer[0]); end = np.array(self.delay_buffer[-1])
                self.delay_buffer.clear()
                # interpolate into 100 equal parts
                for i in range(1,101):
                    α = i/100.0
                    interp = start + (end-start)*α
                    self.sim_queue.append(interp)
                # publish first sim now
                pose0 = self.sim_queue.popleft()
                self._publish_sim(pose0)

        except Exception as e:
            self.get_logger().warn(f"[{self.tag_frame}] transform unavailable: {e}")

    def _publish_real(self, pose):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.origin_frame
        odom.pose.pose.position.x = float(pose[0])
        odom.pose.pose.position.y = float(pose[1])
        odom.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0,0,float(pose[2]))
        odom.pose.pose.orientation.x = q[0]; odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]; odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)

    def _publish_sim(self, pose):
        sim = Odometry()
        sim.header.stamp = self.get_clock().now().to_msg()
        sim.header.frame_id = self.origin_frame
        sim.pose.pose.position.x = float(pose[0])*10.0
        sim.pose.pose.position.y = float(pose[1])*10.0
        sim.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0,0,float(pose[2]))
        sim.pose.pose.orientation.x = q[0]; sim.pose.pose.orientation.y = q[1]
        sim.pose.pose.orientation.z = q[2]; sim.pose.pose.orientation.w = q[3]
        self.sim_odom_pub.publish(sim)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
