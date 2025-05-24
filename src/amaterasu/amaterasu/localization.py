import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener
from tf_transformations import (
    quaternion_matrix,
    euler_from_quaternion,
    quaternion_from_matrix,
    quaternion_from_euler
)


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
        self.tag_frame     = self.declare_parameter('tag_frame', 'tag36h11_0').value
        self.global_tag    = self.declare_parameter('global_tag_frame', 'tag36h11_0').value
        self.origin_frame  = self.declare_parameter('origin_frame', 'camera').value
        prefix = f"/{ns_val}" if ns_val else ""

        # callback group for concurrency
        self.cb_group = ReentrantCallbackGroup()

        # publishers & subscriptions
        self.odom_pub = self.create_publisher(
            Odometry, f"{prefix}/localization", 10)
        self.odom_sub = self.create_subscription(
            Odometry, f"{prefix}/odom", self.odom_callback,
            10, callback_group=self.cb_group)
        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray, "/detections",
            self.detections_callback, 10, callback_group=self.cb_group)

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # state
        self.v = 0.0
        self.w = 0.0
        self.last_stamp = None
        self.prev_pos = np.zeros(3)
        self.prev_yaw = 0.0
        self.new_measurement = False

        # filters
        self.pos_filter = OneEuro(min_cutoff=2.0, beta=4.0, d_cutoff=5.0)
        self.yaw_filter = OneEuro(min_cutoff=1.0, beta=2.0, d_cutoff=5.0)
        self.v0, self.alpha_min, self.alpha_max = 0.2, 0.1, 0.9

        # spawn timer (prediction + conditional fusion)
        self.timer = self.create_timer(
            0.01, self.lookup_transform, callback_group=self.cb_group)

    def odom_callback(self, msg: Odometry):
        # update linear/angular velocities
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def detections_callback(self, msg: AprilTagDetectionArray):
        # mark that next timer event should fuse measurement
        for det in msg.detections:
            tag_id = f"{det.family}_{det.id}"
            if tag_id == self.tag_frame:
                self.new_measurement = True
                break

    def compute_alpha(self, v):
        # velocity-dependent blending factor
        return self.alpha_min + (self.alpha_max - self.alpha_min) * (
            abs(v) / (abs(v) + self.v0)
        )

    def lookup_transform(self):
        now = self.get_clock().now()
        if self.last_stamp is None:
            self.last_stamp = now
            return
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now

        # 1) DEAD-RECKONING PREDICTION
        # predict position and yaw from odometry
        pred_pos = (
            self.prev_pos
            + self.v * np.array([
                math.cos(self.prev_yaw),
                math.sin(self.prev_yaw),
                0.0
            ]) * dt
        )
        pred_yaw = self.prev_yaw + self.w * dt
        pred = np.array([pred_pos[0], pred_pos[1], pred_yaw])

        # apply one-euro filter to prediction
        pred[:2] = self.pos_filter.filt(pred[:2], dt)
        pred[2]  = self.yaw_filter.filt(np.array([pred[2]]), dt)[0]

        fused = pred.copy()

        # 2) MEASUREMENT FUSION if a new AprilTag was seen
        if self.new_measurement:
            try:
                tf_tag = self.tf_buffer.lookup_transform(
                    self.origin_frame, self.tag_frame, rclpy.time.Time())
                t, q = tf_tag.transform.translation, tf_tag.transform.rotation

                # camera->tag homogeneous transform
                T_ct = np.eye(4)
                T_ct[:3, :3] = quaternion_matrix([
                    q.x, q.y, q.z, q.w
                ])[:3, :3]
                T_ct[:3, 3] = [t.x, t.y, t.z]
                # if your camera frame is flipped, apply correction
                # T_ct = self.T_x180 @ T_ct  # uncomment if needed

                meas_pos = T_ct[:3, 3]
                meas_yaw = euler_from_quaternion(
                    quaternion_from_matrix(T_ct)
                )[2]

                # blend measurement and prediction
                a = self.compute_alpha(self.v)
                raw = (
                    a * np.array([meas_pos[0], meas_pos[1], meas_yaw])
                    + (1 - a) * pred
                )

                # filter the fused result
                fused[:2] = self.pos_filter.filt(raw[:2], dt)
                fused[2]  = self.yaw_filter.filt(np.array([raw[2]]), dt)[0]

                # clear measurement flag
                self.new_measurement = False

            except Exception as e:
                self.get_logger().warn(f"TF lookup failed: {e}")

        # 3) UPDATE STATE
        self.prev_pos = fused.copy()
        self.prev_yaw = fused[2]

        # 4) PUBLISH
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.origin_frame
        odom.pose.pose.position.x = float(fused[0])
        odom.pose.pose.position.y = float(fused[1])
        qf = quaternion_from_euler(0, 0, float(fused[2]))
        odom.pose.pose.orientation.x = qf[0]
        odom.pose.pose.orientation.y = qf[1]
        odom.pose.pose.orientation.z = qf[2]
        odom.pose.pose.orientation.w = qf[3]
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizationNode()

    # use a multithreaded executor so timer + subscriptions can run in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
