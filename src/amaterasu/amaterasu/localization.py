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
from apriltag_msgs.msg import AprilTagDetectionArray


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
        self.detections_sub = self.create_subscription(AprilTagDetectionArray, "/detections", self.detections_callback, 10)

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # state
        self.T0 = None
        self.v = 0.0; self.w = 0.0
        self.last_stamp = None
        self.prev_pos = np.zeros(3)
        self.prev_yaw = 0.0
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
        self.new_measurement = False

        self.timer = self.create_timer(0.01, self.lookup_transform)

    def odom_callback(self, msg: Odometry):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def detections_callback(self, msg: AprilTagDetectionArray): 
        self.new_measurement = False
        for detection in msg.detections:
            if (f"{detection.family}_{detection.id}" == self.tag_frame):
                self.new_measurement = True

    def compute_alpha(self, v):
        return self.alpha_min + (self.alpha_max-self.alpha_min)*(abs(v)/(abs(v)+self.v0))

    def lookup_transform(self):
        now = self.get_clock().now()
        if self.last_stamp is None:
            self.last_stamp = now
            return
        dt = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now

        # 1) DEAD-RECKONING PREDICTION from last state:
        #    x_pred = x_prev + v * cos(yaw) * dt
        #    y_pred = y_prev + v * sin(yaw) * dt
        #    yaw_pred = yaw_prev + w * dt   (if you want to include angular)
        pred = self.prev_pos + self.v * np.array([np.cos(self.prev_yaw),
                                                np.sin(self.prev_yaw),
                                                0.0]) * dt

        # 2) TRY APRILTAG MEASUREMENT
        try:
            tf_tag = self.tf_buffer.lookup_transform(
                self.origin_frame, self.tag_frame, rclpy.time.Time())
            t, q = tf_tag.transform.translation, tf_tag.transform.rotation
            
            T_ct = np.eye(4)
            T_ct[:3,:3] = quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
            T_ct[:3,3] = [t.x, t.y, t.z]
            T_ct = self.T_x180 @ T_ct

            meas_pos = T_ct[:3,3]
            meas_yaw = euler_from_quaternion(quaternion_from_matrix(T_ct))[2]

            # 3) FUSE (if you want, include your OneEuro filters here)
            if self.new_measurement:
                # simple α-blend between pred and meas
                a = self.compute_alpha(self.v)
                fused = a * np.array([meas_pos[0], meas_pos[1], meas_yaw]) + \
                        (1-a) * pred
            else:
                fused = pred

            # update internal state
            fused[:2] = self.pos_filter.filt(fused[:2], dt)
            fused[2]  = self.yaw_filter.filt(np.array([fused[2]]), dt)[0]

            self.prev_pos = fused.copy()
            self.prev_yaw = fused[2]

            # 4) PUBLISH fused estimate as Odometry
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.origin_frame
            odom.pose.pose.position.x = float(fused[0])
            odom.pose.pose.position.y = float(fused[1])
            qf = quaternion_from_euler(0, 0, float(meas_yaw))
            odom.pose.pose.orientation.x = qf[0]
            odom.pose.pose.orientation.y = qf[1]
            odom.pose.pose.orientation.z = qf[2]
            odom.pose.pose.orientation.w = qf[3]
            self.odom_pub.publish(odom)

            self.get_logger().info(f"loc x={fused[0]:.2f} y={fused[1]:.2f} yaw={np.rad2deg(fused[2]):.1f}")
            
        except Exception as e:
            self.get_logger().warn(f"No tag measurement: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()