import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from collections import deque

class SimPlaybackNode(Node):
    def __init__(self):
        super().__init__('sim_playback')
        ns_val = self.declare_parameter('robot_ns', 'robot1').value
        prefix = f"/{ns_val}" if ns_val else ""

        self.sub = self.create_subscription(
            Odometry, f"{prefix}/localization", self.odom_cb, 10)
        self.sim_pub = self.create_publisher(
            Odometry, f"{prefix}/simlocalization", 10)

        self.delay_buffer = []
        self.delay_steps  = int(round(1.0/0.04))
        self.sim_queue    = deque()

        self.timer = self.create_timer(0.04, self.publish_sim)

    def odom_cb(self, msg: Odometry):
        # extract pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
        self.delay_buffer.append(np.array([x,y,yaw]))

        if len(self.delay_buffer) >= self.delay_steps:
            pts = np.array(self.delay_buffer)
            self.delay_buffer.clear()
            N = pts.shape[0]; M = self.delay_steps
            t_old = np.linspace(0,1,N)
            t_new = np.linspace(0,1,M)
            interp_pts = np.stack([
                np.interp(t_new, t_old, pts[:,d])
                for d in range(3)
            ], axis=1)
            for p in interp_pts:
                self.sim_queue.append(p)

    def publish_sim(self):
        self.get_logger().info(f"{self.sim_queue}")
        if not self.sim_queue:
            return
        p = self.sim_queue.popleft()
        sim = Odometry()
        sim.header.stamp = self.get_clock().now().to_msg()
        sim.header.frame_id = 'camera'
        sim.pose.pose.position.x = float(p[0]) * 10.0
        sim.pose.pose.position.y = float(p[1]) * 10.0
        sim.pose.pose.position.z = 0.022
        q = quaternion_from_euler(0,0,float(p[2]))
        sim.pose.pose.orientation.x = q[0]; sim.pose.pose.orientation.y = q[1]
        sim.pose.pose.orientation.z = q[2]; sim.pose.pose.orientation.w = q[3]        
        self.sim_pub.publish(sim)

        self.get_logger().info(f"x: {p[0]} y: {p[1]}")


def main(args=None):
    rclpy.init(args=args)
    node = SimPlaybackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()