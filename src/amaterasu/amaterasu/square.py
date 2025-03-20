import numpy as np
import math 

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from amaterasu_interfaces.action import GoToGoal

from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class MoveSquare(Node):
    def __init__(self):
        super().__init__("move_square")
        
        # stopped, forward, turning
        self.state = "turning"

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.start_x = None
        self.start_y = None       

        self.square_distance = 1 # meter
        self.turn_count = 0

        self.odom_sub = self.create_subscription(Odometry, "/ekf_odom", self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.01, self.move_square)

    # update posiiton
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, self.yaw = euler_from_quaternion(quaternion)
        
        # convert to degrees
        self.yaw = self.yaw * (180.0 / np.pi)
    
    def forward(self, speed):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def turn_left(self, yaw_error):
        twist = Twist()
        twist.linear.x = 0.0

        if yaw_error > 30:
            twist.angular.z = 1.5  # Reduce max speed to prevent overshoot
        elif yaw_error > 15:
            twist.angular.z = 0.8  # Reduce medium turn speed
        elif yaw_error > 10:
            twist.angular.z = 0.4  # Slow final adjustment
        else:
            twist.angular.z = 0.1  # Very slow fine-tuning

        self.cmd_vel_pub.publish(twist)


    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def move_square(self):
        # square is completed
        if self.turn_count == 4:
            self.stop()
            self.get_logger().info("Square is completed...")
            return
        
        # if self.state == "forward":
        #     if self.turn_count == 0:
        #         if self.start_x == None and self.start_y == None:
        #             self.start_x = self.x
        #             self.start_y = self.y

        #         self.get_logger().info(f"Started the square motion. Start X: {self.start_x} | Start Y: {self.start_y}")

        #     dist = abs(self.x - self.start_x)
        #     self.get_logger().info(f"Travelled distance: {dist}")
            
        #     self.forward((1 - dist) * 0.4)

        #     # one side is completed
        #     if dist >= 0.9:
        #         self.stop()
        #         self.state = "turning"
        #         self.turn_count += 1
        #         # self.turn_left()
        #         self.get_logger().info(f"Stopped. Travelled distance: {dist}")
        
        if self.state == "turning":
            yaw_diff = 90.0 - self.yaw
            self.get_logger().info(f"Yaw Error: {yaw_diff}")
            

            if abs(yaw_diff) <= 1.5 and self.state != "stopped":
                # self.state = "forward"
                # self.start_x = self.x
                # self.start_y = self.y
                self.stop()
                self.state = "stopped"
                self.get_logger().info("Turn completed. Going forward...")
            else:
                self.turn_left(yaw_diff)

def main(args=None):
    rclpy.init(args=args)

    move_square = MoveSquare()
    rclpy.spin(move_square)

    move_square.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
