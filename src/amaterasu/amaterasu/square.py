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

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.start_x = None
        self.start_y = None       

        self.odom_sub = self.create_subscription(Odometry, "/ekf_odom", self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.draw_square_action_service = ActionServer(
            self,
            GoToGoal,
            "move_square_service",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.goal_x = 0
        self.goal_y = 0
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.r_desired = 0
        self.theta_desired = 0

        self.r_tolerance = 0.05 # meters
        self.theta_tolerance = 3 # degrees

        self.sample_time = 0.01 # s

        # PID parameters for turning
        self.kp_turn = 0.3
        self.ki_turn = 0.01
        self.kd_turn = 0.5

        self.prev_yaw_error = 0.0
        self.yaw_error_sum = 0.0

        self.max_integral = 100

        self.turn_start_time = None
        self.turn_duration = 0.0  # Set the turn duration (in seconds)
        self.is_turning = False
        self.turn_speed = 2.0 # radians/s

        self.execute_rate = self.create_rate(1/self.sample_time)

    # update position
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, self.current_theta = euler_from_quaternion(quaternion)
        
        # convert to degrees
        self.current_theta = self.current_theta * (180.0 / np.pi)
    
    def forward(self, speed):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def calculate_w(self, yaw_error):
        # Integral term
        self.yaw_error_sum += yaw_error * self.sample_time
        self.yaw_error_sum = max(min(self.yaw_error_sum, self.max_integral), -self.max_integral)

        # Derivative term
        yaw_error_derivative = (yaw_error - self.prev_yaw_error) / self.sample_time

        # PID formula
        angular_z = (
            self.kp_turn * yaw_error
        )

        return angular_z

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        return (angle + 180) % 360 - 180
    
    def calculate_theta_desired(self):
        diff_x = self.goal_x - self.current_x
        diff_y = self.goal_y - self.current_y
        theta = math.atan2(diff_y, diff_x) * (180.0 / np.pi) # degrees
        # self.get_logger().info(f"Curent theta: {self.current_theta} | Calculated theta: {theta}")
        return self.normalize_angle(theta - self.current_theta)

    def calculate_distance_to_goal(self):
        diff_x = self.current_x - self.goal_x
        diff_y = self.current_y - self.goal_y
        return math.sqrt((diff_x * diff_x) + (diff_y * diff_y))
    
    def generate_square_waypoints(self, start_x, start_y, size=1.0):
        return [
            (start_x + size, start_y),         # move right
            (start_x + size, start_y + size),  # move up
            (start_x, start_y + size),         # move left
            (start_x, start_y),                # move down (back to start)
        ]
        
    # Update goal coordinates
    def goal_callback(self, goal_request):
        self.goal_x = goal_request.x
        self.goal_y = goal_request.y

        self.get_logger().info(f'Received goal request: ({self.goal_x}, {self.goal_y})')
        self.get_logger().info(f'Current position: ({self.current_x}, {self.current_y})')
        self.get_logger().info(f'Distance to goal: {self.calculate_distance_to_goal()}')
        self.get_logger().info(f'---------------------------------------------------------')
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self.stop()
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        feedback = GoToGoal.Feedback()

        # Add initial goal to square waypoints
        square_waypoints = [(goal_handle.request.x, goal_handle.request.y)] + \
                        self.generate_square_waypoints(goal_handle.request.x, goal_handle.request.y)

        for idx, (gx, gy) in enumerate(square_waypoints):
            self.goal_x, self.goal_y = gx, gy

            while rclpy.ok():
                self.r_desired = self.calculate_distance_to_goal()
                self.theta_desired = self.calculate_theta_desired()

                yaw_error = self.theta_desired

                # Heading correction
                angular_z = self.calculate_w(yaw_error)

                # Linear speed only if facing target
                linear_x = 0.15 if abs(yaw_error) < self.theta_tolerance else 0.0

                # Stop condition
                if self.r_desired < self.r_tolerance:
                    self.stop()
                    break

                twist = Twist()
                twist.linear.x = linear_x
                twist.angular.z = angular_z
                self.cmd_vel_pub.publish(twist)

                feedback.current_x = self.current_x
                feedback.current_y = self.current_y
                feedback.distance = self.r_desired
                goal_handle.publish_feedback(feedback)

                self.execute_rate.sleep()

            self.get_logger().info(f"Reached waypoint {idx+1}")

        self.stop()
        result = GoToGoal.Result()
        result.goal_reached = True
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)

    move_square = MoveSquare()
    executor = MultiThreadedExecutor()

    rclpy.spin(move_square, executor=executor)

    move_square.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()