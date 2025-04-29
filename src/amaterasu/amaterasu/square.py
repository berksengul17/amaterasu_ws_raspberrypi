import numpy as np
import math
import time

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

        self.odom_sub = self.create_subscription(Odometry, "/localization", self.odom_callback, 10)
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

        self.r_tolerance = 0.1 # meters
        self.theta_tolerance = 0.05 # radians ~ 5 degrees

        self.sample_time = 0.03 # s

        # PID parameters for turning
        self.kp_turn = 0.63
        self.ki_turn = 0.01
        self.kd_turn = 0.5

        self.prev_yaw_error = 0.0
        self.yaw_error_sum = 0.0

        self.max_integral = 100

        self.turn_start_time = None
        self.turn_duration = 0.0  # Set the turn duration (in seconds)
        self.is_turning = False
        self.turn_speed = 1.0 # radians/s

        self.execute_rate = self.create_rate(1/self.sample_time)

    # update position
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, self.current_theta = euler_from_quaternion(quaternion)
            
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

        # if (angular_z > 0 and angular_z < 1.5): angular_z = 1.5
        # elif (angular_z < 0 and angular_z > -1.5): angular_z = -1.5

        return angular_z

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def calculate_theta_desired(self):
        diff_x = self.goal_x - self.current_x
        diff_y = self.goal_y - self.current_y
        theta = math.atan2(diff_y, diff_x)
        # self.get_logger().info(f"Curent theta: {self.current_theta} | Calculated theta: {theta}")
        return self.normalize_angle(theta - self.current_theta)

    def calculate_distance_to_goal(self):
        diff_x = self.current_x - self.goal_x
        diff_y = self.current_y - self.goal_y
        return math.sqrt((diff_x * diff_x) + (diff_y * diff_y))
        
    def calculate_primary_direction_tolerance(self):
        # Calculate the differences in x and y directions
        diff_x = abs(self.goal_x - self.current_x)
        diff_y = abs(self.goal_y - self.current_y)

        # Define the base tolerance values
        tolerance = 0.04

        # If the goal is primarily along the x axis, apply tolerance in x
        if diff_x > diff_y:
            # Only check tolerance for x direction
            return 'x', tolerance
        # If the goal is primarily along the y axis, apply tolerance in y
        elif diff_y > diff_x:
            # Only check tolerance for y direction
            return 'y', tolerance
        # If the goal is diagonal, apply the same tolerance for both axes
        else:
            return 'xy', tolerance
    
    def generate_square_waypoints(self, start_x, start_y, size=0.8):
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
            direction, tolerance = self.calculate_primary_direction_tolerance()

            self.get_logger().info(f"Moving to waypoint {idx+1}: ({gx}, {gy})")
            self.get_logger().info(f"Direction: {direction} | Tolerance: {tolerance}")

            if (self.calculate_distance_to_goal() < self.r_tolerance):
                continue

            # Turn to desired orientation first
            while rclpy.ok():
                self.theta_desired = self.calculate_theta_desired()

                # Turn until orientation is correct
                if abs(self.theta_desired) <= self.theta_tolerance:
                    i = 0
                    while (i<10):
                        self.stop()
                        self.execute_rate.sleep()
                        i += 1

                    break

                linear_x = 0.0
                angular_z = self.calculate_w(self.theta_desired)

                twist = Twist()
                twist.linear.x = float(linear_x)
                twist.angular.z = float(angular_z)
                self.cmd_vel_pub.publish(twist)

                feedback.current_x = float(self.current_x)
                feedback.current_y = float(self.current_y)
                feedback.distance = float(self.r_desired)
                goal_handle.publish_feedback(feedback)

                self.execute_rate.sleep()

            # Now move forward to the goal
            while rclpy.ok():
                self.r_desired = self.calculate_distance_to_goal()
                self.theta_desired = self.calculate_theta_desired()

                # Stop condition
                if direction == 'x':
                    if abs(self.current_x - self.goal_x) < tolerance:
                        self.stop()
                        break
                elif direction == 'y':
                    if abs(self.current_y - self.goal_y) < tolerance:
                        self.stop()
                        break
                elif direction == 'xy':
                    # If moving diagonally, check both x and y tolerances
                    if (abs(self.current_x - self.goal_x) < tolerance and
                        abs(self.current_y - self.goal_y) < tolerance):
                        self.stop()
                        break

                # Move forward
                linear_x = 0.3
                angular_z = 0.0

                twist = Twist()
                twist.linear.x = float(linear_x)
                twist.angular.z = float(angular_z)
                self.cmd_vel_pub.publish(twist)

                feedback.current_x = float(self.current_x)
                feedback.current_y = float(self.current_y)
                feedback.distance = float(self.r_desired)
                goal_handle.publish_feedback(feedback)

                self.execute_rate.sleep()

            self.get_logger().info(f"Reached waypoint {idx+1} | Current position: ({self.current_x}, {self.current_y})")

        self.stop()  # Stop the robot after reaching all waypoints

        # Finish goal
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