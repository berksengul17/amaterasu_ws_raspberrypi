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
        
        # TURNING, MOVING, GOAL_REACHED
        self.state = "TURNING"

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.start_x = None
        self.start_y = None       

        self.square_distance = 1 # meter
        self.turn_count = 0

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

        self.r_tolerance = 0.1 # meters
        self.theta_tolerance = 5 # degrees

        self.sample_time = 0.01 # s

        # PID parameters for turning
        self.kp_turn = 0.03
        self.ki_turn = 0.0
        self.kd_turn = 0.0

        self.prev_yaw_error = 0.0
        self.yaw_error_sum = 0.0

        self.max_integral = 100

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

    def turn(self, yaw_error):
        twist = Twist()
        twist.linear.x = 0.0

        # Integral term
        self.yaw_error_sum += yaw_error * self.sample_time
        self.yaw_error_sum = max(min(self.yaw_error_sum, self.max_integral), -self.max_integral)

        # Derivative term
        yaw_error_derivative = (yaw_error - self.prev_yaw_error) / self.sample_time

        # PID formula
        angular_z = (
            self.kp_turn * yaw_error +
            self.ki_turn * self.yaw_error_sum +
            self.kd_turn * yaw_error_derivative
        )

        # Limit angular speed (optional)
        max_angular_speed = 1.5  # rad/s
        angular_z = max(min(angular_z, max_angular_speed), -max_angular_speed)

        twist.angular.z = -angular_z  # negative because of right-hand rule

        self.prev_yaw_error = yaw_error

        self.get_logger().info(f"Publishing angular speed: {-angular_z}")
        self.cmd_vel_pub.publish(twist)

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
        return self.normalize_angle(self.current_theta - theta)

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

    def update(self):
        self.theta_desired = self.calculate_theta_desired()
        self.r_desired = self.calculate_distance_to_goal()

        # self.get_logger().info(f'Current position: ({self.current_x}, {self.current_y}), {self.current_theta}')
        self.get_logger().info(f"Theta error: {self.theta_desired:.2f} | Distance to goal: {self.r_desired:.2f}")

        # If far from goal
        if self.r_desired > self.r_tolerance:
            # Integral term
            self.yaw_error_sum += self.theta_desired * self.sample_time
            self.yaw_error_sum = max(min(self.yaw_error_sum, self.max_integral), -self.max_integral)

            # Derivative term
            yaw_error_derivative = (self.theta_desired - self.prev_yaw_error) / self.sample_time

            # PID formula
            angular_z = (
                self.kp_turn * self.theta_desired +
                self.ki_turn * self.yaw_error_sum +
                self.kd_turn * yaw_error_derivative
            )

            # Limit angular speed (optional)
            max_angular_speed = 1.5  # rad/s
            angular_z = max(min(angular_z, max_angular_speed), -max_angular_speed)

            yaw_error_ratio = max(0.0, 1.0 - abs(self.theta_desired) / 90.0)  # [0,1]
            linear_x = self.r_desired * 0.4 * yaw_error_ratio if abs(self.theta_desired) < self.theta_tolerance else 0.0

            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = -angular_z # if abs(self.theta_desired) > self.theta_tolerance else 0.0 # negative due to right-hand rule

            self.get_logger().info(f"Linear: {linear_x} | Angular: {-angular_z}")
            self.cmd_vel_pub.publish(twist)
            self.prev_yaw_error = self.theta_desired

            return False  # Not yet at goal

        # At goal
        self.stop()
        return True


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

        # Step 1: Go to initial goal
        while not self.update():
            feedback.current_x = float(self.current_x)
            feedback.current_y = float(self.current_y)
            feedback.distance = float(self.calculate_distance_to_goal())
            goal_handle.publish_feedback(feedback)
            self.execute_rate.sleep()
        
        self.stop()
        self.get_logger().info("Reached initial goal.")

        # Step 2: Generate square waypoints
        square_waypoints = self.generate_square_waypoints(self.goal_x, self.goal_y)

        # Step 3: Follow each corner
        for idx, (next_x, next_y) in enumerate(square_waypoints):
            self.goal_x = next_x
            self.goal_y = next_y
            self.get_logger().info(f"Moving to square corner {idx+1}: ({next_x}, {next_y})")

            error = self.calculate_theta_desired()
            if (abs(error) > 5):
                self.state = "TURNING"
            else:
                self.state = "MOVING"

            self.get_logger().info(f"Error: {idx+1}: {error} - {self.state}")

            while not self.update():
                feedback.current_x = float(self.current_x)
                feedback.current_y = float(self.current_y)
                feedback.distance = float(self.calculate_distance_to_goal())
                goal_handle.publish_feedback(feedback)
                self.execute_rate.sleep()

            self.stop()
            self.get_logger().info(f"Reached corner {idx+1}")

        # Done!
        goal_handle.succeed()
        result = GoToGoal.Result()
        result.goal_reached = True
        self.get_logger().info(f'Square path complete. Returning result: {result.goal_reached}')
        return result
    
    def move_square(self):
        # square is completed
        if self.turn_count == 4:
            self.stop()
            self.get_logger().info("Square is completed...")
            return
        
        if self.state == "forward":
            if self.turn_count == 0:
                if self.start_x == None and self.start_y == None:
                    self.start_x = self.x
                    self.start_y = self.y

                self.get_logger().info(f"Started the square motion. Start X: {self.start_x} | Start Y: {self.start_y}")

            # if self.turn_count > 0:
            #     self.get_logger().info(f"Current position: {self.x} - {self.y}")
            #     self.stop()
            #     return 
            
            dist = math.sqrt((self.start_x - self.x)**2 + (self.start_y - self.y)**2)
            self.get_logger().info(f"Travelled distance: {dist}")
            
            self.forward((1 - dist) * 0.4)

            # one side is completed
            if dist >= 0.95:
                self.stop()
                self.state = "turning"
                self.turn_count += 1
                # self.turn_left()
                self.get_logger().info(f"Stopped. Travelled distance: {dist}")
        
        if self.state == "turning":
            yaw_diff = self.normalize_angle(self.turn_count * 90.0) - self.yaw
            self.get_logger().info(f"{self.turn_count * 90.0} | {self.yaw} | Yaw Error: {yaw_diff}")
            
            if abs(yaw_diff) <= 1.5 and self.state != "stopped":
                self.state = "forward"
                self.start_x = self.x
                self.start_y = self.y
                self.get_logger().info("Turn completed. Going forward...")
            else:
                self.turn_left(yaw_diff)

def main(args=None):
    rclpy.init(args=args)

    move_square = MoveSquare()
    executor = MultiThreadedExecutor()

    rclpy.spin(move_square, executor=executor)

    move_square.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
