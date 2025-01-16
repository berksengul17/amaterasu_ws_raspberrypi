from os import times
import rclpy
import time
import math
import csv
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from amaterasu_interfaces.action import GoToGoal
from amaterasu.pid import PidController

class GoToGoalNode(Node):
    """Node for high level control of a differential drive robot"""
    def __init__(self):
        
        super().__init__('go_to_goal_node')
        self.get_logger().info("creating publisher...")
        self.twist_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info("creating odometry subscriber...")
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.odom_subscription

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )


        self.get_logger().info("creating action server...")
        self.go_to_goal_action_service = ActionServer(
            self,
            GoToGoal,
            'go_to_goal_service',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.gyro_z = 0.0
        self.last_gyro_update = self.get_clock().now()

        # transform
        self.tf_broadcaster = TransformBroadcaster(self)
        # important variables
        self.is_moving = False
        self.pos_tolerance = 0.22 # 20 cm

        self.goal_x = 0
        self.goal_y = 0
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0.0

        self.gtg_r = 0
        self.gtg_theta = 0
        self.theta_desired = 0
        self.r_desired = 0

        self.sample_time = 0.1
        self.max_linear_v = 0.2
        self.alpha = 2
        # 0.4, 0.001, 0.002
        # 3.88 3.78 3.94 -> 1.75
        # 0.4, 0.006, 0.0005
        # 0.4, 0.002, 0.0010
        # 0.4, 0.002, 0.0020
        self.angle_pid = PidController(0.4, 0.002, 0.0005, self.sample_time, True)
        # -1, 1
        self.angle_pid.set_output_limits(-3.14, 3.14)

        self.get_logger().info("initialization finished")

        self.linear_command = 0
        self.angular_command = 0

        self.execute_rate = self.create_rate(1/self.sample_time)

        self.yaw_correction_factor = 0.85  # Weight for IMU vs odometry


    def imu_callback(self, imu_msg):
        """Update the latest gyroscopic angular velocity."""
        self.gyro_z = imu_msg.angular_velocity.z * math.pi / 180.0
        self.last_gyro_update = self.get_clock().now()

    def odom_callback(self, odom: Odometry):
        # Odometry-based position and yaw
        self.current_x = odom.pose.pose.position.x
        self.current_y = odom.pose.pose.position.y
        odometry_theta = self.get_euler_from_quaternion(odom.pose.pose.orientation)[2]

        # Time delta
        current_time = self.get_clock().now()
        time_delta = (current_time - self.last_gyro_update).nanoseconds / 1e9

        # IMU-based yaw correction
        imu_yaw_delta = self.gyro_z * time_delta
        corrected_theta = (
            self.yaw_correction_factor * odometry_theta +
            (1 - self.yaw_correction_factor) * (self.current_theta + imu_yaw_delta)
        )

        # Update current theta with corrected value
        self.current_theta = corrected_theta

        self.get_logger().info(f"Corrected Yaw: {self.current_theta:.2f}, IMU Yaw Delta: {imu_yaw_delta:.2f}")

        # Compute go-to-goal vector
        self.gtg_r, self.gtg_theta = self.get_go_to_goal_vector()
        self.publish_ref_vectors()

        
    def add_two_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request a:{request.a}, b:{request.b}')
        return response

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action"""
        if self.is_moving:
            return GoalResponse.REJECT

        self.goal_x = goal_request.x
        self.goal_y = goal_request.y
        self.get_logger().info(f'Received goal request: ({self.goal_x}, {self.goal_y})')
        self.get_logger().info(f'Distance to goal: {self.get_distance_to_goal()}')
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info('Received cancel request')
        # TODO: implement logic to cancel request
        return CancelResponse.ACCEPT

    # async method
    async def execute_callback(self, goal_handle):
        """Execute a goal"""
        self.get_logger().info('Executing action...')

        feedback_msg = GoToGoal.Feedback()
        log_file = open(f'/home/amaterasu/log/pid.log', 'w')

        while not self.update_control_loop():
            self.is_moving = True
            feedback_msg.current_x = float(self.current_x)
            feedback_msg.current_y = float(self.current_y)
            feedback_msg.distance = float(self.get_distance_to_goal())
            self.get_logger().info(f'current pos: {feedback_msg.current_x},{feedback_msg.current_y}, distance to goal: {self.get_distance_to_goal()}')
            self.get_logger().info(f'go_to_goal: [{self.gtg_r:.2f},{self.gtg_theta:.2f}]')
            self.send_twist_command()
            goal_handle.publish_feedback(feedback_msg)
            log_str = f'{int(time.time() * 1000)},{self.current_x},{self.current_y},{self.current_theta},{self.get_distance_to_goal()},{self.linear_command},{self.angular_command}\n'
            #self.get_logger().info(log_str)
            log_file.write(log_str)
            # time.sleep(self.sample_time)
            self.execute_rate.sleep()

        # should be 0
        self.send_twist_command()
        self.is_moving = False
        log_str = f'{int(time.time() * 1000)},{self.current_x},{self.current_y},{self.get_distance_to_goal()},{self.linear_command},{self.angular_command}\n'
        log_file.write(log_str)
        log_file.close()
        goal_handle.succeed()
        result = GoToGoal.Result()
        result.goal_reached = True
        self.get_logger().info(f'Returning result: {result.goal_reached}')
        return result


    def update_control_loop(self):
        self.r_desired = self.gtg_r
        # arctan2(r2sin(ϕ2−ϕ1),r1+r2cos(ϕ2−ϕ1))
        self.theta_desired = self.gtg_theta
        # If near the goal, stop the robot
        if self.get_distance_to_goal() <= self.pos_tolerance:
            self.linear_command = 0
            self.angular_command = 0
            self.get_logger().info(f"Near goal, stopping. Distance: {self.get_distance_to_goal()}")
            return True

        # Input to PID: corrected yaw
        self.angle_pid.set_input(self.current_theta)
        self.angle_pid.set_setpoint(self.theta_desired)

        # Compute PID outputs
        self.angle_pid.compute()
        self.linear_command = self.r_desired
        self.angular_command = self.angle_pid.get_output()

        return False



    def get_go_to_goal_vector(self):
        error_x = self.goal_x - self.current_x
        error_y = self.goal_y - self.current_y
        # self.get_logger().info(f'gtg error: ({error_x}, {error_y})')
        # compute K for linear velocity
        distance_to_goal = self.get_distance_to_goal()
        # self.get_logger().info(f'gtg distance: {distance_to_goal}')

        K = (self.max_linear_v * (1 - math.exp(-self.alpha * (distance_to_goal * distance_to_goal))) / (distance_to_goal * distance_to_goal)) if distance_to_goal > 0.0001 else 0

        # compute control signal
        u_x = K * error_x
        u_y = K * error_y

        # convert to polar coordinates 
        r = math.sqrt((u_x * u_x) + (u_y * u_y))
        theta = math.atan2(u_y, u_x)
        return r, theta

    def get_distance_to_goal(self):
        return math.sqrt(math.pow(self.goal_x - self.current_x, 2) + math.pow(self.goal_y - self.current_y, 2))

    def get_euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr

        return q

    def send_twist_command(self):
        if self.is_moving:
            twist_msg = Twist()
            twist_msg.linear.x = float(self.linear_command)
            twist_msg.angular.z = float(self.angular_command)
            self.get_logger().info(f'Publishing twist: {twist_msg}')
            self.twist_publisher.publish(twist_msg)

    def publish_ref_vectors(self):
        timestamp = self.get_clock().now().to_msg()
        t_goal = TransformStamped()
        t_goal.header.stamp = timestamp
        t_goal.header.frame_id = '/base_link'
        t_goal.child_frame_id = '/gtg'

        t_goal.transform.rotation = self.quaternion_from_euler(0, 0, self.gtg_theta)

        # t_obstacle = TransformStamped()
        # t_obstacle.header.stamp = timestamp
        # t_obstacle.header.frame_id = '/base_link'
        # t_obstacle.child_frame_id = '/ao'

        # t_obstacle.transform.rotation = self.quaternion_from_euler(0, 0, self.ao_theta)

        t_desired = TransformStamped()
        t_desired.header.stamp = timestamp
        t_desired.header.frame_id = '/base_link'
        t_desired.child_frame_id = '/desired'

        t_desired.transform.rotation = self.quaternion_from_euler(0, 0, self.theta_desired)
        # self.get_logger().info('Publishing transforms for vectors')
        self.tf_broadcaster.sendTransform(t_goal)
        # self.tf_broadcaster.sendTransform(t_obstacle)
        self.tf_broadcaster.sendTransform(t_desired)


def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoalNode()

    executor = MultiThreadedExecutor()

    rclpy.spin(go_to_goal_node, executor=executor)

    go_to_goal_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()