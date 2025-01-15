import rclpy
import math
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

from amaterasu_interfaces.action import GoToGoal


class BallGoalClient(Node):

    def __init__(self):
        super().__init__('ball_goal_client')
        self._action_client = ActionClient(self, GoToGoal, 'go_to_goal_service')
        self.ball_sub = self.create_subscription(Float32MultiArray, "/ball/bounding_box", self.ball_callback, 10)
        self.robo_sub = self.create_subscription(Vector3, "/robot/bounding_box", self.robot_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.current_pose = [0.0, 0.0]
        self.current_goal = [0.0, 0.0]
        self.init = False
        self.balls = []

        self.get_logger().info("Ball goal client initialized.")

    def ball_callback(self, ball_list: Float32MultiArray):
        if self.init:
            new_balls = [(ball_list.data[i], ball_list.data[i + 1]) for i in range(0, len(ball_list.data), 2)]
            new_balls.sort(key=lambda ball: math.sqrt((ball[0] - self.current_pose[0])**2 + (ball[1] - self.current_pose[1])**2))

            # self.compare_balls(new_balls)
            for new_ball in new_balls:
                # Check if the new ball is already in the list within the tolerance
                if (self.is_ball_in_list(new_ball) == -1):
                    self.balls.append(new_ball)  # Add to the ball list
                    self.get_logger().info(f"New ball detected: {new_ball}. Sending goal...")
                    self.send_goal(new_ball[0], new_ball[1])
                else:
                    self.get_logger().info(f"Ball {new_ball} is already processed.")

    def robot_callback(self, msg: Vector3):
        if (self.init == False):
            self.current_pose = [msg.x, msg.y]
            self.init = True

    def odom_callback(self, odom: Odometry):
        self.current_pose[0] = odom.pose.pose.position.x
        self.current_pose[1] = odom.pose.pose.position.y

    def sort_and_send_balls(self):
        for ball in self.balls:
            self.current_goal = [ball[0], ball[1]]
            self.send_goal(ball[0], ball[1])

    def send_goal(self, x, y):
        goal_msg = GoToGoal.Goal()
        goal_msg.x = x - self.current_pose[0]
        goal_msg.y = y - self.current_pose[1]

        self.get_logger().info(f'Sending goal: x={goal_msg.x}, y={goal_msg.y}')
        self._action_client.wait_for_server()

        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: Current position ({feedback.current_x}, {feedback.current_y}), Distance: {feedback.distance}")

    def is_ball_in_list(self, new_ball):
        tolerance = 0.1
        for i in range(len(self.balls)):
            ball = self.balls[i]
            if abs(ball[0] - new_ball[0]) <= tolerance and abs(ball[1] - new_ball[1]) <= tolerance:
                return i  # Ball is already in the list
        return -1

    def create_marker(self, x, y):
        """
        Create a marker for RViz visualization.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        return marker

    # def combine_balls(self, new_balls):
    #     for new_ball in new_balls:
    #         if (self.is_ball_in_list(new_ball) != -1):
    #             continue

    #         self.balls.append(new_ball)






def main(args=None):
    rclpy.init(args=args)
    client = BallGoalClient()

    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
