import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import math

from amaterasu_interfaces.action import GoToGoal


class BallGoalClient(Node):

    def __init__(self):
        super().__init__('ball_goal_client')
        self._action_client = ActionClient(self, GoToGoal, 'go_to_goal_service')
        self.ball_sub = self.create_subscription(Float32MultiArray, "/ball/bounding_box", self.ball_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.current_pose = (0.0, 0.0)
        self.balls = []

    def ball_callback(self, ball_list: Float32MultiArray):
        self.balls = [(ball[0], ball[1]) for ball in ball_list.data]
        self.sort_and_send_balls()
        

    def odom_callback(self, odom: Odometry):
        self.current_pose[0] = odom.pose.pose.position.x
        self.current_pose[1] = odom.pose.pose.position.y

    def sort_and_send_balls(self):
        self.balls.sort(key=lambda ball: math.sqrt((ball[0] - self.current_pose[0])**2 + (ball[1] - self.current_pose[1])**2))
        for ball in self.balls:
            self.send_goal(ball[0], ball[1])


    def send_goal(self, x, y):
        goal_msg = GoToGoal.Goal()
        goal_msg.x = x
        goal_msg.y = y

        self.get_logger().info(f'Sending goal: x={goal_msg.x}, y={goal_msg.y}')
        self._action_client.wait_for_server()

        self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: Current position ({feedback.current_x}, {feedback.current_y}), Distance: {feedback.distance}")


def main(args=None):
    rclpy.init(args=args)
    client = BallGoalClient()

    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()