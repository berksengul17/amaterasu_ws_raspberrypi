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
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.current_pose = [0.0, 0.0]
        self.current_goal = [0.0, 0.0]
        self.init = False
        self.balls = []
        self.goal_active = False

        self.get_logger().info("Ball goal client initialized.")

    def ball_callback(self, ball_list: Float32MultiArray):
        if self.init and len(self.balls) > 0:
            self.balls = [(ball_list.data[i], ball_list.data[i + 1]) for i in range(0, len(ball_list.data), 2)]
            self.sort_and_send_ball()
            # new_balls.sort(key=lambda ball: math.sqrt((ball[0] - self.current_pose[0])**2 + (ball[1] - self.current_pose[1])**2))

            # # self.compare_balls(new_balls)
            # for new_ball in new_balls:
            #     # Check if the new ball is already in the list within the tolerance
            #     if (self.is_ball_in_list(new_ball) == -1):
            #         self.balls.append(new_ball)  # Add to the ball list
            #         self.get_logger().info(f"New ball detected: {new_ball}.")

            # self.sort_and_send_balls()

    def odom_callback(self, odom: Odometry):
        self.current_pose[0] = odom.pose.pose.position.x
        self.current_pose[1] = odom.pose.pose.position.y
        # self.get_logger().info(f"Position updated: {self.current_pose}")
        
    def sort_and_send_ball(self):
        self.balls.sort(key=lambda ball: math.sqrt((ball[0] - self.current_pose[0])**2 + (ball[1] - self.current_pose[1])**2))
        if not self.balls:
            self.get_logger().info("No balls detected.")
            return

        # Only send a goal for the closest ball
        closest_ball = self.balls[0]
        if closest_ball != self.current_goal:  # Avoid resending the same goal
            self.current_goal = closest_ball
            self.send_goal(closest_ball[0], closest_ball[1])

    def send_goal(self, x, y):
        self.goal_active = True
        goal_msg = GoToGoal.Goal()
        goal_msg.x = x
        goal_msg.y = y

        self.get_logger().info(f'Sending goal: x={goal_msg.x}, y={goal_msg.y}')

        if (self._action_client.wait_for_server()):
            send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )

            # Add a result callback to handle the response
            send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().error("Action server not available. Cannot send goal.")
            self.goal_active = False  # Reset the flag if the server is unavailable

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_active = False
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.goal_reached:
            self.get_logger().info(f"Goal reached successfully at {self.current_goal}. Removing it from the list.")
            # Remove the current goal from the list
            self.balls = [ball for ball in self.balls if ball != self.current_goal]
            self.current_goal = [0.0, 0.0]
        else:
            self.get_logger().info(f"Failed to reach goal at {self.current_goal}.")

        self.goal_active = False  # Allow the next goal to be sent
        if self.balls:
            self.get_logger().info("Moving to the next closest ball.")
            self.sort_and_send_balls()
        else:
            self.get_logger().info("All goals completed.")


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
