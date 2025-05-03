import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from amaterasu_interfaces.action import GoToGoal
from std_msgs.msg import Float32MultiArray

class MoveSquareClient(Node):
    def __init__(self):
        super().__init__('move_square_client')

        # Create the action client
        self._client = ActionClient(self, GoToGoal, 'move_square_service')
        # self.create_subscription(Float32MultiArray, "/fire_cell_goal", self.fire_callback, 10)
        
        self.executing_goal = False
        self.current_goal_index = 0  # Track the index of the current goal
        self.goals = []

    def fire_callback(self, msg):
        if len(msg.data) >= 2:
            goal = (msg.data[0], msg.data[1])
            self.goals.append(goal)
            self.get_logger().info(
                f"Queued goal: x={goal[0]:.2f}, y={goal[1]:.2f}"
            )
            self.send_goals(self.goals)

    def send_goals(self, goals):
        self.goals = goals
        # Wait for the action server to be ready
        self.get_logger().info('Waiting for action server...')
        self._client.wait_for_server()

        # Send the goal if there are still goals to send
        if self.current_goal_index < len(goals) and not self.executing_goal:
            self.executing_goal = True
            self.send_next_goal(goals)

    def send_next_goal(self, goals):
        if self.current_goal_index < len(goals):
            x = goals[self.current_goal_index][0]
            y = goals[self.current_goal_index][1]

            goal_msg = GoToGoal.Goal()
            goal_msg.x = float(x)
            goal_msg.y = float(y)

            # Send the goal asynchronously and wait for feedback
            self.get_logger().info(f'Sending goal {self.current_goal_index + 1}: ({x}, {y})')
            self.current_goal_index += 1

            goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Print the feedback received from the server
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: current_x={feedback.current_x}, current_y={feedback.current_y}, distance={feedback.distance}')

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            self.executing_goal = False
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.goal_reached}')
        if result.goal_reached:
            # Trigger the next goal
            self.executing_goal = False
            self.send_goals(self.goals)  # Send the next goal if there are more
        else:
            self.executing_goal = False

def main(args=None):
    rclpy.init(args=args)

    move_square_client = MoveSquareClient()

    # Example list of goals (x, y) coordinates for a square path
    goals = [
        (0.8, 0.8),
        (0.8, 0.0),
        (0.0, 0.8),
        (0.0, 0.0)
    ]

    # Send the goals
    move_square_client.send_goals(goals)

    # Spin the client to keep it active
    rclpy.spin(move_square_client)

    # Clean up
    move_square_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
