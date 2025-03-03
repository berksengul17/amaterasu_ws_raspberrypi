import rclpy
import math
import time
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from amaterasu_interfaces.action import GoToGoal

class FireCellGoalClient(Node):
    def __init__(self):
        super().__init__('fire_cell_goal_client')
        self._action_client = ActionClient(self, GoToGoal, 'go_to_goal_service')
        # Subscribe to grid data (fire counts per cell)
        self.grid_sub = self.create_subscription(
            Int32MultiArray, "/grid/fire_count", self.grid_callback, 10
        )
        # Subscribe to odometry to get current robot pose
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        
        self.current_pose = [0.0, 0.0]
        self.grid_data = None
        self.last_update_time = 0.0
        self.update_interval = 1.0  # seconds

        # Parameters for grid dimensions and platform size (real-world)
        self.declare_parameter("grid_rows", 10)
        self.declare_parameter("grid_cols", 10)
        self.declare_parameter("platform_width", 2.10)
        self.declare_parameter("platform_height", 1.54)
        
        self.get_logger().info("Fire Cell Goal Client initialized.")

    def odom_callback(self, odom_msg):
        # Update current robot pose
        self.current_pose[0] = odom_msg.pose.pose.position.x
        self.current_pose[1] = odom_msg.pose.pose.position.y

    def grid_callback(self, msg: Int32MultiArray):
        # Update the grid data (fire counts) from detection node.
        self.grid_data = msg.data  # Expecting a flat list in row-major order.
        current_time = time.time()
        if current_time - self.last_update_time >= self.update_interval:
            self.last_update_time = current_time
            self.process_grid_and_send_goal()

    def process_grid_and_send_goal(self):
        if self.grid_data is None:
            return

        grid_rows = self.get_parameter("grid_rows").value
        grid_cols = self.get_parameter("grid_cols").value
        platform_width = self.get_parameter("platform_width").value
        platform_height = self.get_parameter("platform_height").value

        cell_width = platform_width / grid_cols
        cell_height = platform_height / grid_rows

        candidate_cells = []
        epsilon = 1e-3  # To avoid division by zero

        # Evaluate each cell with nonzero fire count.
        for r in range(grid_rows):
            for c in range(grid_cols):
                index = r * grid_cols + c
                fire_count = self.grid_data[index]
                if fire_count > 0:
                    # Compute cell center in real-world coordinates.
                    center_x = (c + 0.5) * cell_width
                    center_y = (r + 0.5) * cell_height
                    # Compute Euclidean distance from current robot pose to cell center.
                    distance = math.sqrt((center_x - self.current_pose[0])**2 + 
                                         (center_y - self.current_pose[1])**2)
                    # Compute score: lower score is preferred.
                    score = distance / (fire_count + epsilon)
                    candidate_cells.append({
                        'row': r,
                        'col': c,
                        'center': (center_x, center_y),
                        'fire_count': fire_count,
                        'distance': distance,
                        'score': score
                    })

        if not candidate_cells:
            self.get_logger().info("No active fire cells detected.")
            return

        # Sort candidate cells by score (lower score means higher priority).
        candidate_cells.sort(key=lambda cell: cell['score'])
        best_cell = candidate_cells[0]
        self.get_logger().info(
            f"Selected cell at {best_cell['center']} with fire count {best_cell['fire_count']} "
            f"and score {best_cell['score']:.2f}."
        )
        self.send_goal(best_cell['center'][0], best_cell['center'][1])

    def send_goal(self, x, y):
        goal_msg = GoToGoal.Goal()
        goal_msg.x = x
        goal_msg.y = y
        self.get_logger().info(f"Sending goal to cell center at x={x:.2f}, y={y:.2f}")
        if self._action_client.wait_for_server(timeout_sec=5.0):
            send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().error("Action server not available.")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return
        self.get_logger().info("Goal accepted. Waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.goal_reached:
            self.get_logger().info("Goal reached successfully.")
        else:
            self.get_logger().info("Failed to reach goal.")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback: Current position ({feedback.current_x}, {feedback.current_y}), Distance: {feedback.distance}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = FireCellGoalClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
