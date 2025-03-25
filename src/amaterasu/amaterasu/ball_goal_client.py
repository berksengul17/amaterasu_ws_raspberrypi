import rclpy
import math
import time
import random
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry
from amaterasu_interfaces.action import GoToGoal

class FireCellGoalClient(Node):
    def __init__(self):
        super().__init__('fire_cell_goal_client')
        self._action_client = ActionClient(self, GoToGoal, 'go_to_goal_service')
        
        # Subscribe to grid data for fire counts
        self.fire_count_sub = self.create_subscription(
            Int32MultiArray, "/grid/fire_count", self.fire_count_callback, 10
        )
        # Subscribe to grid data for fuel load (w)
        self.fuel_load_sub = self.create_subscription(
            Float32MultiArray, "/grid/fuel_load", self.fuel_load_callback, 10
        )
        # Subscribe to odometry to get current robot pose
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        
        self.current_pose = [0.0, 0.0]
        self.grid_data = None             # Fire count data (flat list)
        self.fuel_load_data = None        # Fuel load (w) data (flat list)
        self.last_update_time = 0.0
        self.update_interval = 1.0  # seconds

        # Parameters for grid dimensions and platform size (real-world)
        self.declare_parameter("grid_rows", 10)
        self.declare_parameter("grid_cols", 10)
        self.declare_parameter("platform_width", 2.10)
        self.declare_parameter("platform_height", 1.54)
        
        # Extra cell parameters (randomized for each cell except for fuel load w):
        #  - relative_humidity (as a percentage, e.g., 5.0 to 100.0)
        #  - flammability (0.1 to 1.0)
        #  - wind_speed in m/s (0.0 to 50.0)  --> wind speed unit is m/s
        #  - H (Heat yield) in kJ/kg (16000.0 to 20000.0)
        #  - w (Fuel load) in kg/m² (now provided by a separate subscription, 0.5 to 2.0)
        #  - r (Rate of Spread) in m/s (not declared since fire_count will be used as r, 0 to 1.0)
        #  Note: Fuel load (w) is not randomized now; it will be received via subscription.
        self.declare_parameter("relative_humidity_min", 5.0)
        self.declare_parameter("relative_humidity_max", 100.0)
        self.declare_parameter("flammability_min", 0.1)
        self.declare_parameter("flammability_max", 1.0)
        self.declare_parameter("wind_speed_min", 0.0)    # in m/s
        self.declare_parameter("wind_speed_max", 50.0)
        
        # New parameters for fire intensity components (H)
        self.declare_parameter("H_min", 16000.0)   # kJ/kg
        self.declare_parameter("H_max", 20000.0)
        # Fuel load (w) will be provided from the subscription; no default values declared.
        # Note: r is not declared since fire_count will be used as r and the value range will be 0 to 1.

        # Global parameters for weighted algorithm:
        self.declare_parameter("recent_rainfall_mm", 0.0)    # recent rainfall in mm
        self.declare_parameter("rainfall_threshold", 1000.0)   # threshold in mm
        self.declare_parameter("temperature", 25.0)            # temperature in Celsius
        self.declare_parameter("temperature_threshold", 30.0)  # temperature threshold in Celsius

        # Flag and storage for current goal
        self.goal_active = False
        self.current_goal = [0.0, 0.0]

        # Initialize extra parameters for every cell in the grid.
        self.cell_params = None
        self.initialize_cell_parameters()
        self.log_initial_cell_parameters()

        self.get_logger().info("Fire Cell Goal Client initialized.")

    def odom_callback(self, odom_msg):
        # Update current robot pose.
        self.current_pose[0] = odom_msg.pose.pose.position.x
        self.current_pose[1] = odom_msg.pose.pose.position.y

    def fire_count_callback(self, msg: Int32MultiArray):
        # Store fire count data (flat list in row-major order)
        self.grid_data = msg.data
        self.try_process_grid()

    def fuel_load_callback(self, msg: Float32MultiArray):
        # Store fuel load (w) data (flat list in row-major order)
        self.fuel_load_data = msg.data
        self.try_process_grid()

    def try_process_grid(self):
        # Process grid if both fire count and fuel load data are available.
        if self.grid_data is not None and self.fuel_load_data is not None:
            current_time = time.time()
            if current_time - self.last_update_time >= self.update_interval:
                self.last_update_time = current_time
                self.process_grid_and_send_goal()

    def initialize_cell_parameters(self):
        """Assign unique extra parameters to every cell (except fuel load w)."""
        grid_rows = self.get_parameter("grid_rows").value
        grid_cols = self.get_parameter("grid_cols").value

        relative_humidity_min = self.get_parameter("relative_humidity_min").value
        relative_humidity_max = self.get_parameter("relative_humidity_max").value
        flammability_min = self.get_parameter("flammability_min").value
        flammability_max = self.get_parameter("flammability_max").value
        wind_speed_min = self.get_parameter("wind_speed_min").value      # in m/s
        wind_speed_max = self.get_parameter("wind_speed_max").value      

        H_min = self.get_parameter("H_min").value   # kJ/kg
        H_max = self.get_parameter("H_max").value   

        # Create a 2D list (grid) to store extra parameters for each cell.
        self.cell_params = []
        for row_i in range(grid_rows):
            row_params = []
            for col_i in range(grid_cols):
                params = {
                    'relative_humidity': random.uniform(relative_humidity_min, relative_humidity_max),
                    'flammability': random.uniform(flammability_min, flammability_max),
                    'wind_speed': random.uniform(wind_speed_min, wind_speed_max),  # in m/s
                    'H': random.uniform(H_min, H_max)  # kJ/kg
                    # Note: Fuel load (w) will be obtained via subscription.
                }
                row_params.append(params)
            self.cell_params.append(row_params)

    def log_initial_cell_parameters(self):
        """Persistently log initial parameters (except fuel load) to a file for debugging."""
        try:
            with open("initial_cell_parameters.log", "w") as f:
                for r, row in enumerate(self.cell_params):
                    for c, params in enumerate(row):
                        f.write(f"Cell ({r}, {c}): "
                                f"Relative Humidity = {params['relative_humidity']:.2f}%, "
                                f"Flammability = {params['flammability']:.2f}, "
                                f"Wind Speed = {params['wind_speed']:.2f} m/s, "
                                f"H = {params['H']:.2f} kJ/kg\n")
            self.get_logger().info("Initial cell parameters logged to 'initial_cell_parameters.log'.")
        except Exception as e:
            self.get_logger().error(f"Failed to log initial cell parameters: {e}")

    def log_scoring_parameters(self, candidate_cells, rainfall_factor, temperature_factor):
        """Log all variables affecting score to a file for review."""
        try:
            with open("scoring_parameters.log", "w") as f:
                f.write("Scoring Parameters for Candidate Cells:\n")
                for cell in candidate_cells:
                    f.write(f"Cell at {cell['center']}:\n")
                    f.write(f"  fire_count (r): {cell['fire_count']}\n")
                    f.write(f"  Fuel Load (w): {cell['w']:.2f} kg/m²\n")
                    f.write(f"  Distance: {cell['distance']:.2f}\n")
                    f.write(f"  Relative Humidity: {cell['relative_humidity']:.2f}%\n")
                    f.write(f"  Flammability: {cell['flammability']:.2f}\n")
                    f.write(f"  Wind Speed: {cell['wind_speed']:.2f} m/s\n")
                    f.write(f"  Fire Intensity: {cell['fire_intensity']:.2f}\n")
                    f.write(f"  Score: {cell['score']:.2f}\n")
                    f.write("\n")
                f.write(f"Global Rainfall Factor: {rainfall_factor}\n")
                f.write(f"Global Temperature Factor: {temperature_factor}\n")
            self.get_logger().info("Scoring parameters logged to 'scoring_parameters.log'.")
        except Exception as e:
            self.get_logger().error(f"Failed to log scoring parameters: {e}")

    def process_grid_and_send_goal(self):
        if self.grid_data is None or self.fuel_load_data is None:
            return

        grid_rows = self.get_parameter("grid_rows").value
        grid_cols = self.get_parameter("grid_cols").value
        platform_width = self.get_parameter("platform_width").value
        platform_height = self.get_parameter("platform_height").value

        cell_width = platform_width / grid_cols
        cell_height = platform_height / grid_rows

        candidate_cells = []
        epsilon = 1e-6  # To avoid division by zero

        # Gather candidate cells with nonzero fire count.
        for r in range(grid_rows):
            for c in range(grid_cols):
                index = r * grid_cols + c
                fire_count = self.grid_data[index]
                if fire_count > 0:
                    center_x = (c + 0.5) * cell_width
                    center_y = (r + 0.5) * cell_height
                    distance = math.sqrt((center_x - self.current_pose[0])**2 +
                                         (center_y - self.current_pose[1])**2)
                    extra_params = self.cell_params[r][c]
                    # Retrieve fuel load (w) from the subscribed data.
                    w_value = self.fuel_load_data[index]
                    # Use fire_count as the r value in the fire intensity formula:
                    # Fire intensity I = H * w * fire_count
                    fire_intensity = extra_params['H'] * w_value * fire_count
                    candidate_cells.append({
                        'row': r,
                        'col': c,
                        'center': (center_x, center_y),
                        'fire_count': fire_count,
                        'w': w_value,
                        'distance': distance,
                        'relative_humidity': extra_params['relative_humidity'],
                        'flammability': extra_params['flammability'],
                        'wind_speed': extra_params['wind_speed'],   # in m/s
                        'fire_intensity': fire_intensity
                    })

        if not candidate_cells:
            self.get_logger().info("No active fire cells detected.")
            return

        # Calculate maximum values for normalization across candidate cells.
        max_distance            = max(cell['distance'] for cell in candidate_cells)
        max_relative_humidity   = max(cell['relative_humidity'] for cell in candidate_cells)
        max_flammability        = max(cell['flammability'] for cell in candidate_cells)
        max_wind_speed          = max(cell['wind_speed'] for cell in candidate_cells)    
        max_fire_intensity      = max(cell['fire_intensity'] for cell in candidate_cells)

        # Define base weights.
        weights = {
            'distance': 0.2,
            'relative_humidity': 0.05,
            'flammability': 0.15,
            'wind_speed': 0.1,      # base weight for wind speed
            'fire_intensity': 0.1,
        }

        # Compute global rainfall factor.
        recent_rainfall_mm = self.get_parameter("recent_rainfall_mm").value
        rainfall_threshold = self.get_parameter("rainfall_threshold").value
        if recent_rainfall_mm > rainfall_threshold:
            rainfall_factor = 0.95  # reduces priority drastically
        else:
            rainfall_factor = 1.0

        # Compute global temperature factor.
        temperature_celsius = self.get_parameter("temperature").value
        temperature_threshold = self.get_parameter("temperature_threshold").value
        if temperature_celsius >= temperature_threshold:
            temperature_factor = 1.3
        else:
            temperature_factor = 1.0

        for cell in candidate_cells:
            normalized_distance         = cell['distance'] / (max_distance + epsilon)
            normalized_relative_humidity = cell['relative_humidity'] / (max_relative_humidity + epsilon)
            normalized_flammability     = cell['flammability'] / (max_flammability + epsilon)
            normalized_wind_speed       = cell['wind_speed'] / (max_wind_speed + epsilon)
            normalized_fire_intensity   = cell['fire_intensity'] / (max_fire_intensity + epsilon)

            # Dynamic adjustment for relative humidity: if below 20, increase weight.
            if cell['relative_humidity'] < 20:
                rel_humidity_weight = 0.1
            else:
                rel_humidity_weight = weights['relative_humidity']

            # Dynamic adjustment for wind speed (m/s):
            wind_speed_weight = weights['wind_speed']
            if 10.8 <= cell['wind_speed'] <= 17.1:
                wind_speed_weight = 0.15
            elif 17.2 <= cell['wind_speed'] <= 24.4:
                wind_speed_weight = 0.20
            elif 24.5 <= cell['wind_speed'] <= 32.6:
                wind_speed_weight = 0.25
            elif cell['wind_speed'] >= 32.7:
                wind_speed_weight = 0.30

            cell['score'] = (
                weights['distance'] * (1 - normalized_distance) +
                weights['flammability'] * normalized_flammability +
                wind_speed_weight * normalized_wind_speed +
                rel_humidity_weight * (1 - normalized_relative_humidity) +
                weights['fire_intensity'] * normalized_fire_intensity
            )
            # Apply the global rainfall and temperature factors.
            cell['score'] *= (rainfall_factor * temperature_factor)

        # Log all variables that affect the score.
        self.log_scoring_parameters(candidate_cells, rainfall_factor, temperature_factor)

        # Sort candidate cells by score in descending order.
        candidate_cells.sort(key=lambda cell: cell['score'], reverse=True)
        best_cell = candidate_cells[0]
        new_goal = best_cell['center']

        # For logging, compute the final dynamic weights for best cell.
        best_ws = best_cell['wind_speed']
        if best_ws < 10.8:
            final_ws_weight = weights['wind_speed']
        elif 10.8 <= best_ws <= 17.1:
            final_ws_weight = 0.15
        elif 17.2 <= best_ws <= 24.4:
            final_ws_weight = 0.20
        elif 24.5 <= best_ws <= 32.6:
            final_ws_weight = 0.25
        else:
            final_ws_weight = 0.30

        if best_cell['relative_humidity'] < 20:
            final_rel_humidity_weight = 0.1
        else:
            final_rel_humidity_weight = weights['relative_humidity']

        self.get_logger().info(
            f"Candidate cell at {new_goal} with relative_humidity {best_cell['relative_humidity']:.2f}% "
            f"(weight {final_rel_humidity_weight}), flammability {best_cell['flammability']:.2f}, "
            f"wind_speed {best_cell['wind_speed']:.2f} m/s (weight {final_ws_weight}), "
            f"fuel_load (w) {best_cell['w']:.2f} kg/m², fire_intensity {best_cell['fire_intensity']:.2f}, "
            f"rainfall_factor {rainfall_factor}, temperature_factor {temperature_factor}, "
            f"and final score {best_cell['score']:.2f}."
        )

        # Goal handling logic.
        if self.goal_active:
            current_goal_distance = math.sqrt((self.current_goal[0] - self.current_pose[0])**2 +
                                              (self.current_goal[1] - self.current_pose[1])**2)
            new_goal_distance = math.sqrt((new_goal[0] - self.current_pose[0])**2 +
                                          (new_goal[1] - self.current_pose[1])**2)
            if new_goal_distance < current_goal_distance * 0.8:
                self.get_logger().info(f"New closer cell detected at {new_goal}. Redirecting goal.")
                self.current_goal = new_goal
                self.send_goal(new_goal[0], new_goal[1])
            else:
                self.get_logger().info("Current goal remains the best candidate.")
        else:
            self.current_goal = new_goal
            self.send_goal(new_goal[0], new_goal[1])

    def send_goal(self, x, y):
        goal_msg = GoToGoal.Goal()
        goal_msg.x = x
        goal_msg.y = y
        self.goal_active = True
        self.get_logger().info(f"Sending goal to cell center at x={x:.2f}, y={y:.2f}")
        if self._action_client.wait_for_server(timeout_sec=5.0):
            send_goal_future = self._action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().error("Action server not available.")
            self.goal_active = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            self.goal_active = False
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
        self.goal_active = False

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
