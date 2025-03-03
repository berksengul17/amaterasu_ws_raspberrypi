import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import copy

class FireSimulatorV2(Node):
    def __init__(self):
        super().__init__('fire_simulator_v2')
        # Parameters for grid dimensions and simulation timing.
        self.declare_parameter("grid_rows", 5)
        self.declare_parameter("grid_cols", 5)
        self.declare_parameter("simulation_period", 1.0)  # seconds
        self.declare_parameter("growth_scaling", 1.0)  # scaling factor for fire growth

        self.grid_rows = self.get_parameter("grid_rows").value
        self.grid_cols = self.get_parameter("grid_cols").value
        self.simulation_period = self.get_parameter("simulation_period").value
        self.growth_scaling = self.get_parameter("growth_scaling").value

        # Define a capacity map.
        # Each cell has a capacity representing the threshold fire size before ignition spreads.
        # (Cells with capacity 0 are non-flammable.)
        self.capacity_map = [
            [100, 100, 100, 100, 100],
            [100, 100, 100, 100, 100],
            [100, 100, 100, 100, 100],
            [100, 100, 100, 100, 100],
            [100, 100, 100, 100, 100]
        ]
        # The high capacity ensures that initial detections don’t immediately cause a spread.

        # Internal simulation grid for "fire size" (accumulated fire intensity per cell).
        # This value increases faster in cells with more detected balls.
        self.fire_size_grid = [[0.0 for _ in range(self.grid_cols)] for _ in range(self.grid_rows)]

        # Latest detection grid (fire counts from detection node).
        # It is expected to be published on a topic as a flat list (row-major order).
        self.detection_grid = [[0 for _ in range(self.grid_cols)] for _ in range(self.grid_rows)]

        # Subscriber to get detection data (fire counts per cell) from your detection node.
        self.detection_sub = self.create_subscription(
            Int32MultiArray,
            "/grid/fire_count",
            self.detection_callback,
            10
        )

        # Publisher for the updated simulation grid (fire sizes).
        self.sim_pub = self.create_publisher(
            Float32MultiArray,
            "/grid/fire_simulation",
            10
        )

        # Timer to run simulation steps periodically.
        self.timer = self.create_timer(self.simulation_period, self.simulation_step)

        self.get_logger().info("Fire simulator V2 node initialized.")

    def detection_callback(self, msg: Int32MultiArray):
        """
        Update the detection grid with the latest fire counts from the detection node.
        These counts represent the number of balls detected in each grid cell.
        """
        if len(msg.data) != self.grid_rows * self.grid_cols:
            self.get_logger().error("Received grid data size does not match expected grid dimensions.")
            return
    
        new_detection = []
        for i in range(self.grid_rows):
            row = msg.data[i * self.grid_cols:(i + 1) * self.grid_cols]
            new_detection.append(list(row))
        self.detection_grid = new_detection
        self.get_logger().info(f"Detection grid updated: {self.detection_grid}")

    def simulation_step(self):
        """
        Perform one simulation step:
        1. Increase fire size in each cell proportionally to the detection count.
           (A cell with 3 detected balls increases 3 times faster than one with 1 ball.)
        2. For cells that exceed their capacity, ignite all 8 neighboring cells
           (i.e. add one unit of fire size to each flammable neighbor).
        """
        # Copy the current fire size grid to update it.
        new_fire_grid = copy.deepcopy(self.fire_size_grid)

        # Step 1: Increase fire size based on detection.
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                # Increase is proportional to the number of detected balls and the scaling factor.
                increase = self.detection_grid[r][c] * self.growth_scaling
                new_fire_grid[r][c] += increase

        # Step 2: Check for cells where fire size exceeds capacity.
        # For each such cell, add one unit of fire size to all 8 neighboring cells.
        neighbor_offsets = [(-1, -1), (-1, 0), (-1, 1),
                            (0, -1),           (0, 1),
                            (1, -1),  (1, 0),  (1, 1)]
        for r in range(self.grid_rows):
            for c in range(self.grid_cols):
                # Only consider flammable cells (capacity > 0)
                if self.capacity_map[r][c] > 0 and new_fire_grid[r][c] > self.capacity_map[r][c]:
                    for dr, dc in neighbor_offsets:
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < self.grid_rows and 0 <= nc < self.grid_cols:
                            if self.capacity_map[nr][nc] > 0:
                                new_fire_grid[nr][nc] += 1

        # Update the internal fire size grid.
        self.fire_size_grid = new_fire_grid

        # Publish the updated simulation grid.
        flat_grid = []
        for row in self.fire_size_grid:
            flat_grid.extend(row)
        msg = Float32MultiArray(data=flat_grid)
        self.sim_pub.publish(msg)
        self.get_logger().info(f"Simulation step complete. Fire size grid: {self.fire_size_grid}")

def main(args=None):
    rclpy.init(args=args)
    node = FireSimulatorV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
