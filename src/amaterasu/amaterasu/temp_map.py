# scripts/flipped_map_publisher.py
#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Quaternion

class FlippedMapPublisher(Node):
    def __init__(self):
        super().__init__('flipped_map_publisher')

        # Transient‐local QoS so late subscribers (RViz, planner) latch the last map
        qos = QoSProfile(depth=1)
        qos.durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(OccupancyGrid, 'map', qos)

        # --- build the grid once ---
        grid = OccupancyGrid()
        grid.header.frame_id = 'camera'

        grid.info = MapMetaData()
        grid.info.resolution = 0.1
        grid.info.width  = 20
        grid.info.height = 20

        # Place the origin at the world coords of the bottom‐right corner:
        origin = Pose()
        origin.position.x = 0.0 #float(grid.info.width * grid.info.resolution) # 2.0 m
        origin.position.y = 0.0
        origin.position.z = 0.0
        # Identity orientation (no built-in rotation)
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        grid.info.origin = origin

        # Initialize all cells free (0)
        grid.data = [0] * (grid.info.width * grid.info.height)

        # Mark world-point (0.5,0.5) occupied:
        def mark(wx, wy):
            # u = map-x index = (origin.x − world.x) / res
            u = int(wx / grid.info.resolution)
            # v = map-y index = (world.y − origin.y) / res
            v = int(wy / grid.info.resolution)
            if 0 <= u < grid.info.width and 0 <= v < grid.info.height:
                idx = v * grid.info.width + u
                grid.data[idx] = 100

        mark(0.8, 0.5)
        mark(1.8, 0.0)

        self.grid = grid
        # publish once immediately, then at 1 Hz
        self.publish_map()
        self.create_timer(1.0, self.publish_map)

    def publish_map(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.grid)


def main(args=None):
    rclpy.init(args=args)
    node = FlippedMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
