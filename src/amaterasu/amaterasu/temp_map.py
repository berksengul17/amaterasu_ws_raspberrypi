# scripts/flipped_map_publisher.py
#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float32MultiArray

class FlippedMapPublisher(Node):
    def __init__(self):
        super().__init__('flipped_map_publisher')

        # Transient‐local QoS so late subscribers (RViz, planner) latch the last map
        qos = QoSProfile(depth=1)
        qos.durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.pub = self.create_publisher(OccupancyGrid, 'map', qos)
        self.fire_sub = self.create_subscription(Float32MultiArray, '/grid/fire_count', self.fire_callback, 10)

        self.width = 10
        self.height = 10
        self.res = 0.2

        # --- build the grid once ---
        grid = OccupancyGrid()
        grid.header.frame_id = 'camera'

        grid.info = MapMetaData()
        grid.info.resolution = self.res
        grid.info.width  = self.width
        grid.info.height = self.height

        half_w = self.width  * self.res / 2.0   # = 1.0
        half_h = self.height * self.res / 2.0   # = 1.0

        # Place the origin at the world coords of the bottom‐right corner:
        origin = Pose()
        origin.position.x = -half_w
        origin.position.y = -half_h
        origin.position.z = 0.0
        # Identity orientation (no built-in rotation)
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        grid.info.origin = origin

        # Initialize all cells free (0)
        grid.data = [0] * (grid.info.width * grid.info.height)

        self.grid = grid
        # publish once immediately, then at 1 Hz
        self.publish_map()
        self.create_timer(1.0, self.publish_map)

    def fire_callback(self, msg: Float32MultiArray):
        self.grid.data = [0] * (self.width * self.height)

        for i, val in enumerate(msg.data):
            if val > 0.5:
                row = i // self.width
                col = i % self.width
                flipped_row = (self.height - 1) - row
                idx = flipped_row * self.width + col
                self.grid.data[idx] = int(val * 20)

        # stamp and publish
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.grid)


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
