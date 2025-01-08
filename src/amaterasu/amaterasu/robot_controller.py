import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import Point
from tf_transformations import quaternion_from_euler

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Platform and camera settings
        self.grid_width = 128  # Platform width (meters)
        self.grid_height = 128  # Platform height (meters)
        self.camera_resolution = (256, 256)  # Camera resolution (pixels)

        # Ball positions
        self.ball_positions = [[2.0, 2.0]]

        self.last_time = time.time()

        # Subscriptions
        self.create_subscription(
            Float32,
            "/imu/fused",
            self.imu_fused_callback,
            10
        )
        self.create_subscription(
            Float32MultiArray,
            '/ball/bounding_box',
            self.bounding_box_callback,
            10
        )
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.publish_robot_pose, 
            10
        )
        self.create_subscription(
            Float32MultiArray,
            '/robot/initial_position',
            self.set_initial_robot_position,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, "/robot/trajectory", 10)
        self.marker_publisher = self.create_publisher(MarkerArray, "/ball_markers", 10)

        # TF2 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot state
        self.robot_position = None
        self.robot_yaw = 0.0  # Current yaw from IMU
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Path initialization
        self.path = Path()
        self.path.header.frame_id = "map"

        # Timer for movement logic
        self.create_timer(0.1, self.move_to_closest_ball)
        #self.create_timer(0.1, self.test)

    def test(self):
        marker_array = MarkerArray()
        marker = self.create_marker(0.0, 0.0)
        marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

    def imu_fused_callback(self, msg):
        """
        Update the robot's yaw from IMU and magnetometer data.
        """
        self.robot_yaw = msg.data

    def set_initial_robot_position(self, msg):
        """
        Set the initial position of the robot using the bounding box received.
        """
        if self.robot_position is None:
            # Convert bounding box [x1, y1, x2, y2] to robot position (center of the bounding box)
            x1, y1, x2, y2 = msg.data
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2

            # Scale coordinates to fit grid dimensions (adjust as needed)
            self.robot_position = [
                (center_x / self.grid_width) * self.grid_width,
                (center_y / self.grid_height) * self.grid_height
            ]
            self.get_logger().info(f"Robot's initial position set to: {self.robot_position}")

    def bounding_box_callback(self, msg):
        """
        Process bounding box data from the camera and transform to global coordinates.
        """
        self.ball_positions = []  # Clear previous positions
        marker_array = MarkerArray()

        for i in range(0, len(msg.data), 4):
            x1, y1, x2, y2 = msg.data[i:i + 4]
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2

            # Convert to camera frame coordinates
            camera_x = (center_x / self.camera_resolution[0]) * self.grid_width
            camera_y = (center_y / self.camera_resolution[1]) * self.grid_height
            camera_z = 0.0  # Assume objects are on the ground

            # Transform to global coordinates
            try:
                # transform = self.tf_buffer.lookup_transform(
                #     "map",  # Target frame
                #     "camera_link",  # Source frame
                #     self.get_clock().now()
                # )
                # global_position = self.transform_point(camera_x, camera_y, camera_z, transform)
                global_position_y = self.grid_height - camera_y
                self.ball_positions.append((camera_x, global_position_y))

                # Add a marker for visualization
                marker = self.create_marker(camera_x, global_position_y)
                marker_array.markers.append(marker)

            except Exception as e:
                self.get_logger().error(f"Transform failed: {e}")

        # Publish markers
        self.marker_publisher.publish(marker_array)
        # self.get_logger().info(f"Ball positions in global frame: {self.ball_positions}")

    def move_to_closest_ball(self):
        """
        Move the robot toward the closest ball.
        """
        if not self.robot_position:
            self.get_logger().warning("Robot position not set. Waiting for initial position...")
            return

        if not self.ball_positions:
            self.get_logger().info("No balls detected to move toward.")
            return

        # Sort balls by distance
        self.ball_positions.sort(key=lambda pos: self.distance_to_ball(pos))
        target_ball = self.ball_positions[0]

        # Calculate yaw error
        yaw_error = self.calculate_yaw_error(target_ball)

        twist_msg = Twist()

        if abs(yaw_error) > 80:  # If yaw error is significant, adjust yaw
            twist_msg.angular.z = 0.8 if yaw_error > 0 else -0.8
        else:  # Otherwise, move forward
            distance = self.distance_to_ball(target_ball)
            if distance > 1.0:  # Move if not close enough
                twist_msg.linear.x = 5.0
            else:  # Extinguish ball if close
                self.extinguish_ball(target_ball)

        self.get_logger().info(f"Yaw error: {yaw_error}, Angular.z: {twist_msg.angular.z}")

        self.cmd_vel_pub.publish(twist_msg)
        self.publish_robot_pose(twist_msg)

    def calculate_angular_z(self, yaw_error):
        k = 0.04  # Scaling constant, adjust based on robot's behavior
        angular_z = math.copysign(min(0.8, max(0.3, abs(yaw_error) * k)), yaw_error)
        return angular_z

    def calculate_yaw_error(self, target_pos):
        """
        Calculate yaw error between robot and the target position.
        """
        dx = target_pos[0] - self.robot_position[0]
        dy = target_pos[1] - self.robot_position[1]
        desired_yaw = math.degrees(math.atan2(dy, dx))
        yaw_error = desired_yaw - self.robot_yaw
        self.get_logger().info(f"Not normalized: {desired_yaw} - {self.robot_yaw} - {yaw_error}")
        return (yaw_error + 180) % (360) - 180  # Normalize to [-π, π]

    def extinguish_ball(self, ball):
        """
        Simulate extinguishing a ball.
        """
        time.sleep(2)  # Simulate extinguishing time
        self.ball_positions.remove(ball)

        # Stop movement after extinguishing
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)

    def distance_to_ball(self, ball_pos):
        """
        Calculate the Euclidean distance to a ball.
        """
        dx = ball_pos[0] - self.robot_position[0]
        dy = ball_pos[1] - self.robot_position[1]
        return math.sqrt(dx**2 + dy**2)   

    def transform_point(self, x, y, z, transform):
        """
        Transform a point from one frame to another using a transform.
        """
        transformed_point = Point()
        transformed_point.x = x + transform.transform.translation.x
        transformed_point.y = y + transform.transform.translation.y
        transformed_point.z = z + transform.transform.translation.z
        return transformed_point

    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

##################################### RViz Codes #####################################
        
    def publish_robot_pose(self, twist: Twist):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_Time = current_time

        self.robot_position[0] += twist.linear.x * dt * math.cos(math.radians(self.robot_yaw))
        self.robot_position[1] += twist.linear.x * dt * math.sin(math.radians(self.robot_yaw))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "robot"
        t.transform.translation.x = self.robot_position[0]
        t.transform.translation.y = self.robot_position[1]
        q = quaternion_from_euler(0, 0, math.radians(self.robot_yaw))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)     


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


##################################### Main Loop #####################################
def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info("Shutting down robot controller...")
    finally:
        robot_controller.stop()
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
