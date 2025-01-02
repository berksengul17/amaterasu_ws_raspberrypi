import rclpy
import math
import time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Imu, CompressedImage, MagneticField
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import RPi.GPIO as GPIO

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # # GPIO setup
        GPIO.setmode(GPIO.BCM)
        self.EnaA = 17
        self.In1A = 27
        self.In2A = 22
        self.EnaB = 23
        self.In1B = 24
        self.In2B = 25

        GPIO.setup(self.EnaA,GPIO.OUT)
        GPIO.setup(self.In1A,GPIO.OUT)
        GPIO.setup(self.In2A,GPIO.OUT)
        GPIO.setup(self.EnaB,GPIO.OUT)
        GPIO.setup(self.In1B,GPIO.OUT)
        GPIO.setup(self.In2B,GPIO.OUT)

        self.pwmA = GPIO.PWM(self.EnaA, 100)
        self.pwmA.start(0)
        self.pwmB = GPIO.PWM(self.EnaB, 100)
        self.pwmB.start(0)

        self.stop_motors()
        self.tf_broadcaster = TransformBroadcaster(self)


        # Subscribe to bounding box topic and IMU topic
        self.ball_bounding_box_sub = self.create_subscription(Float32MultiArray, '/ball/bounding_box', self.bounding_box_callback, 10)
        self.imu_mag_sub = self.create_subscription(Float32, "/imu/fused", self.imu_fused_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ## şu an bunlarla bir şey yapamıyoz
        #self.create_subscription(Float32MultiArray, "/robot/bounding_box", self.update_position, 10)
        #self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        #self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        
        # self.publisher = self.create_publisher(Pose, "/robot/position", 10)
        self.marker_publisher = self.create_publisher(MarkerArray, "/ball_markers", 10)
        self.path_publisher = self.create_publisher(Path, "/robot/trajectory", 10)
        self.path = Path()
        self.path.header.frame_id = "map"
        
        # Robot state
        self.ball_positions = []  # List of ball positions (calculated global coordinates)
        self.robot_position = (0.01, 0.01)  # Robot starting position (x, y)
        self.robot_yaw = 0.0  # Current yaw of the robot

        self.grid_width = 128  # Platform width in meters
        self.grid_height = 128  # Platform height in meters
        self.camera_resolution = (256, 256)  # Camera resolution (pixels)

        # Motor speed adjustment for straight movement
        self.right_motor_speed = 80  # Adjust this value for your motor setup
        self.left_motor_speed = 80  # Adjust this value for your motor setup

        self.tf_timer = self.create_timer(0.1, self.publish_transform)  # Broadcast every 100ms

        self.active_marker_ids = set()

        self.current_heading = 0

    def imu_fused_callback(self, msg):
        self.current_heading = msg.data
        self.get_logger().info(f"Imu fused: {self.current_heading:.2f}")

    def bounding_box_callback(self, msg):
            """Convert bounding boxes to global coordinates dynamically."""
            self.get_logger().info(f"Ball bounding box: {msg}")
            self.ball_positions = []  # Reset positions
            marker_array = MarkerArray()
            current_marker_ids = set()

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
                    transform = self.tf_buffer.lookup_transform(
                        "map",  # Target frame
                        "camera_link",  # Source frame
                        self.get_clock().now()
                    )
                    point_in_global = self.transform_point(camera_x, camera_y, camera_z, transform)
                    self.ball_positions.append((point_in_global.x, point_in_global.y))

                    # Create marker for visualization
                    marker = self.create_marker(point_in_global.x, point_in_global.y)
                    marker_array.markers.append(marker)
                    current_marker_ids.add(marker.id)

                except tf2_ros.LookupException:
                    self.get_logger().error("Transform unavailable for camera frame!")

            self.marker_publisher.publish(marker_array)
            self.get_logger().info(f"Ball positions: {self.ball_positions}")

    #yeni kod buraya eklendi
    @staticmethod
    def transform_point(x, y, z, transform):
        """Transform a point using a TransformStamped."""
        point = PoseStamped()
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = z
        transformed_point = tf2_geometry_msgs.do_transform_pose(point, transform)
        return transformed_point.pose.position

    def update_orientation(self, msg):
        """Update the robot's orientation from IMU data."""
        # Convert quaternion to Euler angles
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        roll, pitch, yaw = tf_transformations.euler_from_quaternion([x, y, z, w])
        self.current_orientation = (roll, pitch, yaw)
        self.get_logger().info(f"Orientation updated: Roll={roll}, Pitch={pitch}, Yaw={yaw}")

    def update_heading(self, msg):
        """Update the robot's heading from Magnetometer data."""
        self.current_heading = msg.magnetic_field.x
        self.get_logger().info(f"Heading updated: {self.current_heading:.2f}°")

    def move_to_ball(self):
        """Use heading and orientation to navigate to the ball."""
        if not self.ball_positions:
            self.get_logger().info("No balls to move to.")
            return

        target_ball = self.ball_positions[0]  # Closest ball logic remains the same
        yaw_error = self.calculate_yaw_error(target_ball)

        # Use IMU orientation for smoother yaw adjustments
        if abs(yaw_error) > 0.1:
            self.adjust_yaw(yaw_error)
        else:
            self.move_forward()

    def calculate_yaw_error(self, target_pos):
        """Calculate yaw error between robot and the target."""
        dx = target_pos[0] - self.robot_position[0]
        dy = target_pos[1] - self.robot_position[1]
        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.robot_yaw
        return (yaw_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]

    def move_to_ball(self):
        """Move the robot to the closest ball."""
        if not self.ball_positions:
            self.get_logger().info("No more balls to extinguish.")
            self.stop_motors()
            return

        # Sort balls by distance
        self.ball_positions.sort(key=lambda pos: self.distance_to_ball(pos))
        target_ball = self.ball_positions[0]

        # Align yaw
        yaw_error = self.calculate_yaw_error(target_ball)
        if abs(yaw_error) > 0.1:  # Adjust yaw if error is significant
            self.adjust_yaw(yaw_error)
        else:
            # Move forward if aligned
            distance = self.distance_to_ball(target_ball)
            if distance > 10:  # Stop if close enough to the ball
                self.move_forward()
            else:
                self.extinguish_ball(target_ball)

    def adjust_yaw(self, yaw_error):
        """Adjust the robot's yaw using PWM for smoother control."""
        # Duration of turning
        turn_duration = 0.2  # Adjust based on your robot's turn speed
        angular_speed = 0.5  # Estimated angular speed in rad/s (calibrated for your motors)

        # Calculate the change in yaw
        delta_yaw = angular_speed * turn_duration if yaw_error > 0 else -angular_speed * turn_duration

        # Update the robot's yaw
        self.robot_yaw += delta_yaw
        self.robot_yaw = (self.robot_yaw + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]

        self.get_logger().info(f"Updated yaw: {self.robot_yaw}")

        self.get_logger().info(f"Adjusting yaw: {yaw_error}")
        if yaw_error > 0:
            # Turn right
            self.right_wheel_pwm.ChangeDutyCycle(60)  # Slow down right wheel
            self.left_wheel_pwm.ChangeDutyCycle(80)  # Speed up left wheel
        else:
            # Turn left
            self.right_wheel_pwm.ChangeDutyCycle(80)  # Speed up right wheel
            self.left_wheel_pwm.ChangeDutyCycle(60)  # Slow down left wheel

        time.sleep(0.2)  # Adjust based on yaw error
        self.stop_motors()

    def move_forward(self):
        """Move the robot forward with calibrated speeds."""
        move_duration = 0.2  # Adjust duration as needed
        linear_speed = 10  # Estimated linear speed in m/s (calibrated for your motors)

        # Calculate the distance moved
        distance = linear_speed * move_duration

        # Update the robot's position based on the current yaw
        self.robot_position = (
            self.robot_position[0] + distance * math.cos(self.robot_yaw),
            self.robot_position[1] + distance * math.sin(self.robot_yaw),
        )

        # Create a PoseStamped message for the new position
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.robot_position[0]
        pose.pose.position.y = self.robot_position[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(self.robot_yaw / 2)
        pose.pose.orientation.w = math.cos(self.robot_yaw / 2)

        # Append to the path
        self.path.poses.append(pose)

        # Publish the updated path
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path)

        # Publish the robot's TF
        #self.publish_transform()

        # Log the updated position
        self.get_logger().info(f"Updated position: {self.robot_position}")

        GPIO.output(self.right_wheel_backward, False)  # Ensure backward is off
        GPIO.output(self.left_wheel_backward, False)

        self.right_wheel_pwm.ChangeDutyCycle(self.right_motor_speed)
        self.left_wheel_pwm.ChangeDutyCycle(self.left_motor_speed)

        # Simulate motion
        time.sleep(0.2)  # Adjust duration as needed
        self.stop_motors()

    def publish_transform(self):
        """Publish the robot's transform for visualization in RViz2."""
        t = TransformStamped()

        # Set the frame names
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"  # Reference frame
        t.child_frame_id = "base_link"  # Robot's frame

        # Set the robot's position
        t.transform.translation.x = self.robot_position[0]
        t.transform.translation.y = self.robot_position[1]
        t.transform.translation.z = 0.0  # Assume 2D motion

        # Set the robot's orientation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.robot_yaw / 2)
        t.transform.rotation.w = math.cos(self.robot_yaw / 2)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    # def move_forward(self):
    #     """Move the robot forward with calibrated speeds."""
    #     # Duration of forward motion
    #     move_duration = 0.2  # Adjust duration as needed
    #     linear_speed = 10  # Estimated linear speed in m/s (calibrated for your motors)

    #     # Calculate the distance moved
    #     distance = linear_speed * move_duration

    #     # Update the robot's position based on the current yaw
    #     self.robot_position = (
    #         self.robot_position[0] + distance * math.cos(self.robot_yaw),
    #         self.robot_position[1] + distance * math.sin(self.robot_yaw),
    #     )

    #     pos = Pose()
    #     pos.position.x = self.robot_position[0]
    #     pos.position.y = self.robot_position[1]

    #     self.publisher.publish(pos)

    #     self.get_logger().info(f"Updated position: {self.robot_position}")

    #     # GPIO.output(self.right_wheel_backward, False)  # Ensure backward is off
    #     # GPIO.output(self.left_wheel_backward, False)

    #     # self.right_wheel_pwm.ChangeDutyCycle(self.right_motor_speed)
    #     # self.left_wheel_pwm.ChangeDutyCycle(self.left_motor_speed)

    #     time.sleep(0.2)  # Adjust duration as needed
    #     self.stop_motors()

    def extinguish_ball(self, ball):
        """Simulate extinguishing a ball."""
        self.get_logger().info(f"Extinguishing ball at: {ball}")
        self.stop_motors()
        time.sleep(2)  # Simulate extinguishing process
        self.ball_positions.remove(ball)

    def stop_motors(self):
        """Stop all motors."""
        pass
        # self.right_wheel_pwm.ChangeDutyCycle(0)
        # self.left_wheel_pwm.ChangeDutyCycle(0)

    def distance_to_ball(self, ball_pos):
        """Calculate distance to a ball."""
        dx = ball_pos[0] - self.robot_position[0]
        dy = ball_pos[1] - self.robot_position[1]
        return math.sqrt(dx**2 + dy**2)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
#        rclpy.spin(robot_controller)
        while rclpy.ok():
            #robot_controller.move_to_ball()
            rclpy.spin_once(robot_controller, timeout_sec=0.1)
    except KeyboardInterrupt:
        robot_controller.get_logger().info("Shutting down robot controller...")
    finally:
        robot_controller.stop_motors()
        # GPIO.cleanup()
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
