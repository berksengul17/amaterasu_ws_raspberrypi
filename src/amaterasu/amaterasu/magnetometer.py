import smbus
import math
import time
import rclpy
from collections import deque
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

# Define I2C address and bus
I2C_ADDRESS = 0x0D
bus = smbus.SMBus(1)

# Constants for configuration
MODE_CONTINUOUS = 0x01
OUTPUT_DATA_RATE_200HZ = 0x0C
FULL_SCALE_8G = 0x10
OVERSAMPLE_512 = 0x00

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def transform_to_map_frame(x_raw, y_raw, z_raw, theta):
    # Define the rotation matrix
    R_map_mag = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

    # Magnetometer readings as a vector
    v_mag = np.array([x_raw, y_raw, z_raw])

    # Transform to the map frame
    v_map = np.dot(R_map_mag, v_mag[:3])
    return v_map

class QMC5883LCompass(Node):
    def __init__(self):
        super().__init__("magnetometer")

        self.offset = [0, 0, 0]
        self.scale = [1, 1, 1]
        self.magnetic_declination = 0.0  # Magnetic declination adjustment
        self.heading_history = deque(maxlen = 20)

        self.publisher = self.create_publisher(Float32, "/magnetometer/smoothed", 10)
        self.timer = self.create_timer(0.1, self.publish_heading)

        self.tf_broadcaster = TransformBroadcaster(self)

    def init(self):
        self.write_register(0x0B, 0x01)  # Soft reset
        self.set_mode(MODE_CONTINUOUS, OUTPUT_DATA_RATE_200HZ, FULL_SCALE_8G, OVERSAMPLE_512)
        
    def write_register(self, reg, value):
        bus.write_byte_data(I2C_ADDRESS, reg, value)

    def set_mode(self, mode, odr, rng, osr):
        self.write_register(0x09, mode | odr | rng | osr)

    def read_raw_data(self):
        raw_data = bus.read_i2c_block_data(I2C_ADDRESS, 0x00, 6)
        x = self._convert(raw_data[0], raw_data[1])
        y = self._convert(raw_data[2], raw_data[3])
        z = self._convert(raw_data[4], raw_data[5])
        return x, y, z

    def _convert(self, low, high):
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def calibrate(self, duration=10):
        self.get_logger().info("Calibrating... Move the sensor in all directions.")
        start_time = time.time()
        min_vals = [float('inf')] * 3
        max_vals = [float('-inf')] * 3

        while time.time() - start_time < duration:
            x, y, z = self.read_raw_data()
            min_vals[0] = min(min_vals[0], x)
            max_vals[0] = max(max_vals[0], x)
            min_vals[1] = min(min_vals[1], y)
            max_vals[1] = max(max_vals[1], y)
            min_vals[2] = min(min_vals[2], z)
            max_vals[2] = max(max_vals[2], z)

        self.offset = [(max_vals[i] + min_vals[i]) / 2 for i in range(3)]
        self.scale = [(max_vals[i] - min_vals[i]) / 2 for i in range(3)]
        self.get_logger().info(f"Calibration complete: Offsets={self.offset}, Scales={self.scale}")

    def set_magnetic_declination(self, degrees, minutes):
        self.magnetic_declination = degrees + minutes / 60.0

    def get_azimuth(self):
        x, y, z = self.read_raw_data()
        x = (x - self.offset[0]) / self.scale[0]
        y = (y - self.offset[1]) / self.scale[1]

        # Calculate heading in the map frame
        heading = math.atan2(y, x) * (180 / math.pi) # degrees
        if heading < 0:
                heading += 360.0
        heading += self.magnetic_declination

        if heading < 0.0:
            heading += 360.0
        elif heading >= 360.0:
            heading -= 360.0

        # Add to history for smoothing
        self.add_heading(heading)
        self.publish_tf(heading)

        return heading
    
    def add_heading(self, heading):
        """Add a new heading to the history."""
        self.heading_history.append(heading)

    def publish_heading(self):
        heading = self.get_azimuth()
        self.heading_history.append(heading)

        magnetic_field = Float32()
        magnetic_field.data = heading

        # self.get_logger().info(f"Yaw: {magnetic_field.data:.2f}")
        
        self.publisher.publish(magnetic_field)

    def publish_tf(self, z):
        """Provide the magnetometer's transform as a TransformStamped."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"  # Match robot's frame
        t.child_frame_id = "magnetometer"

        quaternion = quaternion_from_euler(0, 0, math.radians(z))
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    compass = QMC5883LCompass()
    compass.init()
    compass.calibrate(duration=15)
    compass.set_magnetic_declination(5, 34)  # Adjust declination for your location
    compass.get_logger().info(f"Magnetic declination: {compass.magnetic_declination}")
    rclpy.spin(compass)

    compass.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()