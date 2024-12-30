
import smbus
import math
import time
import rclpy
from collections import deque
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

# Define I2C address and bus
I2C_ADDRESS = 0x0D
bus = smbus.SMBus(1)

# Constants for configuration
MODE_CONTINUOUS = 0x01
OUTPUT_DATA_RATE_200HZ = 0x0C
FULL_SCALE_8G = 0x10
OVERSAMPLE_512 = 0x00

class QMC5883LCompass(Node):
    def __init__(self):
        super().__init__("magnetometer")

        self.offset = [0, 0, 0]
        self.scale = [1, 1, 1]
        self.magnetic_declination = 0.09744  # Magnetic declination adjustment
        self.heading_history = deque(maxlen = 20)

        self.publisher = self.create_publisher(MagneticField, "/magnetometer/smoothed", 10)
        self.timer = self.create_timer(0.1, self.smoothed_heading)

    def add_heading(self, heading):
        """Add a new heading to the history."""
        self.heading_history.append(heading)

    def smoothed_heading(self):
        heading = self.get_azimuth()
        self.heading_history.append(heading)

        magnetic_field = MagneticField()
        magnetic_field.header.stamp = self.get_clock().now().to_msg()
        magnetic_field.magnetic_field.x = sum(self.heading_history) / len(self.heading_history)
        magnetic_field.magnetic_field.y = 0.0
        magnetic_field.magnetic_field.z = 0.0
        
        self.publisher.publish(magnetic_field)
        
    def write_register(self, reg, value):
        bus.write_byte_data(I2C_ADDRESS, reg, value)

    def init(self):
        self.write_register(0x0B, 0x01)  # Soft reset
        self.set_mode(MODE_CONTINUOUS, OUTPUT_DATA_RATE_200HZ, FULL_SCALE_8G, OVERSAMPLE_512)

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
        print("Calibrating... Move the sensor in all directions.")
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
        print(f"Calibration complete: Offsets={self.offset}, Scales={self.scale}")

    def normalize(self, raw_values):
        return [(raw_values[i] - self.offset[i]) / self.scale[i] for i in range(3)]

    def set_magnetic_declination(self, degrees, minutes):
        self.magnetic_declination = degrees + minutes / 60.0


    def get_azimuth(self):
        x, y, _ = self.read_raw_data()
        x = (x - self.offset[0]) / self.scale[0]
        y = (y - self.offset[1]) / self.scale[1]

        # Calculate heading
        heading = math.atan2(y, x) * (180 / math.pi)
        heading += self.magnetic_declination
        if heading < 0:
            heading += 360

        # Add to history for smoothing
        self.add_heading(heading)
        return heading

    def get_direction_name(self, azimuth):
        directions = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                      "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
        index = int((azimuth + 11.25) / 22.5) % 16
        return directions[index]


def main(args=None):
    rclpy.init(args=args)

    compass = QMC5883LCompass()
    compass.init()
    compass.calibrate(duration=15)
    compass.set_magnetic_declination(5, 34)  # Adjust declination for your location

    rclpy.spin(compass)

    compass.destroy_node()
    rclpy.shutdown()



# Example Usage
if __name__ == "__main__":
    main()
    # compass = QMC5883LCompass()
    # compass.init()
    # compass.calibrate(duration=15)
    # compass.set_magnetic_declination(5, 34)  # Adjust declination for your location

    # while True:
    #     raw_azimuth = compass.get_azimuth()  # Get raw azimuth and update the history
    #     smooth_azimuth = compass.smoothed_heading()  # Calculate smoothed azimuth

    #     if smooth_azimuth is not None:
    #         print(f"Raw Azimuth: {raw_azimuth:.2f}°, Smoothed Azimuth: {smooth_azimuth:.2f}°")
    #     else:
    #         print("Calculating smoothing...")

    #     time.sleep(0.5)