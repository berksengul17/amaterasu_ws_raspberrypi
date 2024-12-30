import smbus
import rclpy
import time
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler

# MPU-6050 I2C address
MPU6050_ADDR = 0x68

# MPU-6050 Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

class MPU6050(Node):
    def __init__(self, bus=1):
        super().__init__("imu")

        self.bus = smbus.SMBus(bus)
        self.addr = MPU6050_ADDR
        self.accel_offsets = {'x': 0, 'y': 0, 'z': 0}
        self.gyro_offsets = {'x': 0, 'y': 0, 'z': 0}
        self.kalman_roll = {'angle': 0, 'uncertainty': 2 * 2}
        self.kalman_pitch = {'angle': 0, 'uncertainty': 2 * 2}
        self.kalman_yaw = {'angle': 0, 'uncertainty': 2 * 2}

        self.dt = 0.1

        self.publisher = self.create_publisher(Imu, "/imu/raw_data", 10)
        self.timer = self.create_timer(self.dt, self.publish_imu)

    def initialize(self):
        """Initialize the MPU-6050."""
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0)
        time.sleep(0.1)

    def publish_imu(self):
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()

        accel_angles = self.calculate_angles(accel)

        # Apply Kalman filter
        self.kalman_roll = self.kalman_filter(self.kalman_roll, gyro['x'], accel_angles['roll'], self.dt)
        self.kalman_pitch = self.kalman_filter(self.kalman_pitch, gyro['y'], accel_angles['pitch'], self.dt)
        self.kalman_yaw = self.kalman_filter(self.kalman_yaw, gyro['z'], accel_angles['yaw'], self.dt)
        quaternion = quaternion_from_euler(self.kalman_roll['angle'], self.kalman_pitch['angle'], self.kalman_yaw['angle'])

        self.get_logger().info(f"Roll: {self.kalman_roll['angle']:.2f}° Pitch: {self.kalman_pitch['angle']:.2f}° Yaw: {self.kalman_yaw['angle']:.2f}°")
    

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"  # Replace with your IMU frame

        imu.orientation.x = quaternion[0]
        imu.orientation.y = quaternion[1]
        imu.orientation.z = quaternion[2]
        imu.orientation.w = quaternion[3]

        self.publisher.publish(imu)

    def read_raw_data(self, reg):
        """Read two bytes of raw data from the specified register."""
        high = self.bus.read_byte_data(self.addr, reg)
        low = self.bus.read_byte_data(self.addr, reg + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

    def get_accel_data(self):
        """Read and return accelerometer data."""
        accel_x = self.read_raw_data(ACCEL_XOUT_H)
        accel_y = self.read_raw_data(ACCEL_XOUT_H + 2)
        accel_z = self.read_raw_data(ACCEL_XOUT_H + 4)
        accel_scale = 16384.0
        return {
            'x': accel_x / accel_scale - self.accel_offsets['x'],
            'y': accel_y / accel_scale - self.accel_offsets['y'],
            'z': accel_z / accel_scale - self.accel_offsets['z']
        }

    def get_gyro_data(self):
        """Read and return gyroscope data."""
        gyro_x = self.read_raw_data(GYRO_XOUT_H)
        gyro_y = self.read_raw_data(GYRO_XOUT_H + 2)
        gyro_z = self.read_raw_data(GYRO_XOUT_H + 4)
        gyro_scale = 131.0
        return {
            'x': gyro_x / gyro_scale - self.gyro_offsets['x'],
            'y': gyro_y / gyro_scale - self.gyro_offsets['y'],
            'z': gyro_z / gyro_scale - self.gyro_offsets['z']
        }

    def calibrate(self, samples=2000):
        """Calibrate accelerometer and gyroscope."""
        print("Calibrating... Please keep the sensor stationary.")
        accel_sum = {'x': 0, 'y': 0, 'z': 0}
        gyro_sum = {'x': 0, 'y': 0, 'z': 0}
        for _ in range(samples):
            accel = self.get_accel_data()
            gyro = self.get_gyro_data()
            accel_sum['x'] += accel['x']
            accel_sum['y'] += accel['y']
            accel_sum['z'] += accel['z']
            gyro_sum['x'] += gyro['x']
            gyro_sum['y'] += gyro['y']
            gyro_sum['z'] += gyro['z']
            time.sleep(0.001)
        self.accel_offsets = {k: v / samples for k, v in accel_sum.items()}
        self.gyro_offsets = {k: v / samples for k, v in gyro_sum.items()}
        self.accel_offsets['z'] -= 1.0
        print("Calibration complete.")
        print(f"Accel Offsets: {self.accel_offsets}")
        print(f"Gyro Offsets: {self.gyro_offsets}")

    def calculate_angles(self, accel):
        """Calculate roll and pitch angles from accelerometer data."""
        roll = math.atan2(accel['y'], math.sqrt(accel['x'] ** 2 + accel['z'] ** 2))
        pitch = math.atan2(-accel['x'], math.sqrt(accel['y'] ** 2 + accel['z'] ** 2))
        yaw = math.atan2(accel['y'], accel['x'])
        return {'roll': roll, 'pitch': pitch, 'yaw': yaw}

    def kalman_filter(self, kalman_state, gyro_rate, accel_angle, dt):
        """Apply Kalman filter."""
        # Predict
        kalman_state['angle'] += gyro_rate * dt
        kalman_state['uncertainty'] += (0.004 ** 2) * 16

        # Update
        kalman_gain = kalman_state['uncertainty'] / (kalman_state['uncertainty'] + 9)
        kalman_state['angle'] += kalman_gain * (accel_angle - kalman_state['angle'])
        kalman_state['uncertainty'] *= (1 - kalman_gain)

        return kalman_state
    
def main(args=None):
    rclpy.init(args=args)

    mpu = MPU6050()
    mpu.initialize()
    print("MPU-6050 Initialized. Starting calibration...")
    mpu.calibrate()

    rclpy.spin(mpu)

    mpu.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
    # mpu = MPU6050()
    # mpu.initialize()
    # print("MPU-6050 Initialized. Starting calibration...")
    # mpu.calibrate()

    # print("Reading filtered data...")
    # loop_timer = time.time()
    # try:
    #     while True:
    #         dt = time.time() - loop_timer
    #         loop_timer = time.time()

    #         accel = mpu.get_accel_data()
    #         gyro = mpu.get_gyro_data()

    #         accel_angles = mpu.calculate_angles(accel)

    #         # Apply Kalman filter
    #         mpu.kalman_roll = mpu.kalman_filter(mpu.kalman_roll, gyro['x'], accel_angles['roll'], dt)
    #         mpu.kalman_pitch = mpu.kalman_filter(mpu.kalman_pitch, gyro['y'], accel_angles['pitch'], dt)
    #         mpu.kalman_yaw = mpu.kalman_filter(mpu.kalman_yaw, gyro['z'], accel_angles['yaw'], dt)

    #         print(f"Roll: {mpu.kalman_roll['angle']:.2f}° Pitch: {mpu.kalman_pitch['angle']:.2f}° Yaw: {mpu.kalman_yaw['angle']:.2f}°")

    #         time.sleep(0.05)  # 50ms delay
    # except KeyboardInterrupt:
    #     print("Exiting...")
