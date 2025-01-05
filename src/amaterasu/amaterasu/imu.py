import smbus
import rclpy
import time
import math
# from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
# from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32

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
        # self.tf_broadcaster = TransformBroadcaster(self)

        self.yaw_angle = 0

        self.last_time = time.time()
        self.dt = 0.1

        self.publisher = self.create_publisher(Float32, "/imu/raw_data", 10)
        self.timer = self.create_timer(self.dt, self.publish_imu)

    def initialize(self):
        """Initialize the MPU-6050."""
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0)
        time.sleep(0.1)
        
    # #yeni kod buraya eklendi       
    # def get_transform(self):
    #     """Get the IMU's transform as a TransformStamped message."""
    #     t = TransformStamped()
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = "base_link"  # Match the robot's frame
    #     t.child_frame_id = "imu_link"

    #     quaternion = quaternion_from_euler(
    #         self.kalman_roll['angle'], self.kalman_pitch['angle'], self.kalman_yaw['angle']
    #     )
    #     t.transform.translation.x = 0.0
    #     t.transform.translation.y = 0.0
    #     t.transform.translation.z = 0.0
    #     t.transform.rotation.x = quaternion[0]
    #     t.transform.rotation.y = quaternion[1]
    #     t.transform.rotation.z = quaternion[2]
    #     t.transform.rotation.w = quaternion[3]
    #     return t

    def publish_imu(self):
        current_time = time.time()
        self.dt = current_time - self.last_time
        self.last_time = current_time
        gyro = self.get_gyro_data()
        gyro_z = gyro['z']

        self.yaw_angle = (self.yaw_angle + gyro_z * self.dt) % 360

        # self.get_logger().info(f"Gyro z: {gyro_z:.2f}°")
    
        yaw = Float32()
        yaw.data = gyro_z # degree/s
        
        # #yeni kod buraya eklendi       
        # imu_transform = self.get_transform()
        # self.tf_broadcaster.sendTransform(imu_transform)
     
        self.publisher.publish(yaw)

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