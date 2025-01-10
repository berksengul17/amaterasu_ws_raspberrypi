import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

class OrientationFilter(Node):
    def __init__(self):
        super().__init__('orientation_filter')

        # System constants
        self.deltat = 0.001  # Sampling period in seconds (1 ms)
        self.gyro_meas_error = math.radians(5.0)  # Gyroscope measurement error in rad/s
        self.gyro_meas_drift = math.radians(0.2)  # Gyroscope drift in rad/s/s
        self.beta = math.sqrt(3.0 / 4.0) * self.gyro_meas_error  # Beta
        self.zeta = math.sqrt(3.0 / 4.0) * self.gyro_meas_drift  # Zeta

        # Global system variables
        self.a_x, self.a_y, self.a_z = 0.0, 0.0, 0.0  # Accelerometer measurements
        self.w_x, self.w_y, self.w_z = 0.0, 0.0, 0.0  # Gyroscope measurements (rad/s)
        self.m_x, self.m_y, self.m_z = 0.0, 0.0, 0.0  # Magnetometer measurements
        self.SEq_1, self.SEq_2, self.SEq_3, self.SEq_4 = 1.0, 0.0, 0.0, 0.0  # Orientation quaternion
        self.b_x, self.b_z = 1.0, 0.0  # Flux in earth frame
        self.w_bx, self.w_by, self.w_bz = 0.0, 0.0, 0.0  # Gyroscope biases

        # Subscriptions
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        self.mag_subscription = self.create_subscription(
            Float32,
            '/magnetometer/smoothed',
            self.mag_callback,
            10
        )

        # Publisher
        self.yaw_publisher = self.create_publisher(Float32, '/yaw_angle', 10)

    def imu_callback(self, msg: Imu):
        # Extract accelerometer and gyroscope data from IMU
        self.a_x = msg.linear_acceleration.x
        self.a_y = msg.linear_acceleration.y
        self.a_z = msg.linear_acceleration.z

        self.w_x = msg.angular_velocity.x
        self.w_y = msg.angular_velocity.y
        self.w_z = msg.angular_velocity.z

        # Update the filter
        self.filter_update(self.w_x, self.w_y, self.w_z, self.a_x, self.a_y, self.a_z, self.m_x, self.m_y, self.m_z)

        # Publish the yaw angle
        yaw = self.compute_yaw_angle()
        self.yaw_publisher.publish(Float32(data=yaw))

    def mag_callback(self, msg: Float32):
        # Update magnetometer data
        self.m_x = msg.data  # Assuming smoothed magnetometer data is scalar for this example
        self.m_y = 0.0  # Placeholder for additional data
        self.m_z = 0.0  # Placeholder for additional data

    def filter_update(self, w_x, w_y, w_z, a_x, a_y, a_z, m_x, m_y, m_z):
        # Normalize accelerometer measurement
        norm = math.sqrt(a_x ** 2 + a_y ** 2 + a_z ** 2)
        a_x, a_y, a_z = a_x / norm, a_y / norm, a_z / norm

        # Normalize magnetometer measurement
        norm = math.sqrt(m_x ** 2 + m_y ** 2 + m_z ** 2)
        m_x, m_y, m_z = m_x / norm, m_y / norm, m_z / norm

        # Auxiliary variables
        halfSEq_1 = 0.5 * self.SEq_1
        halfSEq_2 = 0.5 * self.SEq_2
        halfSEq_3 = 0.5 * self.SEq_3
        halfSEq_4 = 0.5 * self.SEq_4
        twoSEq_1 = 2.0 * self.SEq_1
        twoSEq_2 = 2.0 * self.SEq_2
        twoSEq_3 = 2.0 * self.SEq_3
        twoSEq_4 = 2.0 * self.SEq_4

        # Compute the objective function and Jacobian
        f_1 = twoSEq_2 * self.SEq_4 - twoSEq_1 * self.SEq_3 - a_x
        f_2 = twoSEq_1 * self.SEq_2 + twoSEq_3 * self.SEq_4 - a_y
        f_3 = 1.0 - twoSEq_2 * self.SEq_2 - twoSEq_3 * self.SEq_3 - a_z

        # Gradient descent algorithm corrective step
        SEqHatDot_1 = f_1 * (-twoSEq_3) + f_2 * (twoSEq_4) - f_3 * (twoSEq_2)
        SEqHatDot_2 = f_1 * (twoSEq_4) + f_2 * (twoSEq_1) - f_3 * (twoSEq_3)
        SEqHatDot_3 = f_1 * (-twoSEq_1) + f_2 * (twoSEq_2) + f_3 * (twoSEq_4)
        SEqHatDot_4 = f_1 * (twoSEq_2) - f_2 * (twoSEq_3) + f_3 * (twoSEq_1)

        # Normalize the gradient
        norm = math.sqrt(SEqHatDot_1 ** 2 + SEqHatDot_2 ** 2 + SEqHatDot_3 ** 2 + SEqHatDot_4 ** 2)
        SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4 = (
            SEqHatDot_1 / norm,
            SEqHatDot_2 / norm,
            SEqHatDot_3 / norm,
            SEqHatDot_4 / norm,
        )

        # Compute gyroscope biases
        self.w_bx += SEqHatDot_1 * self.deltat * self.zeta
        self.w_by += SEqHatDot_2 * self.deltat * self.zeta
        self.w_bz += SEqHatDot_3 * self.deltat * self.zeta
        w_x -= self.w_bx
        w_y -= self.w_by
        w_z -= self.w_bz

        # Compute the quaternion rate and integrate
        SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z
        SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y
        SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x
        SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x

        self.SEq_1 += (SEqDot_omega_1 - self.beta * SEqHatDot_1) * self.deltat
        self.SEq_2 += (SEqDot_omega_2 - self.beta * SEqHatDot_2) * self.deltat
        self.SEq_3 += (SEqDot_omega_3 - self.beta * SEqHatDot_3) * self.deltat
        self.SEq_4 += (SEqDot_omega_4 - self.beta * SEqHatDot_4) * self.deltat

        # Normalize quaternion
        norm = math.sqrt(self.SEq_1 ** 2 + self.SEq_2 ** 2 + self.SEq_3 ** 2 + self.SEq_4 ** 2)
        self.SEq_1, self.SEq_2, self.SEq_3, self.SEq_4 = (
            self.SEq_1 / norm,
            self.SEq_2 / norm,
            self.SEq_3 / norm,
            self.SEq_4 / norm,
        )

    def compute_yaw_angle(self):
        # Compute the yaw angle (in radians) from the quaternion
        yaw = math.atan2(2.0 * (self.SEq_1 * self.SEq_4 + self.SEq_2 * self.SEq_3),
                         1.0 - 2.0 * (self.SEq_3 ** 2 + self.SEq_4 ** 2))
        return yaw

def main(args=None):
    rclpy.init(args=args)
    filter_node = OrientationFilter()
    rclpy.spin(filter_node)
    filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
