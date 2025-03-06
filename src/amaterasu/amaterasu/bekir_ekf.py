import numpy as np
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class ExtendedKalmanFilter(Node):
    def __init__(self):
        super().__init__("ekf")

        # initial pose
        self.x = 0
        self.y = 0
        self.theta = 0

        # initial sensor values
        self.v_odom = 0
        self.w_odom = 0
        self.w_imu = 0

        # state vector -> x, y, yaw, v, w
        self.mu = np.array([self.x, self.y, self.theta, 0, 0])

        # sensor observations
        self.z_k = np.array([self.v_odom, self.w_odom, self.w_imu]) 

        # sensor noise
        self.sigma_z_sq = np.array([[0.1, 0, 0], #noise in v_odom
                                    [0, 0.3, 0], #noise in w_odom
                                    [0, 0, 0.02]]) #noise in w_gyro
        

        ## DEĞİŞTİR
        self.H = np.array([[0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 1],
                           [0, 0, 0, 0, 1]])
        
        # Pk
        self.sigma_sq = np.array([[0.1, 0, 0, 0, 0],
                                  [0, 0.1, 0, 0, 0],
                                  [0, 0, 0.1, 0, 0],
                                  [0, 0, 0, 0.1, 0],
                                  [0, 0, 0, 0, 0.1]])
        
        # Qk
        self.sigma_m_sq = np.array([[0.1, 0, 0, 0, 0],
                                   [0, 0.1, 0, 0, 0],
                                   [0, 0, 0.1, 0, 0],
                                   [0, 0, 0, 0.1, 0],
                                   [0, 0, 0, 0, 0.1]])

        self.gyro_offset = None

        self.imu_sub = self.create_subscription(Imu, "/imu/z", self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        
        self.dt = 0.01 # 10ms
        self.last_time = time.time()
        self.timer = self.create_timer(self.dt, self.run)

    def imu_callback(self,msg):
        self.w_imu = msg.angular_velocity.z

    def odom_callback(self,msg):
        self.w_odom = msg.twist.twist.angular.z
        self.v_odom = msg.twist.twist.linear.x 

    def run(self):
        curr_time = time.time()
        dk = curr_time - last_time
        last_time = time.time()
        x_k, y_k, theta_k, v_k, w_k = self.mu
        
        self.z_k = np.array([self.v_odom, self.w_odom, self.w_imu])

        #Predict step
        self.mu = np.array([x_k + v_k*dk*np.cos(theta_k),
                            y_k + v_k*dk*np.sin(theta_k),
                            theta_k + w_k*dk,
                            v_k,
                            w_k])
    
        #The Jacobian of update model
        F_k = np.array([[1, 0, -v_k*dk*np.sin(theta_k), dk*np.cos(theta_k), 0],
                        [0, 1, -v_k*dk*np.cos(theta_k), dk*np.sin(theta_k), 0],
                        [0, 0, 1, 0, dk],
                        [0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 1]])
    
        # Update error in prediction
        self.sigma_sq = F_k.dot(self.sigma_sq).dot(F_k.T) + self.sigma_m_sq

        # Update step
        measurement_residual = self.z_k - self.H.dot(self.mu)

        residual_covariance = self.H.dot(self.sigma_sq).dot(self.H.T) + self.sigma_z_sq

        # Kalman gain
        K_k = self.sigma_sq.dot(self.H.T).dot(np.linalg.inv(residual_covariance))

        self.mu = self.mu + K_k.dot(measurement_residual)

        self.sigma_sq = (np.eye(len(self.mu))-K_k.dot(self.H)).dot(self.sigma_sq)

def main(args=None):
    rclpy.init(args=args)
    
    ekf = ExtendedKalmanFilter()
    rclpy.spin(ekf)
    
    ekf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
