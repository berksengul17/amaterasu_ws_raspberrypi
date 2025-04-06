import numpy as np
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time

class ExtendedKalmanFilter(Node):
    def __init__(self):
        super().__init__("ekf")

        # initial pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # initial sensor values
        self.v_odom = 0.0
        self.w_odom = 0.0
        self.yaw_imu = 0.0

        # state vector -> x, y, yaw, v, w
        self.mu = np.array([self.x, self.y, self.theta, 0.0, 0.0])

        # sensor observations
        self.z_k = np.array([self.v_odom, self.w_odom, self.yaw_imu]) 

        # sensor noise
        self.sigma_z_sq = np.array([[0.1, 0, 0], #noise in v_odom
                                    [0, 0.1, 0], #noise in w_odom
                                    [0, 0, 0.02]]) #noise in yaw_imu

        ## DEĞİŞTİR
        self.H = np.array([[0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 1],
                           [0, 0, 1, 0, 0]])
        
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

        self.odom_pub = self.create_publisher(Odometry, "/ekf_odom", 10)
        self.odomBroadcaster = TransformBroadcaster(self)
        
        self.dt = 0.01 # 10ms
        self.last_time = time.time()
        self.timer = self.create_timer(self.dt, self.run)

    def normalize_angle(self, angle):
        return (angle + 180.0) % 360.0 - 180.0

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_imu = self.normalize_angle(yaw)
        
    def odom_callback(self,msg):
        self.w_odom = msg.twist.twist.angular.z
        self.v_odom = msg.twist.twist.linear.x

    def run(self):
        curr_time = time.time()
        dk = curr_time - self.last_time
        self.last_time = time.time()
        x_k, y_k, theta_k, v_k, w_k = self.mu
        
        self.z_k = np.array([self.v_odom, self.w_odom, self.yaw_imu])

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

        odom = Odometry()
        #Publish the new odom message based on the integrated odom values
        odom.header.stamp = Time()  
        odom.header.stamp.sec = int(curr_time)
        odom.header.stamp.nanosec = int((curr_time % 1) * 1e9)

        odom.pose.pose.position.x = float(self.mu[0])
        odom.pose.pose.position.y = float(self.mu[1])
        odom.pose.pose.position.z = 0.0

        print(f"yaw_imu: {self.yaw_imu} | w_odom: {self.w_odom} | Combined yaw: {self.mu[2]:.2f}")

        qt_array = quaternion_from_euler(0,0,self.mu[2])
        quaternion = Quaternion(x=qt_array[0], y=qt_array[1], z=qt_array[2], w=qt_array[3])
        # quaternion.z = sin(self.mu[2]/2.0)
        # quaternion.w = cos(self.mu[2]/2.0)
        odom.pose.pose.orientation = quaternion

        odom.pose.covariance = [self.sigma_sq[0,0], 0, 0, 0, 0, 0, #uncertainty in x
                                0, self.sigma_sq[1,1], 0, 0, 0, 0, #uncertainty in y
                                0, 0, 0, 0, 0, 0, #uncertainty in z
                                0, 0, 0, 0, 0, 0, #uncertainty in roll
                                0, 0, 0, 0, 0, 0, #uncertainty in pitch
                                0, 0, 0, 0, 0, self.sigma_sq[2,2]] #uncertainty in yaw

        #The velocities are in child frame base_link
        odom.twist.twist.linear.x = self.mu[3]
        odom.twist.twist.angular.z = self.mu[4] 

        odom.twist.covariance = [self.sigma_sq[3,3], 0, 0, 0, 0, 0, #uncertainty in x_dot
                                0, 0, 0, 0, 0, 0, #uncertainty in y_dot
                                0, 0, 0, 0, 0, 0, #uncertainty in z_dot
                                0, 0, 0, 0, 0, 0, #uncertainty in change in roll
                                0, 0, 0, 0, 0, 0, #uncertainty in change in pitch
                                0, 0, 0, 0, 0, self.sigma_sq[4,4]] #uncertainty in change in yaw

        tf = TransformStamped()
        tf.header.stamp = Time()
        tf.header.stamp.sec = int(curr_time)
        tf.header.stamp.nanosec = int((curr_time % 1) * 1e9)
        
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = float(self.mu[0])
        tf.transform.translation.y = float(self.mu[1])
        tf.transform.translation.z = 0.0
         
        tf.transform.rotation.x = quaternion.x
        tf.transform.rotation.y = quaternion.y
        tf.transform.rotation.z = quaternion.z
        tf.transform.rotation.w = quaternion.w

        
        self.odomBroadcaster.sendTransform(tf)
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    
    ekf = ExtendedKalmanFilter()
    rclpy.spin(ekf)
    
    ekf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
