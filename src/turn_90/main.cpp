#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <wiringPi.h>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define L_EN_PIN 13
#define R_EN_PIN 12

#define FL_IN1_PIN 17
#define FL_IN2_PIN 27
#define FR_IN1_PIN 22
#define FR_IN2_PIN 23

#define RL_IN1_PIN 26
#define RL_IN2_PIN 16
#define RR_IN1_PIN 24
#define RR_IN2_PIN 25

class Turn90 : public rclcpp::Node {
public:
    Turn90()
        : Node("turn90"), estimated_yaw(0.0), target_yaw(90.0), turning(false)
    {
        // Initialize WiringPi
        wiringPiSetupGpio();
        
        // Set motor pins as outputs
        pinMode(L_EN_PIN, PWM_OUTPUT);
        pinMode(R_EN_PIN, PWM_OUTPUT);
        
        pinMode(FL_IN1_PIN, OUTPUT);
        pinMode(FL_IN2_PIN, OUTPUT);
        pinMode(FR_IN1_PIN, OUTPUT);
        pinMode(FR_IN2_PIN, OUTPUT);
        
        pinMode(RL_IN1_PIN, OUTPUT);
        pinMode(RL_IN2_PIN, OUTPUT);
        pinMode(RR_IN1_PIN, OUTPUT);
        pinMode(RR_IN2_PIN, OUTPUT);

        pwmSetRange(1024);
        pwmSetClock(192);

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ekf_odom", 10, std::bind(&Turn90::updateYaw, this, std::placeholders::_1));

        last_time = this->now();

        turnRight();
        turning = true;
        
        RCLCPP_INFO(this->get_logger(), "Turn90 Node Initialized");
    }
    
    ~Turn90() {
        RCLCPP_WARN(this->get_logger(), "Node shutting down... Stopping motors.");
        stopMotors();
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    
    double estimated_yaw;
    double target_yaw;
    bool turning;
    rclcpp::Time last_time;
    double last_gyro_z = 0.0;

    void updateYaw(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Extract quaternion from odometry
        const auto& orientation = msg->pose.pose.orientation;
        tf2::Quaternion q(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        );
    
        // Convert quaternion to roll, pitch, yaw
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
        // Convert yaw to degrees and store as estimated yaw
        estimated_yaw = yaw * (180.0 / M_PI);
    
        // Normalize to [-180, 180]
        estimated_yaw = std::fmod(estimated_yaw + 180.0, 360.0) - 180.0;
    
        // Compute yaw error to target
        double error = std::fabs(std::fabs(target_yaw) - std::fabs(estimated_yaw));
    
        RCLCPP_INFO(this->get_logger(), "Current Yaw: %.2f | Target: %.2f | Error: %.2f", estimated_yaw, target_yaw, error);
    
        // Stopping logic
        if (turning && error <= 5.0) {
            stopMotors();
            turning = false;
            RCLCPP_INFO(this->get_logger(), "Turn Completed! Estimated Yaw = %.2f degrees", estimated_yaw);
        } else if (turning) {
            changeSpeed(error / 90.0 * 1.0);
        }
    }    
    
    void turnRight() {
        digitalWrite(FL_IN1_PIN, LOW);
        digitalWrite(FL_IN2_PIN, HIGH);
        digitalWrite(FR_IN1_PIN, HIGH);
        digitalWrite(FR_IN2_PIN, LOW);

        digitalWrite(RL_IN1_PIN, LOW);
        digitalWrite(RL_IN2_PIN, HIGH);
        digitalWrite(RR_IN1_PIN, HIGH);
        digitalWrite(RR_IN2_PIN, LOW);

        RCLCPP_INFO(this->get_logger(), "Turning left...");
    }

    void changeSpeed(double speed) {
        // Constrain speed within valid range (min 200, max 1024)
        int constrained_speed = static_cast<int>(std::max(0.5, std::min(speed, 1.0)) * 500.0);
    
        RCLCPP_INFO(this->get_logger(), "Speed changed: %d", constrained_speed);
        pwmWrite(L_EN_PIN, constrained_speed);
        pwmWrite(R_EN_PIN, constrained_speed);
    }
    
    void stopMotors() {
        pwmWrite(L_EN_PIN, 0);
        pwmWrite(R_EN_PIN, 0);

        digitalWrite(FL_IN1_PIN, LOW);
        digitalWrite(FL_IN2_PIN, LOW);
        digitalWrite(FR_IN1_PIN, LOW);
        digitalWrite(FR_IN2_PIN, LOW);

        digitalWrite(RL_IN1_PIN, LOW);
        digitalWrite(RL_IN2_PIN, LOW);
        digitalWrite(RR_IN1_PIN, LOW);
        digitalWrite(RR_IN2_PIN, LOW);

        RCLCPP_INFO(this->get_logger(), "Motors Stopped.");
    }
};

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Turn90>());
    rclcpp::shutdown();
    return 0;
}
