#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <wiringPi.h>
#include <chrono>

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
        : Node("turn90"), estimated_yaw(0.0), target_yaw(-90.0), turning(false)
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

        // Subscribe to IMU topic (only angular velocity z)
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/z", 10, std::bind(&Turn90::imuCallback, this, std::placeholders::_1));

        // Timer to periodically update yaw estimation
        timer = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&Turn90::updateYaw, this)
        );

        last_time = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Turn90 Node Initialized");
    }
    
    ~Turn90() {
        RCLCPP_WARN(this->get_logger(), "Node shutting down... Stopping motors.");
        stopMotors();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::TimerBase::SharedPtr timer;
    
    double estimated_yaw;
    double target_yaw;
    bool turning;
    rclcpp::Time last_time;
    double last_gyro_z = 0.0;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_gyro_z = msg->angular_velocity.z; // Read angular velocity (rad/s)
    }

    void updateYaw() {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time).seconds(); // Time step in seconds
        last_time = current_time;
    
        // Integrate angular velocity to estimate yaw angle
        estimated_yaw += last_gyro_z * dt * (180.0 / M_PI); // Convert rad to degrees
        RCLCPP_INFO(this->get_logger(), "Estimated yaw = %.2f", estimated_yaw);
    
        if (!turning) {
            turning = true;
            estimated_yaw = 0.0; // Reset yaw estimation when starting
            turnRight(); // Start turning
            RCLCPP_INFO(this->get_logger(), "Started turning...");
        }
    
        // **Correct the stopping condition**
        double error = fabs(fabs(target_yaw) - fabs(estimated_yaw));
        
        if (turning && fabs(error) <= 2.0) { 
            stopMotors();
            turning = false;  // Stop turning
            RCLCPP_INFO(this->get_logger(), "Turn Completed! Estimated Yaw = %.2f degrees", estimated_yaw);
            timer->cancel(); // Disable the timer
        } else {
            changeSpeed(static_cast<int>(error * 1.5)); // Increase P-Gain for faster correction
        }
    }
    
    void turnRight() {
        digitalWrite(FL_IN1_PIN, HIGH);
        digitalWrite(FL_IN2_PIN, LOW);
        digitalWrite(FR_IN1_PIN, LOW);
        digitalWrite(FR_IN2_PIN, HIGH);

        digitalWrite(RL_IN1_PIN, HIGH);
        digitalWrite(RL_IN2_PIN, LOW);
        digitalWrite(RR_IN1_PIN, LOW);
        digitalWrite(RR_IN2_PIN, HIGH);

        RCLCPP_INFO(this->get_logger(), "Turning Right...");
    }

    void changeSpeed(int speed) {
        // Constrain speed within valid range (min 200, max 1024)
        int constrained_speed = std::max(300, std::min(speed, 500)); 
    
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
