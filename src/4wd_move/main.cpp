#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <wiringPi.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>  // For M_PI

// Motor control pins
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

// Encoder pins
#define FL_ENC_PIN 6
#define FR_ENC_PIN 5
#define RL_ENC_PIN 8
#define RR_ENC_PIN 14

#define TICKS_PER_ROTATION 40   
#define WHEEL_DIAMETER 0.065  
#define WHEEL_BASE_WIDTH 0.13  
#define DEBOUNCE_TIME_US 1000   
#define DISTANCE 1 // meters

// Compute steps per meter
const double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;
const int STEPS_PER_METER = (DISTANCE / WHEEL_CIRCUMFERENCE) * TICKS_PER_ROTATION;

// Atomic variables for encoder steps
std::atomic<unsigned long> steps_FL(0);
std::atomic<unsigned long> steps_FR(0);
std::atomic<unsigned long> steps_RL(0);
std::atomic<unsigned long> steps_RR(0);
std::atomic<bool> running(true);

enum State { MOVING_FORWARD, TURNING };
State robot_state = MOVING_FORWARD;

class MoveSquare : public rclcpp::Node {
public:
    MoveSquare() : Node("move_square"), estimated_yaw(0.0), target_yaw(-90.0), turning(false), completed_turns(0),
                   kp(3.0f), ki(0.09f), kd(0.5f), integral(0), yaw_error(0.0), last_error(0), left_speed(200), right_speed(200)
    {
        wiringPiSetupGpio();

        // Initialize motor pins
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

        pinMode(FL_ENC_PIN, INPUT);
        pullUpDnControl(FL_ENC_PIN, PUD_UP);
        pinMode(FR_ENC_PIN, INPUT);
        pullUpDnControl(FR_ENC_PIN, PUD_UP);
        pinMode(RL_ENC_PIN, INPUT);
        pullUpDnControl(RL_ENC_PIN, PUD_UP);
        pinMode(RR_ENC_PIN, INPUT);
        pullUpDnControl(RR_ENC_PIN, PUD_UP);

        pwmSetRange(1024);
        pwmSetClock(192);

        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/z", 10, std::bind(&MoveSquare::imuCallback, this, std::placeholders::_1));

        timer = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&MoveSquare::updateMotion, this)
        );

        encoder_thread = std::thread(&MoveSquare::countEncoders, this);
        last_time = this->now();

        RCLCPP_INFO(this->get_logger(), "MoveSquare Node Initialized");
        moveForward();
    }

    ~MoveSquare() {
        RCLCPP_WARN(this->get_logger(), "Node shutting down... Stopping motors.");
        stopMotors();
        running = false;
        if (encoder_thread.joinable()) {
            encoder_thread.join();
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::TimerBase::SharedPtr timer;
    std::thread encoder_thread;

    double estimated_yaw;
    double target_yaw;
    bool turning;
    int completed_turns;
    rclcpp::Time last_time;
    double last_gyro_z = 0.0;

    float kp, ki, kd;
    float integral;
    double yaw_error;
    int last_error;
    int left_speed, right_speed;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_gyro_z = msg->angular_velocity.z;
    }

    void updateMotion() {
        if (robot_state == MOVING_FORWARD) {
            int error = ((steps_FL + steps_RL) / 2) - ((steps_FR + steps_RR) / 2);

            auto current_time = this->now();
            double dt = (current_time - last_time).seconds();
            last_time = current_time;

            integral += error * dt;
            integral = std::max(std::min(integral, 500.0f), -500.0f);

            int derivative = (error - last_error) / dt;
            last_error = error;

            int correction = static_cast<int>(kp * error + ki * integral + kd * derivative);

            if (correction > 0) {  
                right_speed = std::min(300, right_speed + correction);
                left_speed = std::max(100, left_speed - correction);
            } else {  
                left_speed = std::min(300, left_speed - correction);
                right_speed = std::max(100, right_speed + correction);
            }

            pwmWrite(L_EN_PIN, left_speed);
            pwmWrite(R_EN_PIN, right_speed);

            RCLCPP_INFO(this->get_logger(), "FL: %lu | FR: %lu | RL: %lu | RR: %lu | LS: %d | RS: %d",
                        steps_FL.load(), steps_FR.load(), steps_RL.load(), steps_RR.load(),
                        left_speed, right_speed);

            if (steps_FL >= STEPS_PER_METER || steps_FR >= STEPS_PER_METER ||
                steps_RL >= STEPS_PER_METER || steps_RR >= STEPS_PER_METER) {
                
                stopMotors();
                robot_state = TURNING;
                estimated_yaw = 0.0;
                // target_yaw = -90.0 + yaw_error;
                resetEncoders();
                turnLeft();
                RCLCPP_INFO(this->get_logger(), "Started turning. Target yaw: %.2f", target_yaw);
            }
        } else if (robot_state == TURNING) {
            RCLCPP_INFO(this->get_logger(), "Turning left...");
            rclcpp::Time current_time = this->now();
            double dt = (current_time - last_time).seconds();
            last_time = current_time;

            estimated_yaw += last_gyro_z * dt * (180.0 / M_PI);
            
            yaw_error = target_yaw - estimated_yaw;
            RCLCPP_INFO(this->get_logger(), "Error: %.2f | Estimated Yaw: %.2f", yaw_error, estimated_yaw);

            if (fabs(yaw_error) <= 4.0) {
                stopMotors();
                completed_turns++;

                if (completed_turns < 4) {
                    robot_state = MOVING_FORWARD;
                    resetEncoders();
                    moveForward();
                } else {
                    RCLCPP_INFO(this->get_logger(), "Square path completed!");
                    timer->cancel();
                }
            } else {
                changeSpeed(static_cast<int>(yaw_error * 2.0));
            }
        }
    }

    void moveForward() {
        RCLCPP_INFO(this->get_logger(), "Moving forward...");
        digitalWrite(FL_IN1_PIN, HIGH);
        digitalWrite(FL_IN2_PIN, LOW);
        digitalWrite(FR_IN1_PIN, HIGH);
        digitalWrite(FR_IN2_PIN, LOW);
        digitalWrite(RL_IN1_PIN, HIGH);
        digitalWrite(RL_IN2_PIN, LOW);
        digitalWrite(RR_IN1_PIN, HIGH);
        digitalWrite(RR_IN2_PIN, LOW);
    }

    void turnLeft() {
        digitalWrite(FL_IN1_PIN, LOW);
        digitalWrite(FL_IN2_PIN, HIGH);
        digitalWrite(FR_IN1_PIN, HIGH);
        digitalWrite(FR_IN2_PIN, LOW);
        digitalWrite(RL_IN1_PIN, LOW);
        digitalWrite(RL_IN2_PIN, HIGH);
        digitalWrite(RR_IN1_PIN, HIGH);
        digitalWrite(RR_IN2_PIN, LOW);
    }

    void changeSpeed(int speed) {
        // Constrain speed within valid range (min 200, max 1024)
        int constrained_speed = std::max(300, std::min(speed, 350));
    
        RCLCPP_INFO(this->get_logger(), "Speed changed: %d", constrained_speed);
        pwmWrite(L_EN_PIN, constrained_speed);
        pwmWrite(R_EN_PIN, constrained_speed);
    }

    void stopMotors() {
        pwmWrite(L_EN_PIN, 0);
        pwmWrite(R_EN_PIN, 0);
    }

    void resetEncoders() {
        steps_FL = steps_FR = steps_RL = steps_RR = 0;
    }
    
    void countEncoders() {
        bool last_state_FL = digitalRead(FL_ENC_PIN);
        bool last_state_FR = digitalRead(FR_ENC_PIN);
        bool last_state_RL = digitalRead(RL_ENC_PIN);
        bool last_state_RR = digitalRead(RR_ENC_PIN);
    
        while (running) {
            bool current_state_FL = digitalRead(FL_ENC_PIN);
            bool current_state_FR = digitalRead(FR_ENC_PIN);
            bool current_state_RL = digitalRead(RL_ENC_PIN);
            bool current_state_RR = digitalRead(RR_ENC_PIN);
    
            if (current_state_FL != last_state_FL) { steps_FL++; }
            if (current_state_FR != last_state_FR) { steps_FR++; }
            if (current_state_RL != last_state_RL) { steps_RL++; }
            if (current_state_RR != last_state_RR) { steps_RR++; }
    
            last_state_FL = current_state_FL;
            last_state_FR = current_state_FR;
            last_state_RL = current_state_RL;
            last_state_RR = current_state_RR;
    
            usleep(DEBOUNCE_TIME_US);  // Small delay for accuracy
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveSquare>());
    rclcpp::shutdown();
    return 0;
}
