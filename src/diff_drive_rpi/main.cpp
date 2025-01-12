#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "amaterasu_interfaces/msg/robot_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rotary_encoder.hpp"
#include "robot.h"
#include "robot_pins.h"
#include "dc_motor.h"
#include "encoder.h"
#include <pigpiod_if2.h>
#include <cmath>
#include <stdexcept>

#define MOTOR_PPR 960.0f
#define SAMPLE_TIME_MS 20
#define PWM_FREQUENCY 25

class RobotNode : public rclcpp::Node {
public:
    RobotNode()
        : Node("robot_control_node"), linear_(0.0), angular_(0.0),
          kp1_(0.04), ki1_(0.01), kd1_(0.0),
          robot_pins_{{PWM_FREQUENCY, L_ENA_PIN, L_IN1_PIN, L_IN2_PIN},
                      {PWM_FREQUENCY, R_ENB_PIN, R_IN3_PIN, R_IN4_PIN}},
          robot_(pigpio_handle_, kp1_, kd1_, ki1_, SAMPLE_TIME_MS, robot_pins_)
    {

        // Publisher for odometry
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscription for cmd_vel
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&RobotNode::cmdVelCallback, this, std::placeholders::_1));

        // Timer for periodic updates
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(SAMPLE_TIME_MS),
            std::bind(&RobotNode::update, this));

        // Initialize hardware
        setup();
    }

private:
    void setup() {
        // Initialize pigpio
        pigpio_handle_ = pigpio_start(NULL, NULL);
        if (pigpio_handle_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio.");
            throw std::runtime_error("Failed to initialize pigpio.");
        }

        // // Initialize encoders dynamically after pigpio_handle_ is ready
        // left_encoder_ = std::make_unique<Encoder>(L_ENC_PIN, pigpio_handle_);
        // right_encoder_ = std::make_unique<Encoder>(R_ENC_PIN, pigpio_handle_);

        // // Initialize encoders
        // left_encoder_->set_pulses(0);
        // right_encoder_->set_pulses(0);

        // Define encoder callbacks
        auto left_encoder_callback = [this](int direction) {
            left_encoder_pulses_ += direction;
        };

        auto right_encoder_callback = [this](int direction) {
            right_encoder_pulses_ += direction;
        };

        // Initialize dual_encoder
        dual_encoders_ = std::make_unique<dual_encoder>(
            L_ENC_PIN, left_encoder_callback,
            R_ENC_PIN, right_encoder_callback);

        RCLCPP_INFO(this->get_logger(), "Setup complete.");
    }

    void printState(RobotState state, RobotOdometry odometry)
    {
        printf(
            // diff setpoint,wheel setpoint, speed
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            state.l_ref_speed, state.r_ref_speed, state.l_speed, state.r_speed, state.l_effort, state.r_effort,
            odometry.x_pos, odometry.y_pos, odometry.theta, odometry.v, odometry.w
            );
    }


    geometry_msgs::msg::Quaternion quaternionFromEuler(double roll, double pitch, double yaw) {
        geometry_msgs::msg::Quaternion q;

        // Precompute trigonometric terms
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        // Compute quaternion components
        q.w = cy * cp * cr + sy * sp * sr;
        q.x = cy * cp * sr - sy * sp * cr;
        q.y = sy * cp * sr + cy * sp * cr;
        q.z = sy * cp * cr - cy * sp * sr;

        return q;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        linear_ = msg->linear.x;
        angular_ = msg->angular.z;
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear=%.2f, angular=%.2f", linear_, angular_);
    }

    void update() {
        // Update robot state
        robot_.setUnicycle(linear_, angular_);
        robot_.updatePid(left_encoder_pulses_, right_encoder_pulses_);

        // Reset encoder pulses after reading
        left_encoder_pulses_ = 0;
        right_encoder_pulses_ = 0;

        // Get the current robot state and odometry
        // auto state = robot_.getState();
        auto odometry = robot_.getOdometry();
        auto robot_orientation = quaternionFromEuler(0.0, 0.0, odometry.theta);
        
        // Timestamp for odometry and transforms
        auto timestamp = this->get_clock()->now();

        // Create and send transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = timestamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = odometry.x_pos;
        transform.transform.translation.y = odometry.y_pos;
        transform.transform.translation.z = 0.0325; // Fixed height of the robot
        transform.transform.rotation = robot_orientation;

        tf_broadcaster_->sendTransform(transform);

        // Create and publish odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Pose
        odom_msg.pose.pose.position.x = odometry.x_pos;
        odom_msg.pose.pose.position.y = odometry.y_pos;
        odom_msg.pose.pose.position.z = 0.0325; // Same fixed height
        odom_msg.pose.pose.orientation = robot_orientation;

        // Twist
        odom_msg.twist.twist.linear.x = odometry.v;
        odom_msg.twist.twist.angular.z = odometry.w;

        // printState(robot_.getState(), odometry);

        odom_publisher_->publish(odom_msg);
    }

    // ROS 2 Publishers and Subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Control variables
    float linear_;
    float angular_;

    // PID parameters
    float kp1_;
    float ki1_;
    float kd1_;

    // Encoders and Robot
    int pigpio_handle_;
    RobotPins robot_pins_;
    // std::unique_ptr<Encoder> left_encoder_;
    // std::unique_ptr<Encoder> right_encoder_;
    std::unique_ptr<dual_encoder> dual_encoders_;
    int left_encoder_pulses_ = 0;
    int right_encoder_pulses_ = 0;
    Robot robot_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
