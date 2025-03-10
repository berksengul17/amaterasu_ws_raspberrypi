#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "amaterasu_interfaces/msg/robot_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "robot.h"
#include "robot_pins.h"
#include "dc_motor.h"
#include "encoder.h"
#include <wiringPi.h>
#include <cmath>
#include <stdexcept>
#include <array>

#define MOTOR_PPR 20.0f
#define SAMPLE_TIME_MS 500
#define PWM_FREQUENCY 800

class RobotNode : public rclcpp::Node {
public:
    RobotNode()
        : Node("robot_control_node"), linear_(0.0), angular_(0.0),
        startPose{0.0, 0.0}, initPose(false),
        kp1_(declare_parameter("kp", 9.0)),
        ki1_(declare_parameter("ki", 0.03)),
        kd1_(declare_parameter("kd", 0.0)),
        robot_pins_{{PWM_FREQUENCY, L_ENA_PIN, L_IN1_PIN, L_IN2_PIN},
                    {PWM_FREQUENCY, R_ENB_PIN, R_IN3_PIN, R_IN4_PIN}},
        robot_(wiringpi_handle_, kp1_, kd1_, ki1_, SAMPLE_TIME_MS, robot_pins_)
    {

        // Publisher for odometry
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Subscription for cmd_vel
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&RobotNode::cmdVelCallback, this, std::placeholders::_1));
        robot_pos_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/robot/bounding_box", 10,
            std::bind(&RobotNode::robotCallback, this, std::placeholders::_1));

        // Timer for periodic updates
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(SAMPLE_TIME_MS),
            std::bind(&RobotNode::update, this));

        on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&RobotNode::parameterCallback, this, std::placeholders::_1));

        // Initialize hardware
        setup();
    }

    ~RobotNode() {
        stopEncoders();
    }

private:
    void setup() {
        // Initialize pigpio
        wiringpi_handle_ = wiringPiSetupPinType(WPI_PIN_BCM);
        if (wiringpi_handle_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize wiringpi.");
            throw std::runtime_error("Failed to initialize wiringpi.");
        }

        // Initialize encoders dynamically after wiringpi_handle_ is ready
        left_encoder_ = std::make_unique<Encoder>(L_ENC_PIN);
        right_encoder_ = std::make_unique<Encoder>(R_ENC_PIN);

        // Initialize encoders
        left_encoder_->set_pulses(0);
        right_encoder_->set_pulses(0);

        left_encoder_->start();
        right_encoder_->start();

        RCLCPP_INFO(this->get_logger(), "Setup complete.");
    }

    void stopEncoders() {
        left_encoder_->stop();
        right_encoder_->stop();
    }


    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;  // Default to successful unless an error occurs
        result.reason = "Parameters updated successfully.";

        for (const auto &parameter : parameters) {
            if (parameter.get_name() == "kp") {
                kp1_ = parameter.as_double();
                robot_.updatePidParams(kp1_, kd1_, ki1_);
                RCLCPP_INFO(this->get_logger(), "Updated kp: %.2f", kp1_);
            } else if (parameter.get_name() == "ki") {
                ki1_ = parameter.as_double();
                robot_.updatePidParams(kp1_, kd1_, ki1_);
                RCLCPP_INFO(this->get_logger(), "Updated ki: %.2f", ki1_);
            } else if (parameter.get_name() == "kd") {
                kd1_ = parameter.as_double();
                robot_.updatePidParams(kp1_, kd1_, ki1_);
                RCLCPP_INFO(this->get_logger(), "Updated kd: %.2f", kd1_);
            } else {
                result.successful = false;  // Mark failure if an unexpected parameter is encountered
                result.reason = "Unknown parameter: " + parameter.get_name();
            }
        }

        return result;
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
        // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear=%.2f, angular=%.2f", linear_, angular_);
    }

    void robotCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        if (!initPose) {
            startPose[0] = msg->x;
            startPose[1] = msg->y;
            initPose = true;
            RCLCPP_INFO(this->get_logger(), "Initial pose set: x = %.2f, y = %.2f", startPose[0], startPose[1]);
        }
    }

    void update() {
        // Update robot state
        robot_.setUnicycle(linear_, angular_);
        robot_.updatePid(left_encoder_->get_pulses(), right_encoder_->get_pulses());

        // Get the current robot state and odometry
        // auto state = robot_.getState();
        auto odometry = robot_.getOdometry();

        if (initPose) {
            odometry.x_pos += startPose[0];
            odometry.y_pos += startPose[1];
        }

        auto robot_orientation = quaternionFromEuler(0.0, 0.0, odometry.theta);
        
        // Timestamp for odometry and transforms
        auto timestamp = this->get_clock()->now();

        // Create and send transform
        // geometry_msgs::msg::TransformStamped transform;
        // transform.header.stamp = timestamp;
        // transform.header.frame_id = "odom";
        // transform.child_frame_id = "base_link";
        // transform.transform.translation.x = odometry.x_pos;
        // transform.transform.translation.y = odometry.y_pos;
        // transform.transform.translation.z = 0.0325; // Fixed height of the robot
        // transform.transform.rotation = robot_orientation;

        // tf_broadcaster_->sendTransform(transform);

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
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr robot_pos_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Control variables
    float linear_;
    float angular_;

    std::array<float, 2> startPose;
    std::atomic<bool> initPose;

    // PID parameters
    float kp1_;
    float ki1_;
    float kd1_;

    OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    // Encoders and Robot
    int wiringpi_handle_;
    RobotPins robot_pins_;
    std::unique_ptr<Encoder> left_encoder_;
    std::unique_ptr<Encoder> right_encoder_;
    Robot robot_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}