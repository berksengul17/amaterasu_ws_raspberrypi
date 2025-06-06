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

#define MOTOR_PPR 39.0f
#define SAMPLE_TIME_MS 30
#define PWM_FREQUENCY 800

class RobotNode : public rclcpp::Node {
public:
    RobotNode()
        : Node("robot_control_node"), linear_(0.0), angular_(0.0),
        startPose{0.0, 0.0}, initPose(false),
        robot_ns_(declare_parameter<std::string>("robot_ns", "")),
        kp1_(declare_parameter("kp", 0.05f)),
        ki1_(declare_parameter("ki", 0.0)),
        kd1_(declare_parameter("kd", 0.0f)),
        robot_pins_{{PWM_FREQUENCY, L_EN_PIN, FL_IN1_PIN, FL_IN2_PIN},
                    {PWM_FREQUENCY, R_EN_PIN, FR_IN1_PIN, FR_IN2_PIN},
                    {PWM_FREQUENCY, L_EN_PIN, RL_IN1_PIN, RL_IN2_PIN},
                    {PWM_FREQUENCY, R_EN_PIN, RR_IN1_PIN, RR_IN2_PIN}},
        robot_(wiringpi_handle_, kp1_, kd1_, ki1_, SAMPLE_TIME_MS, robot_pins_),
        sample_counter_(0), tick_sum_left_(0), tick_sum_right_(0),
        prev_ticks_left_(0), prev_ticks_right_(0)
    {

        // Publisher for odometry
        prefix_ = robot_ns_.empty() ? std::string("") : std::string("/") + robot_ns_;

        // namespaced odometry publisher:
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            prefix_ + "/odom", 10);
      
        // namespaced cmd_vel subscription:
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            prefix_ + "/cmd_vel", 10,
            std::bind(&RobotNode::cmdVelCallback, this, std::placeholders::_1));     

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
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
        front_left_encoder_ = std::make_unique<Encoder>(FL_ENC_PIN);
        front_right_encoder_ = std::make_unique<Encoder>(FR_ENC_PIN);
        rear_left_encoder_ = std::make_unique<Encoder>(RL_ENC_PIN);
        rear_right_encoder_ = std::make_unique<Encoder>(RR_ENC_PIN);

        // Initialize encoders
        front_left_encoder_->set_pulses(0);
        front_right_encoder_->set_pulses(0);
        rear_left_encoder_->set_pulses(0);
        rear_right_encoder_->set_pulses(0);

        front_left_encoder_->start();
        front_right_encoder_->start();
        rear_left_encoder_->start();
        rear_right_encoder_->start();

        RCLCPP_INFO(this->get_logger(), "Setup complete.");
    }

    void stopEncoders() {
        front_left_encoder_->stop();
        front_right_encoder_->stop();
        rear_left_encoder_->stop();
        rear_right_encoder_->stop();
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

        printf("Linear: %.2f | Angular: %.2f\n", linear_, angular_);
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
        int32_t fl_encoder_ticks = front_left_encoder_->get_pulses();
        int32_t fr_encoder_ticks = front_right_encoder_->get_pulses();
        int32_t rl_encoder_ticks = rear_left_encoder_->get_pulses();
        int32_t rr_encoder_ticks = rear_right_encoder_->get_pulses();

        robot_.setUnicycle(linear_, angular_);
        robot_.updatePid(fl_encoder_ticks, fr_encoder_ticks, rl_encoder_ticks, rr_encoder_ticks);

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
        odom_msg.pose.pose.position.z = 0.0; // Same fixed height
        odom_msg.pose.pose.orientation = robot_orientation;

        // Twist
        odom_msg.twist.twist.linear.x = odometry.v;
        odom_msg.twist.twist.angular.z = odometry.w;

        // printState(robot_.getState(), odometry);

        odom_publisher_->publish(odom_msg);

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = odometry.v;
        twist_msg.angular.z = odometry.w;

        // }
    }

    // ROS 2 Publishers and Subscribers
    std::string prefix_;
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
    std::string robot_ns_;

    // PID parameters
    float kp1_;
    float ki1_;
    float kd1_;

    OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    // Encoders and Robot
    int wiringpi_handle_;
    RobotPins robot_pins_;
    std::unique_ptr<Encoder> front_left_encoder_;
    std::unique_ptr<Encoder> front_right_encoder_;
    std::unique_ptr<Encoder> rear_left_encoder_;
    std::unique_ptr<Encoder> rear_right_encoder_;
    Robot robot_;

    int sample_counter_;
    int32_t tick_sum_left_;
    int32_t tick_sum_right_;
    int32_t prev_ticks_left_;
    int32_t prev_ticks_right_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}