#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdometryTFBroadcaster : public rclcpp::Node {
public:
    OdometryTFBroadcaster()
        : Node("odometry_tf_broadcaster"),
          tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)) {
        
        // Subscription to the filtered odometry topic
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 
            10, 
            std::bind(&OdometryTFBroadcaster::odomCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Odometry TF Broadcaster Node has started.");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Create a TransformStamped message
        geometry_msgs::msg::TransformStamped transform;

        // Set header
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = msg->header.frame_id;  // Example: "odom"
        transform.child_frame_id = msg->child_frame_id;    // Example: "base_link"

        // Set translation
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;

        // Set rotation (orientation)
        transform.transform.rotation = msg->pose.pose.orientation;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform);

        RCLCPP_INFO(this->get_logger(), 
                    "Broadcasting transform from %s to %s",
                    transform.header.frame_id.c_str(),
                    transform.child_frame_id.c_str());
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
