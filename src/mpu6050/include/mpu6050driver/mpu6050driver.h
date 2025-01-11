#ifndef MPU6050DRIVER_H
#define MPU6050DRIVER_H

#include "mpu6050driver/mpu6050sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class MPU6050Driver : public rclcpp::Node {
 public:
  MPU6050Driver();

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::unique_ptr<MPU6050Sensor> mpu6050_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;

   // Calibration Tracking
  std::chrono::steady_clock::time_point last_calibration_time_;
  size_t measurement_counter_;
  const size_t calibration_threshold_count_ = 1000;
  const std::chrono::seconds calibration_time_interval_ = std::chrono::seconds(10);

  void handleInput();
  void declareParameters();
  void checkAndCalibrate();
};

#endif  // MPU6050DRIVER_H