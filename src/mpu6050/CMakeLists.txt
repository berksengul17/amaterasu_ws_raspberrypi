cmake_minimum_required(VERSION 3.8)
project(mpu6050)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Define executable
add_executable(mpu6050 src/mpu6050driver.cpp src/mpu6050sensor.cpp)
ament_target_dependencies(mpu6050 rclcpp sensor_msgs)

# Link libraries
target_link_libraries(mpu6050 i2c)

# Include directories
target_include_directories(
  mpu6050 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS mpu6050
  DESTINATION lib/${PROJECT_NAME}
)

# Install other necessary files
install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
)

# Export package
ament_package()
