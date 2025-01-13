cmake_minimum_required(VERSION 3.8)
project(diff_drive_rpi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(amaterasu_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(${PROJECT_NAME}
  main.cpp
  src/dc_motor.cpp 
  src/encoder.cpp 
  src/pid_controller.cpp 
  src/robot.cpp
  )

target_include_directories(${PROJECT_NAME} PRIVATE include/diff_drive_rpi)
#target_link_libraries(diff_drive_rpi PUBLIC )
# Link ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  amaterasu_interfaces
  tf2_ros
)

# Link pigpio library
target_link_libraries(${PROJECT_NAME} pigpiod_if2)

install(TARGETS diff_drive_rpi
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
