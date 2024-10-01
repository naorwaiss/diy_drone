cmake_minimum_required(VERSION 3.5)
project(drone_cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Required for ROS 2 C++ nodes
find_package(sensor_msgs REQUIRED)  # Add this line for sensor_msgs
find_package(std_msgs REQUIRED)  # If you use standard messages

# Add your executable
add_executable(roc_teensy_node src/roc_teensy_c++.cpp)

# Link the necessary libraries (e.g., rclcpp, sensor_msgs)
ament_target_dependencies(roc_teensy_node rclcpp std_msgs sensor_msgs)

# Install the executable
install(TARGETS
  roc_teensy_node
  DESTINATION lib/${PROJECT_NAME}
)

# Make the package ament-compatible
ament_package()

