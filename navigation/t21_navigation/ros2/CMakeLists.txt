cmake_minimum_required(VERSION 3.8)
project(t21_navigation)

cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Link libraries
target_link_libraries(ground_filter_node
  ${PCL_LIBRARIES}
)

# ROS 2 dependencies
ament_target_dependencies(ground_filter_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(DIRECTORY 
  launch 
  config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()