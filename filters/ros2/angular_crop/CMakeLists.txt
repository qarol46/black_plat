cmake_minimum_required(VERSION 3.8)
project(angular_crop)

# Default to C++17 as it is standard for modern ROS 2 distributions
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. Define the executable
# I have used "angular_crop_node" consistently here to match common ROS 2 naming conventions
add_executable(angular_crop_node src/angular_crop_node.cpp)

# 3. Add dependencies to the target
ament_target_dependencies(angular_crop_node 
  rclcpp 
  sensor_msgs
)

# 4. Include directories (Uncomment if you have a header folder)
# target_include_directories(angular_crop_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# 5. Install the executable so 'ros2 run' can find it
install(TARGETS angular_crop_node
  DESTINATION lib/${PROJECT_NAME})

# 6. Testing and Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 7. Package export
ament_package()