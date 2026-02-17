cmake_minimum_required(VERSION 3.8)
project(tracked_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── DEPENDENCIES ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 

find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diff_drive_controller REQUIRED)

# ── EXECUTABLE dummy ─────────────────────────────────────────────────────────
add_executable(dummy src/dummy.cpp)
target_compile_features(dummy PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(dummy
  rclcpp
  std_msgs
  geometry_msgs
  xacro urdf robot_state_publisher ros2_control
  controller_interface diff_drive_controller)

# ── EXECUTABLE cmd_vel_to_float ──────────────────────────────────────────────
add_executable(cmd_vel_to_float src/cmd_vel_to_float.cpp)
target_compile_features(cmd_vel_to_float PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(cmd_vel_to_float
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs)

# ── INSTALL ------------------------------------------------------------------
install(TARGETS dummy cmd_vel_to_float
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch urdf mesh config
  DESTINATION share/${PROJECT_NAME})

# ── TESTS / LINT --------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
