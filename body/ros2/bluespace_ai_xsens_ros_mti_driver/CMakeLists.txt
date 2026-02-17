cmake_minimum_required(VERSION 3.12)  # Updated minimum version
project(bluespace_ai_xsens_mti_driver)

# Modern CMake policies
cmake_policy(SET CMP0057 NEW)

# Default to C++14 (can be updated to 17 if needed)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

###########
## Build ##
###########

# Build Xsens libraries first
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscontroller/libxscontroller.a
         ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscommon/libxscommon.a
         ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xstypes/libxstypes.a
  COMMAND $(MAKE) -C ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
  COMMENT "Building Xsens libraries..."
)

add_custom_target(xsens_libs DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscontroller/libxscontroller.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscommon/libxscommon.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xstypes/libxstypes.a
)

# Main executable
add_executable(xsens_mti_node
  src/main.cpp
  src/xdainterface.cpp
  src/xdacallback.cpp
)

add_dependencies(xsens_mti_node xsens_libs)

target_include_directories(xsens_mti_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic
)

ament_target_dependencies(xsens_mti_node
  rclcpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_link_libraries(xsens_mti_node
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscontroller/libxscontroller.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xscommon/libxscommon.a
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/xspublic/xstypes/libxstypes.a
  pthread
  dl
)

#############
## Install ##
#############

install(TARGETS xsens_mti_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()