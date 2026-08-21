cmake_minimum_required(VERSION 3.8)
project(clahe_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Сначала создаём цель — потом к ней привязываем всё остальное
add_executable(clahe_ros_node
  src/clahe_ros_node.cpp
  src/clahe_ros.cpp
)

# Путь к заголовкам: include/clahe_ros/clahe_ros.h
target_include_directories(clahe_ros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(clahe_ros_node
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

target_link_libraries(clahe_ros_node
  ${OpenCV_LIBS}
)

install(TARGETS clahe_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()