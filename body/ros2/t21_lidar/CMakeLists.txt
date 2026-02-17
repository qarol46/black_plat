cmake_minimum_required(VERSION 3.5)
project(t21_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Установка launch- и конфигурационных файлов
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Установка YAML-файлов калибровки
install(
  FILES config/VLP16-velodyne_driver_node-params.yaml
        config/VLP16-velodyne_transform_node-params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(
  rclcpp_lifecycle
  velodyne_driver
  velodyne_pointcloud
)

ament_package()