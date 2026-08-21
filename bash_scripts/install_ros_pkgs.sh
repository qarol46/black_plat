#!/bin/bash

# Скрипт для установки пакетов ROS 2 Humble
# Проверяем, что запущено от root или с sudo
if [ "$EUID" -ne 0 ]; then 
    echo "Пожалуйста, запустите скрипт с sudo: sudo $0"
    exit 1
fi

# Обновляем список пакетов
apt-get update

# Блок 1: Пакеты управления и контроллеров
echo "Установка пакетов управления и контроллеров..."
apt-get install -y \
    ros-humble-admittance-controller \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-control-msgs \
    ros-humble-control-toolbox \
    ros-humble-controller-interface \
    ros-humble-controller-manager \
    ros-humble-controller-manager-msgs \
    ros-humble-ros2-control \
    ros-humble-ros2-control-test-assets \
    ros-humble-ros2-controllers \
    ros-humble-hardware-interface \
    ros-humble-diff-drive-controller \
    ros-humble-effort-controllers \
    ros-humble-kinematics-interface \
    ros-humble-velocity-controllers \
    ros-humble-range-sensor-broadcaster \
    ros-humble-costmap-queue \
    ros-humble-joint-limits \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joint-trajectory-controller

# Блок 2: Пакеты Gazebo
echo "Установка пакетов Gazebo..."
apt-get install -y \
    gazebo \
    gazebo-common \
    gazebo-plugin-base \
    libgazebo-dev \
    ros-humble-gazebo-dev \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control

# Блок 3: Пакеты для работы с изображениями
echo "Установка пакетов для работы с изображениями..."
apt-get install -y \
    ros-humble-image-geometry \
    ros-humble-image-tools \
    ros-humble-image-transport

# Блок 4: IMU-пакеты
echo "Установка IMU-пакетов..."
apt-get install -y \
    ros-humble-imu-complementary-filter \
    ros-humble-imu-filter-madgwick \
    ros-humble-imu-sensor-broadcaster \
    ros-humble-imu-tools

# Блок 5: Пакеты SLAM и навигации
echo "Установка пакетов SLAM и навигации..."
apt-get install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-cartographer-ros-msgs \
    ros-humble-slam-toolbox \
    ros-humble-nav-2d-msgs \
    ros-humble-nav-2d-utils \
    ros-humble-nav-msgs \
    ros-humble-nav2-amcl \
    ros-humble-nav2-behavior-tree \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-bringup \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-collision-monitor \
    ros-humble-nav2-common \
    ros-humble-nav2-constrained-smoother \
    ros-humble-nav2-controller \
    ros-humble-nav2-core \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-dwb-controller \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-mppi-controller \
    ros-humble-nav2-msgs \
    ros-humble-nav2-navfn-planner \
    ros-humble-nav2-planner \
    ros-humble-nav2-regulated-pure-pursuit-controller \
    ros-humble-nav2-rotation-shim-controller \
    ros-humble-nav2-route \
    ros-humble-nav2-rviz-plugins \
    ros-humble-nav2-simple-commander \
    ros-humble-nav2-smac-planner \
    ros-humble-nav2-smoother \
    ros-humble-nav2-theta-star-planner \
    ros-humble-nav2-util \
    ros-humble-nav2-velocity-smoother \
    ros-humble-nav2-voxel-grid \
    ros-humble-nav2-waypoint-follower \
    ros-humble-navigation2

# Блок 6: Пакеты RealSense
echo "Установка пакетов RealSense..."
apt-get install -y \
    ros-humble-librealsense2 \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-camera-msgs \
    ros-humble-realsense2-description \
    ros-humble-depthimage-to-laserscan \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-object-recognition-msgs \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv 

# Блок 7: Пакеты локализации и state publisher
echo "Установка пакетов локализации..."
apt-get install -y \
    ros-humble-robot-localization \
    ros-humble-robot-state-publisher

# Блок 8: Пакеты для работы с командами движения
echo "Установка пакетов для работы с командами движения..."
apt-get install -y \
    ros-humble-twist-mux \
    ros-humble-twist-mux-msgs

# Блок 9: Пакеты для работы с лидарами и облаками точек
echo "Установка пакетов для работы с лидарами и облаками точек..."
apt-get install -y \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-laser-geometry \
    ros-humble-velodyne \
    ros-humble-velodyne-driver \
    ros-humble-velodyne-msgs \
    ros-humble-velodyne-pointcloud \
    ros-humble-point-cloud-interfaces \
    ros-humble-point-cloud-msg-wrapper \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-pcl-ros \
    ros-humble-perception-pcl

# Блок 10: Xacro
echo "Установка Xacro..."
apt-get install -y \
    ros-humble-xacro

# Блок 11: GTSAM-PPA
echo "Установка Georgia Tech Smoothing and Mapping library..."
# Добавляем PPA репозиторий
add-apt-repository -y ppa:borglab/gtsam-release-4.1

# Обновляем список пакетов после добавления PPA
apt-get update

# Устанавливаем GTSAM библиотеки
apt-get install -y \
    libgtsam-dev \
    libgtsam-unstable-dev

echo "Установка всех пакетов завершена!"

# Блок 12: Realsense

sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

apt-get install -y \
    librealsense2-dkms \
    librealsense2-utils