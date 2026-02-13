# **t21_sim**
# Оглавление

- [Цель](#цель)
- [Структура проекта](#структура-проекта)
- [Зависимости](#зависимости)
- [Настройка](#настройка)
- [Использование](#использование)
- [Сохранение карты](#сохранение-карты)
- [TODO](#TODO)

---

## Цель

Пакет предназначен для запуска симуляции в Gazebo Classic (поддержка остановлена в 2025).

---

#  [Структура проекта](#оглавление)

```bash
src/t21_sim/
├── CMakeLists.txt
├── config
│   ├── description.rviz
│   ├── gazebo_params.yaml # Настройки симулятора
│   ├── my_controllers.yaml # Настройки контроллера в симуляции
│   ├── odometry_fus.yaml
│   └── t21.ros2_control.xacro # Описание контроллеров в симуляции (будет удалено)
├── launch
│   ├── launch_sim.launch.py # Запуск симуляции
│   └── rsp.launch.py # Запуск публикации положения робота
├── meshes # Низкополигональные модели робота
│   ├── base_link.STL
│   ├── big_box.stl
│   ├── fliper_link.STL
│   ├── left_wheel_link.STL
│   └── right_wheel_link.STL
├── package.xml
├── README.md # <Вы находитесь здесь>
├── urdf # Описание робота (будет удалено)
│   ├── camera.xacro
│   ├── depth_camera.xacro
│   ├── imu.xacro
│   ├── ros2_control.xacro
│   ├── t21_ros2_control.xacro
│   ├── t21.urdf.xacro
│   └── tracked_robot.urdf.xacro
└── worlds # Файлы виртуальной среды, где будет действовать робот
```

---

# [Зависимости](#оглавление)

```bash
sudo apt install ros-${ROS_DISTRO}-admittance-controller \
    ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-controller-manager-msgs \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-control-test-assets \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-kinematics-interface \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-range-sensor-broadcaster \
    ros-${ROS_DISTRO}-costmap-queue \
    ros-${ROS_DISTRO}-joint-limits \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joint-trajectory-controller
    gazebo \
    gazebo-common \
    gazebo-plugin-base \
    libgazebo-dev \
    ros-${ROS_DISTRO}-gazebo-dev \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros2-control
```

---

# [Настройка](#оглавление)

Выбор виртуальной среды осуществляется через launch-файл заменой файла gazebo_world_file.

---

# [Использование](#оглавление)

```bash
ros2 launch t21_sim launch_sim.launch.py
```

---

# [TODO](#оглавление)
- Обновить launch-файлы
- Удалить описание робота

---