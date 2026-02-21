# **t21_navigation**
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

Пакет предназначен для проецирования среза облака точек на плоскость 2D-скана и использования slam_toolbox и nav2.

---

#  [Структура проекта](#оглавление)

```bash
src/t21_navigation/
├── behavior_trees              # Файлы для старых версий nav2
├── CMakeLists.txt              
├── config                      
│   ├── navigation_sim.yaml     # Настройка nav2 для симуляции
│   ├── navigation.yaml         # Настройка nav2 для реального робота
│   └── slam.yaml               # Настройка slam_toolbox
├── launch                      #
│   ├── navigation.launch.py    # Запуск nav2
│   ├── slam.launch.py          # Запуск slam_toolbox
│   └── translate.launch.py     # Запуск проецирования облака точек на 2D-скан (poitcloud_to_laserscan)
├── maps                        # Директория для сохранения карт
├── package.xml                 
├── README.md                   # <Вы находитесь здесь>
├── rviz                        # Для конфига rviz
└── src                         
    └── ground_filter_node.cpp  # Кастомная нода для предобработки облака точек, не рекомендуется к использованию
```

---

# [Зависимости](#оглавление)

```bash
sudo apt install ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-nav-2d-msgs \
    ros-${ROS_DISTRO}-nav-2d-utils \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-nav2-amcl \
    ros-${ROS_DISTRO}-nav2-behavior-tree \
    ros-${ROS_DISTRO}-nav2-behaviors \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-bt-navigator \
    ros-${ROS_DISTRO}-nav2-collision-monitor \
    ros-${ROS_DISTRO}-nav2-common \
    ros-${ROS_DISTRO}-nav2-constrained-smoother \
    ros-${ROS_DISTRO}-nav2-controller \
    ros-${ROS_DISTRO}-nav2-core \
    ros-${ROS_DISTRO}-nav2-costmap-2d \
    ros-${ROS_DISTRO}-nav2-dwb-controller \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-nav2-mppi-controller \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-navfn-planner \
    ros-${ROS_DISTRO}-nav2-planner \
    ros-${ROS_DISTRO}-nav2-regulated-pure-pursuit-controller \
    ros-${ROS_DISTRO}-nav2-rotation-shim-controller \
    ros-${ROS_DISTRO}-nav2-route \
    ros-${ROS_DISTRO}-nav2-rviz-plugins \
    ros-${ROS_DISTRO}-nav2-simple-commander \
    ros-${ROS_DISTRO}-nav2-smac-planner \
    ros-${ROS_DISTRO}-nav2-smoother \
    ros-${ROS_DISTRO}-nav2-theta-star-planner \
    ros-${ROS_DISTRO}-nav2-util \
    ros-${ROS_DISTRO}-nav2-velocity-smoother \
    ros-${ROS_DISTRO}-nav2-voxel-grid \
    ros-${ROS_DISTRO}-nav2-waypoint-follower \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-msgs \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-perception-pcl
```

---

# [Настройка](#оглавление)

Прежде всего необходимо убедиться, что проецирование облака точек работает корректно. Пакет может не видеть входного топика -- пока решение не найдено, но может помочь переустановка пакетов pcl-*.

Далее производится настройка slam_toolbox и nav2.

---

# [Использование](#оглавление)

Запуск проецирования (по умолчанию use_sim_time:=false):

```bash
ros2 launch t21_navigation translate.launch.py
```

Запуск slam_toolbox (по умолчанию use_sim_time:=false rviz:=false):

```bash
ros2 launch t21_navigation slam.launch.py
```
Используется online_async режим картографирования.


Запуск nav2 (по умолчанию use_sim_time:=false):

```bash
ros2 launch t21_navigation navigation.launch.py
```

---

# [Сохранение карты](#оглавление)

Для сохранения карты удобнее всего настроить в rviz отображение панели slam_toolbox и указать название карты. Будут сохранены pgm-изображение и yaml-описание OccupancyGrid карты.

---

# [TODO](#оглавление)
- Обновить launch-файлы
- Добавить фильтрацию облака точек или вынести в отдельных пакет
- Добавить советы по отладке и комментарии к конфигам  

---