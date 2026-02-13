# **t21_lidar**
# Оглавление

- [Цель](#цель)
- [Структура проекта](#структура-проекта)
- [Настройка](#настройка)
- [Зависимости](#зависимости)
- [Использование](#использование)

---

## Цель

Пакет предназначен для запуска ROS-драйвера лидара VLP-16. Выполнен на базе velodyne ros-driver.

---

#  [Структура проекта](#оглавление)

```bash
src/t21_lidar/
├── CMakeLists.txt
├── config
│   ├── VLP16-velodyne_driver_node-params.yaml    # Настройка параметров работы лидара
│   └── VLP16-velodyne_transform_node-params.yaml # Настройка параметров обработки облака точек
├── launch
│   └── t21_lidar.launch.py # Запуск
├── package.xml
└── README.md # <Вы находитесь здесь>
```

---

# [Настройка](#оглавление)

Перед началом работы необходимо подключить VLP-16 к ПК, подключиться к лидару по IP. Настройка угла обзора производиться именно через веб-интерфейс. Например, для задания угла обзора от -3pi/4 до 3pi/4 необходимо указать углы 225 - 135.

---

# [Зависимости](#оглавление)

```bash
sudo apt install ros-${ROS_DISTRO}-velodyne ros-${ROS_DISTRO}-velodyne-driver ros-${ROS_DISTRO}-velodyne-pointcloud ros-${ROS_DISTRO}-velodyne-msgs ros-${ROS_DISTRO}-point-cloud-interfaces ros-${ROS_DISTRO}-point-cloud-msg-wrapper ros-${ROS_DISTRO}-velodyne-laserscan
```

---

# [Использование](#оглавление)

Запуск:

```bash
ros2 launch t21_lidar t21_lidar.launch.py
```

---