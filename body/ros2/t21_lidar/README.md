[***ВЕРНУТЬСЯ***](/README.md)

# **t21_lidar**
# Оглавление

- [Цель](#цель)
- [Структура проекта](#структура-проекта)
- [Настройка](#настройка)
- [Зависимости](#зависимости)
- [Использование](#использование)

---

## Цель

Пакет предназначен для запуска ROS-драйвера лидара VLP-16. Выполнен на базе [velodyne ros-driver](https://github.com/ros-drivers/velodyne).

---

#  [Структура проекта](#оглавление)

```bash
body
├── docker
│   └── Dockerfile
├── ros2
│   ...
│    └──t21_lidar/
│       ├── CMakeLists.txt
│       ├── config
│       │   ├── VLP16-velodyne_driver_node-params.yaml    # Настройка параметров работы лидара
│       │   └── VLP16-velodyne_transform_node-params.yaml # Настройка параметров обработки облака точек
│       ├── launch
│       │   └── t21_lidar.launch.py # Запуск
│       ├── package.xml
│       └── README.md # <Вы находитесь здесь>
```

---

# [Настройка](#оглавление)

Перед началом работы необходимо подключить VLP-16 к ПК, подключиться к лидару по IP. Параметры:

- **Address:** `192.168.1.180`;
- **Netmask:** `255.255.255.0`;
- **Веб‑интерфейс:** `http://192.168.1.201/`.

---

# [Зависимости](#оглавление)

```bash
sudo apt install ros-${ROS_DISTRO}-velodyne ros-${ROS_DISTRO}-velodyne-driver ros-${ROS_DISTRO}-velodyne-pointcloud ros-${ROS_DISTRO}-velodyne-msgs ros-${ROS_DISTRO}-point-cloud-interfaces ros-${ROS_DISTRO}-point-cloud-msg-wrapper ros-${ROS_DISTRO}-velodyne-laserscan
```

---

# [Использование](#оглавление)
### Запуск внутри контейнера
Запуск производится внутри сервиса `body` — либо через `Makefile`, либо напрямую через `docker compose`. 

### Запуск вне контейнера

```bash
ros2 launch t21_lidar t21_lidar.launch.py
```

---

[***ВЕРНУТЬСЯ***](/README.md)