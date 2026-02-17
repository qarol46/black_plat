# **Черная платформа**

![youbot](../../materials/image.png)

## Оглавление

- [Цель](#цель)
- [Описание каталогов](#описание-каталогов)
- [Зависимости](#зависимости)
- [Сборка рабочего пространства](#сборка-рабочего-пространства)
- [Запуск модели в RViz2](#запуск-модели-в-rviz2)
---

# Цель

Этот репозиторий содержит минимальное описание URDF/Xacro и базовую инфраструктуру запуска для небольшой гусеничной платформы с подвижной геометрией (одна степень подвижности на всю пару). Предназначен в качестве отправной точки для проектов на ROS 2 Humble.

---
#  [Описание каталогов ](#оглавление)

```bash
t21_ws/src/tracked_description
├── CMakeLists.txt                # скрипт ament_cmake
├── package.xml                   # зависимости
├── mesh/                         # STL‑модели
│   ├── base.stl
│   └── fliper.stl
├── urdf/
│   └── tracked_robot.urdf.xacro  # описание робота (xacro)
├── launch/
│   └── display.launch.py         # быстрый запуск визуализации
├── config/
│   └── rviz.rviz                 # конфигурационный RViz2
└── src/
    └── dummy.cpp                 # заглушка‑нода
```

---

# [Зависимости](#оглавление)

- ROS 2 Humble

- xacro, urdf, robot_state_publisher

- joint_state_publisher_gui (опционально, включён по умолчанию)

- RViz2 

Установите пакеты из стандартных репозиториев Ubuntu 22.04:

```bash
sudo apt update && \
  sudo apt install ros-humble-xacro \
                       ros-humble-robot-state-publisher \
                       ros-humble-joint-state-publisher-gui \
                       ros-humble-rviz2 \
                       sudo apt install ros-humble-rtabmap-ros
```

---

# [Сборка рабочего пространства](#оглавление)

```bash
# создаём (или используем существующее) рабочее пространство
mkdir -p ~/t21_ws/src
cd ~/t21_ws

# клонируем пакет в src/
git clone git@github.com:dakolzin/black_plat.git 

# сборка
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# инициализируем рабочее пространство
source install/setup.bash
```

---

# [Запуск модели в RViz2](#оглавление)

```bash
ros2 launch tracked_description display.launch.py
```

В RViz2 вы увидите:

1. base_link — основное гусеничное шасси (серое);

2. flip — меш флиппера, соединённый вращательным geom_joint вокруг оси Y
(пределы: 10° … 340°). Перемещайте слайдер GUI, чтобы проверить движение.

# [Система управления на ros2_control](#оглавление)

Для запуска визуализации и системы управления выполняем в терминале:
```bash
ros2 launch tracked_description bringup_t21.launch.py
```

Для управления с джойстика запускаем в новом терминале:
```bash
ros2 launch t21_teleop joy_full_teleop.launch.py
```

При работе с реальным роботом строчки ip-адреса и порта должны быть такими:
```cpp
  EthTrackedSocket(const char *listen_ip  = "0.0.0.0",
                   uint16_t     listen_prt = 4001,
                   const char *remote_ip  = "192.168.3.5",
                   uint16_t     remote_prt = 4001);
```

Для работы с пакетом без робота, необходимо запустить эхо-сервер:
```bash
ros2 run tracked_description dummy
```