# **Черная платформа**

## Оглавление

- [Цель](#цель)
- [Структура проекта](#структура-проекта)
- [Зависимости](#зависимости)
- [Сборка](#сборка)
---

## Цель

Этот репозиторий содержит наработки для построения системы автономной навигации гусеничной платформы. Основной целью проекта на текущий момент является построение системы, способной создавать точные трёхмерных карты окружающей среды и локализоваться по ним и данным сенсорных компонентов.

---
#  [Структура проекта](#оглавление)

bash_scripts - директория для bash-скриптов (управления, установки пакетов и т.д.)

[bluespace_ai_xsens_ros_mti_driver](src/bluespace_ai_xsens_ros_mti_driver/README.md) - пакет для запуска драйвера IMU XSENS.

[LIO-SAM](src/LIO-SAM/README.md) - пакет для запуска SLAM-алгоритма LIO-SAM.

[odometry_fus](src/odometry_fus/README.md) - пакет для получения комплесированной одометрии(ros2_control + IMU) и отправки управляющих команд для прямолинейного движения и поворотов на указанное расстоняние/угол.

[ros2_control] - пакет с hardware для реализации контроллера мобильной платформы с использованием фреймворка ros2_control.

[t21_cartographer](src/t21_cartographer/README.md) - пакет для запуска SLAM-алгоритма Cartographer.

[t21_lidar](src/t21_lidar/README.md) - пакет для запуска драйверов VLP-16.

[t21_navigation](src/t21_navigation/README.md) - пакет для запуска SLAM Toolbox, Nav2 и узла pointcloud_to_laserscan.

[t21_rtabmap](src/t21_rtabmap/README.md) - пакет для запуска драйвера Realsense и SLAM-алгоритма RTAB-Map.  

[t21_sim](src/t21_sim/README.md) - пакет симуляции мобильной платформы. 

[t21_teleop] - пакет управление реальной платформой.

[tracked_description](src/tracked_description/README.md) - основной пакет, содержащий описание робота и файлы запуска для реальной платформы.

[USR-DR134-GUI](https://github.com/dakolzin/USR-DR134-GUI.git) - пакет для отслеживания заряда аккумулятора. [Устанавливается из репозитория автора - dakolzin](https://github.com/dakolzin/USR-DR134-GUI.git).

velodyne_simulator - сторонний пакет, содержащий описание LiDAR VLP-16 и плагины для Gazebo. deb package на момент проверки содержал ошибку.

---

# [Зависимости](#оглавление)

Устанавливаются при сборке рабочего пространства. Отдельно перечислены при описании каждого из подпроектов. **Для [t21_rtabmap](src/t21_rtabmap/README.md) необходимо провести дополнительные установки - смотреть в описании.**

---

# [Сборка рабочего пространства](#оглавление)

```bash
# создаём (или используем существующее) рабочее пространство
mkdir -p ~/t21_ws/src
cd ~/t21_ws

# клонируем пакет в src/
git clone https://github.com/qarol46/black_plat.git

# устанавливаем необходимые ros-пакеты 
chmod +x bash_scripts/install_ros_pkgs.sh
sudo bash bash_scripts/install_ros_pkgs.sh

# сборка
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# инициализируем рабочее пространство
source install/setup.bash
```

---
