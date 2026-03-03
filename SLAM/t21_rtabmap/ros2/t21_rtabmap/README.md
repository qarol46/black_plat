# **t21_rtabmap**
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

Пакет предназначен для запуска драйвера камеры Realsense и RTAB-Map. 

---

#  [Структура проекта](#оглавление)

```bash
src/t21_rtabmap/
├── CMakeLists.txt
├── config 
│   ├── camera.yaml # Параметры камеры
│   ├── champ # Директория с параметрами примера champ для сравнения
│   ├── icp_odom_params.yaml # Далее идут параметры различных узлов rtabmap 
│   ├── real_rtabmap.yaml
│   ├── rgbd_odom_params.yaml
│   ├── rgbd_sync_params.yaml
│   ├── rtabmap_nav2_params.yaml
│   ├── rtabmap_params.yaml
│   ├── rtabmap_viz.yaml
│   └── rtabmap.yaml
├── launch
│   ├── camera.launch.py
│   ├── t21_sim_vslam.launch.py
│   └── t21_vslam.launch.py
├── package.xml
└── README.md # <Вы находитесь здесь>
```

---

# [Зависимости](#оглавление)

### Realsense
Для работы пакета необходимо установить драйвера для камер Realsense. Для работы самой камеры нужен [Realsense SDK](https://github.com/realsenseai/librealsense/releases). Основные зависимости:

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```

Для взаимодействия с ROS нужны следующие пакеты:
```bash
sudo apt install ros-${ROS_DISTRO}-librealsense2 \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-camera-msgs \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-depthimage-to-laserscan \
    ros-${ROS_DISTRO}-camera-calibration-parsers \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-object-recognition-msgs \

```
### RTAB-Map
Можно установить как deb-пакеты:
```bash
sudo apt install ros-${ROS_DISTRO}-rtabmap \
    ros-${ROS_DISTRO}-rtabmap-ros \

```
Остальные зависимости будут установлены автоматически.

Но в таком случае **пакет не будет работать с камерами Realsense**. Рекомендуется устанавливать из исходного кода:

```bash
sudo apt remove ros-$ROS_DISTRO-rtabmap* # Убедитесь, что удалены все пакеты, которые могут вызвать конфликты
mkdir -p ~/rtabmap/src
cd ~/rtabmap/src
git clone -b humble-devel https://github.com/introlab/rtabmap.git src/rtabmap
git clone -b ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
rosdep update && rosdep install --from-paths src --ignore-src -r -y
export MAKEFLAGS="-j6" # Обязательный флаг для сборки пакета и предотвращения вылета системы
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

```

---

# [Настройка](#оглавление)

### Realsense
Описание параметров приведено в конфигурационном файле.

### RTAB-Map
Каждый из узлов имеет 100 и более параметров, что сильно усложняет их калибровку. Рекомендуется смотреть на настройки примера champ и калибровать параметры в файле rtabmap.yaml. Если не будет достигнута нужная стабильность системы, необходимо будет производить настройку для каждого узла и изменять параметры в соответствующих файлах.

---

# [Использование](#оглавление)

### Realsense

```bash
ros2 launch t21_rtabmap camera.launch.py
```

### RTAB-Map

Запуск в режиме симуляции:

```bash
ros2 launch t21_rtabmap t21_sim_vslam.launch.py
```

Запуск на реальном роботе:

```bash
ros2 launch t21_rtabmap t21_vslam.launch.py
```

---

# [Сохранение карты](#оглавление)

Производится через gui-интерфейс.

---

# [TODO](#оглавление)
- Калибровка параметров для стабильной работы.

---