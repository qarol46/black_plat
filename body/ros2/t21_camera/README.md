[***ВЕРНУТЬСЯ***](/README.md)

# **t21_camera**
# Оглавление

- [Цель](#цель)
- [Структура проекта](#структура-проекта)
- [Настройка](#настройка)
- [Зависимости](#зависимости)
- [Использование](#использование)

---

## Цель

Пакет предназначен для запуска ROS-драйвера камеры глубины Realsense D435i.

---

#  [Структура проекта](#оглавление)

```bash
body
├── docker
│   └── Dockerfile
├── ros2
│   ...
│    └──t21_camera/
│       ├── CMakeLists.txt
│       ├── launch
│       │   └── t21_camera.launch.py
│       ├── package.xml
│       └── README.md # <Вы находитесь здесь>
```

---

# [Настройка](#оглавление)


[Конфигурационный файл](/data/configs/body/camera_config/camera.yaml) содержит описания всех параметров. Один из важных моментов - ```unite_imu_method``` отвечает за публикацию данных инерциального блока в один топик /imu/data (2).

---

# [Зависимости](#оглавление)

```bash
apt-get install -y ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs ros-humble-realsense2-description ros-humble-depthimage-to-laserscan ros-humble-camera-calibration-parsers  ros-humble-camera-info-manager ros-humble-object-recognition-msgs ros-humble-cv-bridge ros-humble-vision-opencv 
```

---

# [Использование](#оглавление)
### Запуск внутри контейнера
Запуск производится внутри сервиса `body` — либо через `Makefile`, либо напрямую через `docker compose`. 

### Запуск вне контейнера

```bash
ros2 launch t21_camera t21_camera.launch.py
```

---

[***ВЕРНУТЬСЯ***](/README.md)