# **odometry_fus**
# Оглавление

- [Цель](#цель)
- [Структура проекта](#структура-проекта)
- [Настройка](#настройка)
- [Примеры запуска нод](#примеры-запуска-нод)
- [Зависимости](#зависимости)
- [TODO](#TODO)

---

## Цель

Преданазначен для получения комплексированной одометрии(ros2_control + IMU), а также отправки управляющих команд для прямолинейного движения и повротов на заданное расстояние/угол. 

---


#  [Структура проекта](#оглавление)

```bash
src/odometry_fus/
├── CMakeLists.txt
├── package.xml
├── README.md # <Вы находитесь здесь>
└── src
│   ├── motion_emulator.cpp # нода для отправки управляющих команд для прямолинейного движения и поворота на заданное расстояние/угол
│   └── odometry_fus.cpp # нода для слияния одометрии(ros2_control + IMU)


```

---

# [Настройка](#оглавление)
В ноде odometry_fus можно настроить следующие параметры(также есть вариант реализации передачи данных параметров через yaml):
  
  odom_topic (string, default: /diff_drive_controller/odom) - топик колесной одометрии
  
  imu_topic (string, default: /imu/data) - топик данных IMU
  
  output_topic (string, default: /odom) - выходной топик сфьюженной одометрии
  
  child_frame (string, default: base_link) - дочерний фрейм для TF
  
  world_frame (string, default: odom) - родительский фрейм для TF
  
  publish_tf (bool, default: true) - публиковать ли TF-трансформации
  
  min_speed (double, default: 0.001) - минимальная скорость для учета движения
  
  min_angular_speed (double, default: 0.025) - порог угловой скорости для определения поворота
  
---
# [Примеры запуска нод](#оглавление)
Запуск ноды odometry_fus с параметрами по умолчанию:
```
  ros2 run odometry_fus odometry_fusion
```
Запуск ноды odometry_fus с передачей параметров:
```
ros2 run odometry_fus odometry_fusion --ros-args \
  -p odom_topic:=/wheel_odom \
  -p imu_topic:=/imu_sensor/data \
  -p output_topic:=/fused_odom
  -p min_speed:=0.0005 \
  -p min_angular_speed:=0.01
```
Запуск ноды motion_emulator для прямолинейного движения вперёд на 2 м и поворота на 45 градусов:
```
  ros2 run odometry_fus motion_emulator 2.0 45.0
```

---
# [Зависимости](#оглавление)
Будет дополнено

---

# [TODO](#оглавление)
Дописать данный файл

---

