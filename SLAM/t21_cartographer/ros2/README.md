# **Cartographer**
# Оглавление

- [Цель](#цель)
- [Структура проекта](#структура-проекта)
- [Зависимости](#зависимости)
- [Настройка](#настройка)
- [Использование](#использование)
- [Сохранение карты](#сохранение-карты)

---

## Цель

Предназначен для запуска пакета 2D- и 2.5D-картографирования и локализации по данным лазерного дальномера и IMU. Поддерживает 2 режима работы, принимая на вход данные как /LaserScan, так и /PointCloud.

---

#  [Структура проекта](#оглавление)

```bash
src/t21_cartographer/
├── CMakeLists.txt
├── config
│   └── cartographer.lua # Параметры
├── launch
│   └── cartographer.launch.py # Запуск Cartographer
├── package.xml
└── README.md# # <Вы находитесь здесь>
```

---

# [Зависимости](#оглавление)

```bash
sudo apt install ros-${ROS_DISTRO}-cartographer ros-${ROS_DISTRO}-cartographer-ros ros-${ROS_DISTRO}-cartographer-ros-msgs 
```

---

# [Настройка](#оглавление)

Раздел будет дополнен.
На данный момент конфигурационный файл настроен для работы без нижнего уровня на реальном роботе. В симуляции карта наклонена относительно робота.

Код взят из следующих статей (ссылки на репозитории в них):
- https://ouster.com/insights/blog/building-maps-using-google-cartographer-and-the-os1-lidar-sensor

- https://www.waveshare.com/wiki/Cartographer_Map_Building

---

# [Использование](#оглавление)

Запуск:

```bash
ros2 launch t21_cartographer cartographer.launch.py
```

При необходимости указывается launch-аргумент use_sim_time (false по умолчанию):

```bash
ros2 launch t21_cartographer cartographer.launch.py use_sim_time:=true
```

---

# [Сохранение карты](#оглавление)

Для сохранения карты необходимо вызвать сервис:

```bash
  ros2 service call /write_state ./maps/map.pbstream
```

---
