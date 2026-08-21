[***ВЕРНУТЬСЯ***](/README.md)

# **XSENS Driver**
# Оглавление

- [Цель](#цель)
- [Структура проекта](#структура-проекта)
- [Запуск](#запуск)
- [Настройка](#настройка)
- [Зависимости](#зависимости)
- [TODO](#TODO)

---

## [Цель](#оглавление)

Предназначен для запуска драйверов IMU XSENS MTi-G-710 GNSS/INS

---


##  [Структура проекта](#оглавление)

```bash
body
├── docker
│   └── Dockerfile
├── ros2
│   ...
│   └── bluespace_ai_xsens_ros_mti_driver
│        ├── CMakeLists.txt
│        ├── launch
│        │   ├── display.launch.py # Визуализация данных
│        │   └── xsens_mti_node.launch.py # Основной файл запуска
│        ├── lib # Интерфейсы
│        ├── LICENSE.txt
│        ├── package.xml
│        ├── README.md # <Вы находитесь здесь>
│        ├── README.txt # Оригинальный README
│        ├── rviz # Параметры для запуска RViz
│        ├── src  # Исполняемые файлы
│        └── urdf # URDF-описание и STl-модели

```
---
## [Запуск](#оглавление)

### Запуск внутри контейнера
Запуск производится внутри сервиса `body` — либо через `Makefile`, либо напрямую через `docker compose`. 

### Запуск вне контейнера

```bash
. install/setup.bash
ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py
```

---

## [Настройка](#оглавление)

В конфигурационном файле `/data/configs/body/imu_config/xsens_mti_node.yaml` задаётся фрейма публикации данных IMU и набора публикуемых данных. Для использования с ROS-пакетами достаточно оставить только `pub_imu: true`. Подробнее о других публикуемых данных можно посмотреть в настройках mtmanager.

Качество данных сенсора зависит от правильности указания связей в URDF. Если дерево преобразований задано некорректно, то и данные сенсора могут интерпретироваться неверно.
---

## [Зависимости](#оглавление)

Для работы с датчиком необходимо установить ПО от производителя:
[mtmanager](https://base.xsens.com/s/article/MT-Manager-Installation-Guide-for-ubuntu-20-04-and-22-04?language=en_US) --- GUI для вывода данных и настройки встроенного Фильтра Калмана у сенсора. 

[Объяснение шумовых параметров сенсора](https://base.xsens.com/s/article/Understanding-the-relationship-between-the-noise-characteristics?language=en_US), используемых SLAM-алгоритмами.

[Вычисление шумовых параметров сенсора](/filters/ros2/allan_variance/README.md).


Xsens Documentation ещё [тут](https://base.xsens.com/s/article/All-MTi-Related-Documentation-Links?language=en_US) и [тут](https://mtidocs.xsens.com/mti-system-overview-2).

ROS-пакеты:
```bash
sudo apt install ros-humble-tf2 ros-humble-tf2_ros ros-humble-std_msgs ros-humble-geometry_msgs ros-humble-sensor_msgs
```

---

[***ВЕРНУТЬСЯ***](/README.md)