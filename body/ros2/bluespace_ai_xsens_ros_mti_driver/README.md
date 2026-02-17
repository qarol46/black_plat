# **XSENS Driver**
# Оглавление

- [Цель](#цель)
- [Структура проекта](#структура-проекта)
- [Зависимости](#зависимости)
- [TODO](#TODO)

---

## Цель

Предназначен для запуска драйверов IMU XSENS MTi-G-710 GNSS/INS
---


#  [Структура проекта](#оглавление)

```bash
src/bluespace_ai_xsens_ros_mti_driver
├── CMakeLists.txt
├── launch
│   ├── display.launch.py # Визуализация данных
│   └── xsens_mti_node.launch.py # Основной файл запуска
├── lib # Интерфейсы
├── LICENSE.txt
├── package.xml
├── param
│   └── xsens_mti_node.yaml # Параметры узла драйвера #TODO добавить недостающие параметры
├── README.md # <Вы находитесь здесь>
├── README.txt # Оригинальный README
├── rviz # Параметры для запуска RViz
├── src  # Исполняемые файлы
└── urdf # URDF-описание и STl-модели

```
---

# [Зависимости](#оглавление)

Требует основные пакеты ROS2 -> см. package.xml
---

# [TODO](#оглавление)
На данный момент для задания параметра `parent_id` необходимо в файле [packetcallback.h](src/messagepublishers/packetcallback.h) изменить соответствующее значение. В дальнейшем баг будет исправлен.

---
