# **Черная платформа**

## Оглавление

- [Цель](#цель)
- [Аппаратная часть](#аппаратная-часть)
- [Структура проекта](#структура-проекта)
- [Зависимости и установка](#зависимости-и-установка)
- [Сборка](#сборка)
- [Запуск](#запуск)
- [Мониторинг через tmux](#мониторинг-через-tmux)
- [Известные проблемы](#известные-проблемы)

---

## Цель

Этот репозиторий содержит наработки для построения системы автономной навигации гусеничной платформы. Основной целью проекта на текущий момент является построение системы, способной создавать точные трёхмерные карты окружающей среды и локализоваться по ним с использованием данных сенсорных компонентов.

---

## Аппаратная часть

| Компонент | Описание |
|-----------|----------|
| Гусеничная платформа | Мобильная платформа с ros2_control |
| IMU | XSENS MTi 710 |
| LiDAR | Velodyne VLP-16 |
| Камера | Intel RealSense D435i |

---

## Структура проекта

Проект основан на контейнеризации с помощью `docker compose`. Каждый функциональный блок вынесен в отдельный сервис. Всего выделено 4 функциональных группы: `body`, `SLAM`, `navigation`, `sim` и `visualization`.

### Общая структура репозитория

```
black_plat/
├── bash_scripts/           # Bash-скрипты управления и установки пакетов
├── body/                   # Управление платформой и сенсорные данные
├── SLAM/                   # SLAM-алгоритмы
├── navigation/             # Автономная навигация
├── sim/                    # Симуляция
├── data/                   # Конфиги всех пакетов, описание робота, rosbag'и, карты
│   ├── configs/
│   │   ├── body/           # Конфиги сенсоров (IMU, LiDAR, камера)
|   |   ├── navigation/           
│   │   └── slam/           
│   ├── maps/
│   ├── mesh/
|   ├── tmux/               # Конфиг tmux → data/configs/tmux/tmux.conf
│   └── rosbags/
├── visualization/          # Инструменты визуализации
├── dds/                    # Конфигурация DDS (middleware ROS2)
├── materials/              # Вспомогательные материалы
├── docker-compose.yaml     # Корневой compose-файл
├── Makefile                # Основной make-файл для сборки и запуска
└── README.md
```

### Структура функционального блока

```
Имя_функциональной_директории/
├── docker-compose.yaml
└── Имя_пакета/
    ├── docker/
    │   ├── Dockerfile
    │   └── ros_entrypoint.sh
    └── ros2/
        └── Имя_пакета/
            └── Файлы_пакета
```

---

### body — управление платформой

Пакеты, относящиеся к управлению платформой и получению сенсорных данных.

- [bluespace_ai_xsens_ros_mti_driver](body/ros2/bluespace_ai_xsens_ros_mti_driver/README.md) — драйвер IMU XSENS MTi.
- [t21_lidar](body/ros2/t21_lidar/README.md) — драйверы Velodyne VLP-16.
- [t21_camera](body/ros2/t21_camera/README.md) — драйвер камеры.
- [ros2_control] — hardware-интерфейсы для управления платформой.
- [t21_teleop] — пакет телеуправления реальной платформой.
- [tracked_description](body/ros2/tracked_description/README.md) — описание робота (URDF/XACRO) и launch-файлы.
- [power_monitor](https://github.com/dakolzin/USR-DR134-GUI.git) — мониторинг заряда аккумулятора. [Репозиторий автора](https://github.com/dakolzin/USR-DR134-GUI.git).

---

### SLAM — алгоритмы построения карт

- [LeGO-LOAM](SLAM/LeGO-LOAM-ROS2/ros2/README.md) — SLAM на основе LiDAR.
- [t21_slam_toolbox](SLAM/t21_slam_toolbox/ros2/t21_slam_toolbox/README.md) — интеграция slam_toolbox.
- [LIO-SAM](SLAM/LIO_SAM/ros2/lio_sam/README.md) — LiDAR-Inertial Odometry SLAM.
- [t21_cartographer](SLAM/t21_cartographer/ros2/t21_cartographer/README.md) — Google Cartographer SLAM.
- [t21_rtabmap](SLAM/t21_rtabmap/ros2/t21_rtabmap/README.md) — RTAB-Map с поддержкой Realsense. ⚠️ **В данный момент не используется** из-за ошибки сборки контейнера.

---

### navigation — автономная навигация

- [t21_nav2](navigation/t21_nav2/ros2/README.md) — конфигурация и запуск Nav2.

---

### sim — симуляция

- [t21_sim](sim/ros2/t21_sim/README.md) — симуляция мобильной платформы в Gazebo.
- velodyne_simulator — LiDAR VLP-16 и плагины для Gazebo. ⚠️ deb-пакет содержит ошибку, рекомендуется сборка из исходников.

---

## Зависимости и установка

### На хосте необходимы

| Пакет | Версия | Установка |
|-------|--------|-----------|
| Docker Engine | ≥ 24.x | см. ниже |
| Docker Compose V2 | встроен в Docker | — |
| NVIDIA Container Toolkit | последняя | только при наличии GPU |
| make | любая | `sudo apt install make` |
| tmux | ≥ 3.2 | `sudo apt install tmux` |

### Установка Docker Desktop (Ubuntu)

```bash
sudo apt-get update
sudo apt install ./docker-desktop-amd64.deb
```

### Установка NVIDIA Container Toolkit (если используется GPU)

```bash
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list \
  | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Настройка tmux (опционально)

Конфигурационный файл находится в [`data/configs/tmux/tmux.conf`](data/configs/tmux/tmux.conf).
Для использования скопируй его в стандартное место:

```bash
mkdir -p ~/.config/tmux
cp data/configs/tmux/tmux.conf ~/.config/tmux/tmux.conf
```

---

## Сборка

```bash
# Создаём рабочее пространство
mkdir -p ~/t21_ws
cd ~/t21_ws

# Клонируем репозиторий
git clone -b compose https://github.com/qarol46/black_plat.git

# Добавляем переменную окружения (и в ~/.bashrc для постоянного эффекта)
export ROOT_DIR=~/t21_ws/black_plat/
echo "export ROOT_DIR=~/t21_ws/black_plat/" >> ~/.bashrc

# Собираем все образы
make build-all

# Или через docker compose напрямую
docker compose build
```

> ⚠️ Переменная `ROOT_DIR` должна быть объявлена в окружении до любого запуска через `make` или `docker compose`.

---

## Запуск

### Через Makefile

```bash
# Запустить весь стек (все сервисы из ALL_SERVICES)
make up-all
# Для вывода (или скрытия) логов нужно настроить переменную LOG_MODE в Makefile или 
make up-all LOG_MODE=false
# Запустить конкретную группу
make up-platform          # body + teleop
make up-slam              # lio-sam
make up-nav2              # nav2

# Запустить конкретные сервисы вручную
make up SERVICES="body lio-sam rviz2"

# Остановить всё
make down-all

# Посмотреть статус контейнеров
make ps

# Логи одного сервиса (в текущем терминале)
make logs SERVICE=lio-sam

# Открыть bash внутри контейнера
make shell SERVICE=body
```

### Через docker compose напрямую

```bash
# Запустить конкретные сервисы
docker compose up body lio-sam rviz2

# Запустить в фоне
docker compose up -d body lio-sam

# Остановить
docker compose stop body
docker compose down          # остановить и удалить контейнеры
```

---

## Мониторинг через tmux

`make monitor` запускает tmux-сессию, где каждый сервис из `SERVICES` открывается в отдельной панели с живыми логами. Дополнительно создаётся вкладка `shell` для управляющих команд.

### Быстрые сценарии

```bash
# Весь стек (ALL_SERVICES) горизонтально
make monitor

# Полный robot-стек (body + slam + nav2 + rviz2 + power_monitor) сеткой
make monitor-robot

# Только SLAM вертикально
make monitor-slam

# Только платформа
make monitor-platform

# Произвольный набор сервисов
make monitor SERVICES="body lio-sam nav2" LAYOUT=tiled SESSION=debug

# Запустить сервисы в фоне И сразу открыть мониторинг
make run-and-monitor SERVICES="body lio-sam rviz2"

# Остановить все сервисы и закрыть текущую сессию
make monitor-kill

# Список активных tmux-сессий
make monitor-ls
```

### Параметры `make monitor`

| Переменная | Описание | Значение по умолчанию |
|------------|----------|-----------------------|
| `SERVICES` | Список сервисов через пробел | `ALL_SERVICES` |
| `SESSION`  | Имя tmux-сессии | `black_plat` |
| `LAYOUT`   | Расположение панелей: `h` / `v` / `tiled` | `h` |

### Клавиши tmux (кастомный конфиг)

| Клавиша | Действие |
|---------|----------|
| `Alt + стрелки` | Переключение между панелями |
| `Alt + 1..9` | Переключение между окнами |
| `Alt + h` | Разделить панель горизонтально |
| `Alt + v` | Разделить панель вертикально |
| `Alt + Enter` | Новое окно |
| `Alt + c` | Закрыть панель |
| `Alt + q` | Закрыть окно |
| `Alt + d` | Detach от сессии (сессия остаётся в фоне) |
| `Alt + Q` | Завершить всю сессию (с подтверждением) |
| `Alt + s` | Дерево сессий |
| `Alt + r` | Перезагрузить конфиг tmux |
| `Alt + /` | Поиск вниз в логах (copy-mode) |
| `Alt + ?` | Поиск вверх в логах (copy-mode) |
| `Shift + drag` | Выделить текст мышью в буфер **терминала** |

> В режиме copy-mode: `v` — начать выделение, `y` — скопировать в буфер обмена (wl-copy / xclip).

---

## Известные проблемы

| Проблема | Статус |
|----------|--------|
| `t21_rtabmap` — ошибка сборки контейнера | 🔴 Не исправлено |
| `velodyne_simulator` deb-пакет содержит ошибку | 🟡 Workaround: сборка из исходников |
| Устаревшие описания пакетов | 🟡 Идёт исправление |