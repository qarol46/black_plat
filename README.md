# **Черная платформа**

## Оглавление

- [Цель](#цель)
- [Аппаратная часть](#аппаратная-часть)
- [Структура проекта](#структура-проекта)
- [Зависимости и установка](#зависимости-и-установка)
- [Сборка](#сборка)
- [Подготовка платформы](#подготовка-платформы)
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
| IMU | XSENS MTi-G-710 |
| LiDAR | Velodyne VLP-16 |
| Камера | Intel RealSense D435i |

---

## Структура проекта

Проект основан на контейнеризации с помощью `docker compose`. Каждый функциональный блок вынесен в отдельный сервис. Всего выделено 6 функциональных группы: `body`, `SLAM`, `navigation`, `sim`, `visualization` и `filters`. Разбиение на блоки сделано для выделения общих функциональных групп в отдельные директории.

### Общая структура репозитория

```
black_plat/
├── bash_scripts/           # Bash-скрипты управления и установки пакетов
├── body/                   # Управление платформой и сенсорные данные
├── data/                   # Конфиги всех пакетов, описание робота, rosbag'и, карты
│   ├── configs/
│   │   ├── body/           # Конфиги сенсоров (IMU (в т.ч. прошивка .xsa), LiDAR, камера) и контроллера
|   |   ├── navigation/           
│   │   └── slam/           
│   ├── maps/
│   ├── mesh/
│   ├── rosbags/
│   ├── rviz2/
│   ├── scripts/
|   |   ├── flipper/
|   |   ├── glim/
|   |   └── imu/    
│   └── tmux/               # Конфиг tmux → data/configs/tmux/tmux.conf
├── dds/                    # Конфигурация DDS (middleware ROS2)
├── filters/                # Фильтры данных с сенсоров
├── materials/              # Вспомогательные материалы
├── navigation/             # Автономная навигация
├── sim/                    # Симуляция
├── SLAM/                   # SLAM-алгоритмы
├── visualization/          # Инструменты визуализации
├── docker-compose.yaml     # Корневой compose-файл
├── Makefile                # Основной make-файл для сборки и запуска
└── README.md               # <Вы находитесь здесь>
```

### Структура функционального блока

```
Имя_функциональной_директории/
├── docker-compose.yaml         # Файл, содержащий описание docker compose сервисов
└── Имя_пакета/
    ├── docker/
    │   ├── Dockerfile          # Докерфайл для данного блока
    │   └── ros_entrypoint.sh   # Скрипт, выполняемый при открытии терминала контейнера
    └── ros2/
        └── Имя_пакета/
            └── Файлы_пакета
```

## Функциональный блоки

### body — управление платформой

Пакеты, относящиеся к управлению платформой и получению сенсорных данных. tracked_description, ros2_control и блоки сенсоров запускаются в виде одного сервиса одновременно.

- [bluespace_ai_xsens_ros_mti_driver](body/ros2/bluespace_ai_xsens_ros_mti_driver/README.md) — драйвер IMU XSENS MTi.
- [t21_lidar](body/ros2/t21_lidar/README.md) — драйверы Velodyne VLP-16.
- [t21_camera](body/ros2/t21_camera/README.md) — драйвер камеры.
- [ros2_control] — hardware-интерфейсы для управления платформой.
- [t21_teleop] — пакет телеуправления реальной платформой.
- [tracked_description](body/ros2/tracked_description/README.md) — описание робота (URDF/XACRO) и launch-файлы.
- [power_monitor](body/ros2/power_monitor/README.md) — мониторинг заряда аккумулятора. [Репозиторий автора](https://github.com/dakolzin/USR-DR134-GUI.git).

---

### SLAM — алгоритмы построения карт

- [GLIM](SLAM/GLIM/ros2/glim_ros2/README.md) — современный LiDAR-Inertial алгоритм.
- [t21_slam_toolbox](SLAM/t21_slam_toolbox/ros2/t21_slam_toolbox/README.md) — интеграция slam_toolbox.
- [LIO-SAM](SLAM/LIO_SAM/ros2/lio_sam/README.md) — LiDAR-Inertial Odometry SLAM.

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
| NVIDIA Container Toolkit | последняя | только при наличии GPU |
| make | любая | `sudo apt install make` |
| tmux | ≥ 3.2 | `sudo apt install tmux` |

### Установка Docker Desktop (Ubuntu)

```bash
<<<<<<< HEAD
t21_ws/src
/tracked_description
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
/LIO_SAM
├── CMakeLists.txt                # скрипт ament_cmake
├── package.xml                   # зависимости
├── launch/
│   ├── rviz.launch.py            # быстрый запуск визуализации
│   └── run.launch.py             # запуск программного модуля локализации
├── config/
|   ├── params.yaml               # список параметров
|   ├── robot.urdf.xacro          # urdf описание робота для запуска примера
│   └── rviz2.rviz                # конфигурационный RViz2

=======
# Add Docker's official GPG key:
sudo apt update 
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update

sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### Установка NVIDIA Container Toolkit (если используется GPU)

**Установка с apt: Ubuntu**

```bash
# Install the prerequisites 
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
   ca-certificates \
   curl \
   gnupg2
```
```bash
# Configure the production repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```
```bash
sudo apt-get update
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.19.0-1
sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
```
**Configuration**
```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### Настройка tmux (опционально)

Конфигурационный файл находится в [`data/tmux/tmux.conf`](data/tmux/tmux.conf).
Для использования скопируй его в стандартное место:
```bash
mkdir -p ~/.config/tmux
cp data/tmux/tmux.conf ~/.config/tmux/tmux.conf
```

Установи менеджер плагинов tpm и плагины:
```bash
# Клонировать tpm
git clone https://github.com/tmux-plugins/tpm ~/.config/tmux/plugins/tpm

# Установить плагины
~/.config/tmux/plugins/tpm/bin/install_plugins
```

Перезапусти tmux чтобы изменения вступили в силу:
```bash
tmux kill-server
tmux
>>>>>>> work
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

## Подготовка платформы
Перед запуском платформы необходимо обеспечить безопасность работ.
### Подключение электроники
Для запуска платформы необходимо убедиться, что аккумулятор установлен, все необходимые сенсоры подключены к USB-хабу и длина провода позволит безопасно управлять платформой. 
⚠️ **Не все USB-удлинители позволят одновременно передавать данные с роутера платформы, IMU, LiDAR'a и камеры.** Обычно страдает именно камера. Решение заключается в выводе дополнительного провода для изображения с Realsense.
Провода нужно закрепить, чтобы они не мешали движению платформы. 
Возможны перезапуски платформы из-за дребезга контактов кнопки.
### Гусеницы и флипперы
Тормоза на моторах нормально замкнуты и питаются от аккумулятора. ⚠️ ***В случае просадки напряжения на аккумуляторе ниже 18-17 В тормоза могут перестать разблокироваться, что приведёт к перегреву моторов и выходу их из строя.*** Поэтому необходимо отслеживать заряд через пакет power monitor.

Гусеницы на флипперах мешают движению платформы по асфальты: её начинает уводить в сторону. По этой причине они по умолчанию сняты.

При тестах системы управления флипперами платформу нужно расположить на подставке, которая обеспечит беспрепятственное движение флипперов по всей окружности. Это необходимо для безопасности управления платформой, т.к. флипперы могут начать самопроизвольно поворачивать по всей окружности из-за ошибки углового положения.

### Настройка сетевых профилей

Необходимо создать два сетевых профиля: для платформы и лазерного дальномера. Для этого необходимо добавить профиль и в настройках IPv4 Method выбрать Manual и указать следующие параметры:

| Модуль | Adress | Netmask |
|------------|----------|-----------------------|
| Платформа | ```192.168.3.12``` | ```255.255.255.255``` |
| LiDAR  | ```192.168.1.100``` | ```255.255.255.255``` |

Для проверки подключения к дальномеру можно открыть в браузере страницу настроек ```http://192.168.1.201/```.

### Управление

Управлять платформой необходимо с геймпада, плавно двигая стиками. При управлении через teleop_twist_keyboard или при резком надавливании на стик платформа будет передвигаться рывками, что может привести к выходу из строя моторов. Левый мотор склонен к перегреву, поэтому лучше не проводить длительные (>20-30 мин) заезды на высокой скорости и периодически давать мотору остывать.
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

# Остановить и удалить все контейнеры
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
<<<<<<< HEAD
ros2 launch tracked_description display.launch.py 
```
В RViz2 вы увидите:
=======
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
>>>>>>> work

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

<<<<<<< HEAD
Для запуска визуализации и системы управления выполняем в терминале:
```bash
ros2 launch tracked_description bringup_t21.launch.py
```
Для запуска симуляция необходимо дописать use_sim_time:=True
Для управления с джойстика запускаем в новом терминале:
```bash
ros2 launch t21_teleop joy_full_teleop.launch.py
```
=======
> Для сохранения сессии перед перезагрузкой используй `Ctrl+B`, затем `Ctrl+S`.
> После перезагрузки просто запусти `tmux` — сессия восстановится автоматически.

> В режиме copy-mode: `v` — начать выделение, `y` — скопировать в буфер обмена (wl-copy / xclip).
>>>>>>> work

> **При выделении мышью строк для копирования они сохраняются в буфер автоматически.**

<<<<<<< HEAD
Для работы с пакетом без робота, необходимо запустить эхо-сервер:
```bash
ros2 run tracked_description dummy
```
# [Модуль локализации lio_sam](#оглавление)
Для запуска модуля локализации необходимо после запуска системы управления выполнить в терминале:
```bash
ros2 launch lio_sam run.launch.py
```
=======
> [Подробности в видео](https://www.youtube.com/watch?v=GnP_SsMPNro&pp=ygUPYW1wZWVyc2FuZCB0bXV4)
---

## Известные проблемы

| Проблема | Статус |
|----------|--------|
| `velodyne_simulator` deb-пакет содержит ошибку | 🟡 Workaround: сборка из исходников |
>>>>>>> work
