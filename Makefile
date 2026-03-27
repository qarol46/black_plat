# ============================================================================ #
# _____________________________ COMMON VARIABLES _____________________________ #
# ============================================================================ #

MKFILE_PATH   := $(abspath $(lastword $(MAKEFILE_LIST)))
ROOT_DIR      := $(dir $(MKFILE_PATH))

DISPLAY       ?= :0
ROS_DOMAIN_ID ?= 0
LOG_MODE 	  ?= true
export DISPLAY ROS_DOMAIN_ID ROOT_DIR

# ============================================================================ #
# _________________________________ SERVICES _________________________________ #
# ============================================================================ #

# Отдельные сервисы
SIM_SERVICE           ?= sim
RVIZ_SERVICE          ?= rviz2
NAV2_SERVICE          ?= nav2
BODY_SERVICE          ?= body
TELEOP_SERVICE        ?= teleop
POWER_MONITOR_SERVICE ?= power_monitor
LIO_SAM_SERVICE       ?= lio-sam
LEGO_LOAM_SERVICE     ?= lego-loam
SLAM_TOOLBOX_SERVICE  ?= slam_toolbox
CARTOGRAPHER_SERVICE  ?= cartographer

# Группы сервисов
PLATFORM_SERVICES := $(BODY_SERVICE) $(TELEOP_SERVICE)
SLAM_SERVICE      := $(LIO_SAM_SERVICE)
SIM_SERVICES      := $(SIM_SERVICE)

# Полный стек (исправлено: было BODY_SERVICES — переменной не существовало)
ALL_SERVICES  := $(BODY_SERVICE) $(SLAM_SERVICE) $(RVIZ_SERVICE) $(POWER_MONITOR_SERVICE)

# Сервисы для запуска (переопределяется при вызове make)
SERVICES      ?= $(ALL_SERVICES)

# ============================================================================ #
# _______________________________ DOCKER COMPOSE _____________________________ #
# ============================================================================ #

DC := docker compose

# ============================================================================ #
# __________________________________ HELPERS _________________________________ #
# ============================================================================ #

.PHONY: prepare-x11
prepare-x11:
	@xhost +local: > /dev/null 2>&1 || true
	@xhost + > /dev/null 2>&1 || true

# ============================================================================ #
# ___________________________ BUILD TARGETS __________________________________ #
# ============================================================================ #

.PHONY: build build-platform build-slam build-nav2 build-sim build-rviz \
		build-body build-teleop build-power-monitor build-lio-sam build-lego-loam \
		build-slam-toolbox build-cartographer

build:
	@echo "==> Building: $(SERVICES)"
	@$(DC) build $(SERVICES)

build-platform:
	@$(MAKE) build SERVICES="$(PLATFORM_SERVICES)"

build-slam:
	@$(MAKE) build SERVICES="$(SLAM_SERVICE)"

build-nav2:
	@$(MAKE) build SERVICES="$(NAV2_SERVICE)"

build-sim:
	@$(MAKE) build SERVICES="$(SIM_SERVICES)"

build-rviz:
	@$(MAKE) build SERVICES="$(RVIZ_SERVICE)"

build-body:
	@$(MAKE) build SERVICES="$(BODY_SERVICE)"

build-power-monitor:
	@$(MAKE) build SERVICES="$(POWER_MONITOR_SERVICE)"

build-lio-sam:
	@$(MAKE) build SERVICES="$(LIO_SAM_SERVICE)"

build-lego-loam:
	@$(MAKE) build SERVICES="$(LEGO_LOAM_SERVICE)"

build-slam-toolbox:
	@$(MAKE) build SERVICES="$(SLAM_TOOLBOX_SERVICE)"

build-cartographer:
	@$(MAKE) build SERVICES="$(CARTOGRAPHER_SERVICE)"

build-all:
	@$(MAKE) build SERVICES="$(ALL_SERVICES)"

# ============================================================================ #
# ______________________________ UP TARGETS __________________________________ #
# ============================================================================ #

.PHONY: up up-platform up-slam up-nav2 up-sim up-rviz \
		up-body up-teleop up-power-monitor up-lio-sam up-lego-loam \
		up-slam-toolbox up-cartographer

up: prepare-x11
	@echo "==> Starting: $(SERVICES)"
	ifeq ($(LOG_MODE),true)
		@$(DC) up $(SERVICES)
	else
		@$(DC) up -d $(SERVICES)
	endif
				
up-platform: prepare-x11
	@$(MAKE) up SERVICES="$(PLATFORM_SERVICES)"

up-slam: prepare-x11
	@$(MAKE) up SERVICES="$(SLAM_SERVICE)"

up-nav2: prepare-x11
	@$(MAKE) up SERVICES="$(NAV2_SERVICE)"

up-sim: prepare-x11
	@$(MAKE) up SERVICES="$(SIM_SERVICES)"

up-rviz: prepare-x11
	@$(MAKE) up SERVICES="$(RVIZ_SERVICE)"

up-body: prepare-x11
	@$(MAKE) up SERVICES="$(BODY_SERVICE)"

up-teleop: prepare-x11
	@$(MAKE) up SERVICES="$(TELEOP_SERVICE)"

up-power-monitor: prepare-x11
	@$(MAKE) up SERVICES="$(POWER_MONITOR_SERVICE)"

up-lio-sam: prepare-x11
	@$(MAKE) up SERVICES="$(LIO_SAM_SERVICE)"

up-lego-loam: prepare-x11
	@$(MAKE) up SERVICES="$(LEGO_LOAM_SERVICE)"

up-slam-toolbox: prepare-x11
	@$(MAKE) up SERVICES="$(SLAM_TOOLBOX_SERVICE)"

up-cartographer: prepare-x11
	@$(MAKE) up SERVICES="$(CARTOGRAPHER_SERVICE)"

up-all: prepare-x11
	@$(MAKE) up SERVICES="$(ALL_SERVICES)"

# ============================================================================ #
# ______________________________ DOWN TARGETS ________________________________ #
# ============================================================================ #

.PHONY: down down-platform down-slam down-nav2 down-sim down-rviz \
		down-body down-teleop down-power-monitor down-lio-sam down-lego-loam \
		down-slam-toolbox down-cartographer
down:
	@echo "==> Stopping: $(SERVICES)"
	@$(DC) stop $(SERVICES)

down-platform:
	@$(MAKE) down SERVICES="$(PLATFORM_SERVICES)"

down-slam:
	@$(MAKE) down SERVICES="$(SLAM_SERVICE)"

down-nav2:
	@$(MAKE) down SERVICES="$(NAV2_SERVICE)"

down-sim:
	@$(MAKE) down SERVICES="$(SIM_SERVICES)"

down-rviz:
	@$(MAKE) down SERVICES="$(RVIZ_SERVICE)"

down-body:
	@$(MAKE) down SERVICES="$(BODY_SERVICE)"

down-teleop:
	@$(MAKE) down SERVICES="$(TELEOP_SERVICE)"

down-power-monitor:
	@$(MAKE) down SERVICES="$(POWER_MONITOR_SERVICE)"

down-lio-sam:
	@$(MAKE) down SERVICES="$(LIO_SAM_SERVICE)"

down-lego-loam:
	@$(MAKE) down SERVICES="$(LEGO_LOAM_SERVICE)"

down-slam-toolbox:
	@$(MAKE) down SERVICES="$(SLAM_TOOLBOX_SERVICE)"

down-cartographer:
	@$(MAKE) down SERVICES="$(CARTOGRAPHER_SERVICE)"

down-all:
	@$(DC) down

# ============================================================================ #
# __________________________________ SHELL __________________________________ #
# ============================================================================ #

.PHONY: shell logs ps

shell:
	@if [ -z "$(SERVICE)" ]; then \
		echo "Usage: make shell SERVICE=<name>"; exit 1; \
	fi
	@$(DC) exec $(SERVICE) bash

logs:
	@if [ -z "$(SERVICE)" ]; then \
		echo "Usage: make logs SERVICE=<name>"; exit 1; \
	fi
	@$(DC) logs -f $(SERVICE)

ps:
	@$(DC) ps

# ============================================================================ #
# __________________________________ TMUX ____________________________________ #
# ============================================================================ #
#
#  Переменные:
#    SERVICES  — список сервисов через пробел (по умолчанию ALL_SERVICES)
#    SESSION   — имя tmux-сессии (по умолчанию "black_plat")
#    LAYOUT    — расположение панелей: h (horizontal) | v (vertical) | tiled
#                  h (even-horizontal) — панели в ряд слева направо
#                  v (even-vertical)   — панели столбцом сверху вниз
#                  tiled               — сетка, максимально квадратная
#
#  Примеры:
#    make monitor                                          — все сервисы, горизонтально
#    make monitor SERVICES="lio-sam nav2" LAYOUT=v        — два сервиса, вертикально
#    make monitor SERVICES="body lio-sam nav2" LAYOUT=tiled SESSION=debug
# ============================================================================ #

SESSION ?= black_plat
LAYOUT  ?= h

# Внутреннее имя tmux-layout
tmux_layout = $(if $(filter h,$(LAYOUT)),even-horizontal,$(if $(filter v,$(LAYOUT)),even-vertical,tiled))

.PHONY: monitor monitor-platform monitor-slam monitor-nav2 monitor-robot monitor-sim \
		monitor-down monitor-kill monitor-kill-all monitor-ls monitor-attach

# Основная цель — запускает tmux с панелями для каждого сервиса из SERVICES
monitor:
	@if ! command -v tmux > /dev/null 2>&1; then \
		echo "tmux не установлен. Установи: sudo apt install tmux"; exit 1; \
	fi
	@# Убиваем старую сессию с тем же именем если есть
	@tmux kill-session -t $(SESSION) 2>/dev/null || true
	@# Создаём новую сессию с первым сервисом
	@first=1; \
	for service in $(SERVICES); do \
		if [ $$first -eq 1 ]; then \
			tmux new-session -d -s $(SESSION) -n "logs" \
				-x "$(shell tput cols 2>/dev/null || echo 220)" \
				-y "$(shell tput lines 2>/dev/null || echo 50)"; \
			tmux send-keys -t $(SESSION):logs \
				"$(DC) up $$service" Enter; \
			first=0; \
		else \
			tmux split-window -t $(SESSION):logs \
				"$(DC) up $$service"; \
		fi; \
	done
	@# Выравниваем панели
	@tmux select-layout -t $(SESSION):logs $(call tmux_layout)
	@# Добавляем вкладку для управления (shell)
	@tmux new-window -t $(SESSION) -n "shell"
	@tmux send-keys -t $(SESSION):shell "echo 'Shell ready. Сервисы: $(SERVICES)'" Enter
	@# Возвращаемся на вкладку с логами
	@tmux select-window -t $(SESSION):logs
	@echo "==> tmux сессия '$(SESSION)' запущена"
	@echo "     Переключение между вкладками: Alt+1 / Alt+2"
	@echo "     Переключение между панелями:  Alt+стрелки"
	@echo "     Detach:                       Alt+d"
	@echo "     Закрыть сессию:               make monitor-kill SESSION=$(SESSION)"
	@tmux attach -t $(SESSION)

monitor-attach:
	@tmux attach -t $(SESSION)

# Просмотр логов для конкретных групп
monitor-platform:
	@$(MAKE) monitor SERVICES="$(PLATFORM_SERVICES)" SESSION=platform LAYOUT=v

monitor-slam:
	@$(MAKE) monitor SERVICES="$(SLAM_SERVICE)" SESSION=slam LAYOUT=v

monitor-nav2:
	@$(MAKE) monitor SERVICES="$(NAV2_SERVICE)" SESSION=nav LAYOUT=v

monitor-robot:
	@$(MAKE) monitor \
		SERVICES="$(BODY_SERVICE) $(SLAM_SERVICE) $(NAV2_SERVICE) $(RVIZ_SERVICE) $(POWER_MONITOR_SERVICE)"  \
		SESSION=robot \
		LAYOUT=tiled

monitor-sim:
	@$(MAKE) monitor SERVICES="$(SIM_SERVICES)" SESSION=sim

# Запустить сервисы И сразу открыть мониторинг
.PHONY: run-and-monitor

run-and-monitor: prepare-x11
	@echo "==> Запускаем сервисы: $(SERVICES)"
	@$(DC) up -d $(SERVICES)
	@sleep 1
	@$(MAKE) monitor SERVICES="$(SERVICES)" SESSION="$(SESSION)" LAYOUT="$(LAYOUT)"

# Закрыть tmux сессию
monitor-kill:
	@$(MAKE) down-all
	@tmux kill-session -t $(SESSION) 2>/dev/null && \
		echo "==> Сессия '$(SESSION)' закрыта" || \
		echo "Сессия '$(SESSION)' не найдена"

monitor-kill-all:
	@$(MAKE) down-all
	@tmux kill-server 2>/dev/null && \
		echo "==> Все tmux сессии закрыты" || \
		echo "Нет активных tmux сессий"

# Список активных сессий
monitor-ls:
	@tmux ls 2>/dev/null || echo "Нет активных tmux сессий"

# ============================================================================ #
# __________________________________ HELP ____________________________________ #
# ============================================================================ #

.PHONY: help
help:
	@echo ""
	@echo "╔══════════════════════════════════════════════════════════════════╗"
	@echo "║                        ROS2 MAKE TARGETS                        ║"
	@echo "╠══════════════════════════════════════════════════════════════════╣"
	@echo "║  BUILD                                                           ║"
	@echo "║    make build-platform       — собрать platform сервисы         ║"
	@echo "║    make build-slam           — собрать SLAM сервисы             ║"
	@echo "║    make build-nav2           — собрать navigation               ║"
	@echo "║    make build-sim            — собрать симуляцию                ║"
	@echo "║    make build-all            — собрать все образы               ║"
	@echo "║    make build SERVICES='s1 s2' — собрать конкретные сервисы     ║"
	@echo "╠══════════════════════════════════════════════════════════════════╣"
	@echo "║  UP / DOWN                                                       ║"
	@echo "║    make up-all               — запустить весь стек (detached)   ║"
	@echo "║    make up-platform          — body + teleop                    ║"
	@echo "║    make up-slam              — SLAM сервис (lio-sam)            ║"
	@echo "║    make up-lio-sam           — только lio-sam                   ║"
	@echo "║    make up-sim               — симуляция                        ║"
	@echo "║    make up SERVICES='s1 s2'  — запустить конкретные             ║"
	@echo "║    make down-all             — остановить все                   ║"
	@echo "╠══════════════════════════════════════════════════════════════════╣"
	@echo "║  МОНИТОРИНГ (tmux)                                               ║"
	@echo "║    make monitor              — логи ALL_SERVICES в панелях      ║"
	@echo "║    make monitor-robot        — body+slam+nav2+rviz (tiled)      ║"
	@echo "║    make monitor-platform     — body+teleop (вертикально)        ║"
	@echo "║    make monitor-slam         — SLAM логи (вертикально)          ║"
	@echo "║    make monitor SERVICES='s1 s2' LAYOUT=h|v|tiled               ║"
	@echo "║    make run-and-monitor SERVICES='s1 s2' — up + monitor         ║"
	@echo "║    make monitor-kill         — остановить сервисы + сессию      ║"
	@echo "║    make monitor-kill SESSION=name — конкретная сессия            ║"
	@echo "║    make monitor-kill-all     — остановить все сессии tmux       ║"
	@echo "║    make monitor-ls           — список активных сессий           ║"
	@echo "╠══════════════════════════════════════════════════════════════════╣"
	@echo "║  ПРОЧЕЕ                                                          ║"
	@echo "║    make shell SERVICE=<n>    — bash внутри контейнера           ║"
	@echo "║    make logs  SERVICE=<n>    — follow логи одного сервиса       ║"
	@echo "║    make ps                   — статус контейнеров               ║"
	@echo "╠══════════════════════════════════════════════════════════════════╣"
	@echo "║  tmux (кастомный конфиг):                                        ║"
	@echo "║    Alt+стрелки — переключение панелей                           ║"
	@echo "║    Alt+1..9    — переключение окон                              ║"
	@echo "║    Alt+h/v     — разделить панель горизонтально/вертикально     ║"
	@echo "║    Alt+Enter   — новое окно                                     ║"
	@echo "║    Alt+d       — detach от сессии                               ║"
	@echo "║    Shift+drag  — выделить текст мышью в буфер терминала         ║"
	@echo "╚══════════════════════════════════════════════════════════════════╝"
	@echo ""