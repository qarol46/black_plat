# ============================================================================ #
# _____________________________ COMMON VARIABLES _____________________________ #
# ============================================================================ #

MKFILE_PATH   := $(abspath $(lastword $(MAKEFILE_LIST)))
ROOT_DIR      := $(dir $(MKFILE_PATH))

DISPLAY       ?= :0
ROS_DOMAIN_ID ?= 0

export DISPLAY ROS_DOMAIN_ID ROOT_DIR

# ============================================================================ #
# _________________________________ SERVICES _________________________________ #
# ============================================================================ #

# Отдельные сервисы
BODY_SERVICE          ?= body
TELEOP_SERVICE        ?= teleop
POWER_MONITOR_SERVICE ?= power_monitor
RVIZ_SERVICE          ?= rviz2
LIO_SAM_SERVICE       ?= lio-sam
LEGO_LOAM_SERVICE     ?= lego-loam
SLAM_TOOLBOX_SERVICE  ?= slam_toolbox
CARTOGRAPHER_SERVICE  ?= cartographer
NAV2_SERVICE          ?= nav2
SIM_SERVICE           ?= sim

# Группы сервисов
BODY_SERVICES := $(BODY_SERVICE) $(TELEOP_SERVICE) $(POWER_MONITOR_SERVICE)
SLAM_SERVICES := $(LIO_SAM_SERVICE) $(SLAM_TOOLBOX_SERVICE) $(CARTOGRAPHER_SERVICE)
NAV_SERVICES  := $(NAV2_SERVICE)
VIZ_SERVICES  := $(RVIZ_SERVICE)
SIM_SERVICES  := $(SIM_SERVICE)

# Полный стек
ALL_SERVICES  := $(BODY_SERVICES) $(SLAM_SERVICES) $(NAV_SERVICES) $(VIZ_SERVICES)

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

.PHONY: build build-body build-slam build-nav build-sim build-viz

build:
	@echo "==> Building: $(SERVICES)"
	@$(DC) build $(SERVICES)

build-body:
	@$(MAKE) build SERVICES="$(BODY_SERVICES)"

build-slam:
	@$(MAKE) build SERVICES="$(SLAM_SERVICES)"

build-nav:
	@$(MAKE) build SERVICES="$(NAV_SERVICES)"

build-sim:
	@$(MAKE) build SERVICES="$(SIM_SERVICES)"

build-viz:
	@$(MAKE) build SERVICES="$(VIZ_SERVICES)"

build-all:
	@$(MAKE) build SERVICES="$(ALL_SERVICES)"

# ============================================================================ #
# ______________________________ UP TARGETS __________________________________ #
# ============================================================================ #

.PHONY: up up-body up-slam up-nav up-sim up-viz up-robot up-full-sim

up: prepare-x11
	@echo "==> Starting: $(SERVICES)"
	@$(DC) up -d $(SERVICES)

up-body: prepare-x11
	@$(MAKE) up SERVICES="$(BODY_SERVICES)"

up-slam: prepare-x11
	@$(MAKE) up SERVICES="$(SLAM_SERVICES)"

up-lio-sam: prepare-x11
	@$(MAKE) up SERVICES="$(LIO_SAM_SERVICE)"

up-slam-toolbox: prepare-x11
	@$(MAKE) up SERVICES="$(SLAM_TOOLBOX_SERVICE)"

up-cartographer: prepare-x11
	@$(MAKE) up SERVICES="$(CARTOGRAPHER_SERVICE)"

up-nav: prepare-x11
	@$(MAKE) up SERVICES="$(NAV_SERVICES)"

up-sim: prepare-x11
	@$(MAKE) up SERVICES="$(SIM_SERVICES)"

up-viz: prepare-x11
	@$(MAKE) up SERVICES="$(VIZ_SERVICES)"

# Комбинированные
up-robot: prepare-x11
	@$(MAKE) up SERVICES="$(BODY_SERVICES) $(LIO_SAM_SERVICE) $(VIZ_SERVICES)"

up-full-sim: prepare-x11
	@$(MAKE) up SERVICES="$(SIM_SERVICES) $(VIZ_SERVICES)"

up-all: prepare-x11
	@$(MAKE) up SERVICES="$(ALL_SERVICES)"

# ============================================================================ #
# ______________________________ DOWN TARGETS ________________________________ #
# ============================================================================ #

.PHONY: down down-body down-slam down-nav down-sim down-viz down-all

down:
	@echo "==> Stopping: $(SERVICES)"
	@$(DC) stop $(SERVICES)

down-body:
	@$(MAKE) down SERVICES="$(BODY_SERVICES)"

down-slam:
	@$(MAKE) down SERVICES="$(SLAM_SERVICES)"

down-nav:
	@$(MAKE) down SERVICES="$(NAV_SERVICES)"

down-sim:
	@$(MAKE) down SERVICES="$(SIM_SERVICES)"

down-viz:
	@$(MAKE) down SERVICES="$(VIZ_SERVICES)"

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
#    SESSION   — имя tmux-сессии (по умолчанию "ros")
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

SESSION ?= ros
LAYOUT  ?= h

# Внутреннее имя tmux-layout
tmux_layout = $(if $(filter h,$(LAYOUT)),even-horizontal,$(if $(filter v,$(LAYOUT)),even-vertical,tiled))

.PHONY: monitor monitor-body monitor-slam monitor-nav monitor-robot monitor-kill

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
				"$(DC) logs -f --tail=200 $$service" Enter; \
			first=0; \
		else \
			tmux split-window -t $(SESSION):logs \
				"$(DC) logs -f --tail=200 $$service"; \
		fi; \
	done
	@# Выравниваем панели
	@tmux select-layout -t $(SESSION):logs $(call tmux_layout)
	@# Включаем синхронизацию прокрутки между панелями (опционально, закомментируй если не нужно)
	@# tmux setw -t $(SESSION):logs synchronize-panes on
	@# Добавляем вкладку для управления (shell)
	@tmux new-window -t $(SESSION) -n "shell"
	@tmux send-keys -t $(SESSION):shell "echo 'Shell ready. Сервисы: $(SERVICES)'" Enter
	@# Возвращаемся на вкладку с логами
	@tmux select-window -t $(SESSION):logs
	@echo "==> tmux сессия '$(SESSION)' запущена"
	@echo "     Переключение между вкладками: Ctrl+B затем 0/1"
	@echo "     Переключение между панелями:  Ctrl+B затем стрелки"
	@echo "     Закрыть сессию:               make monitor-kill SESSION=$(SESSION)"
	@tmux attach -t $(SESSION)

# Просмотр логов для конкретных групп
monitor-body:
	@$(MAKE) monitor SERVICES="$(BODY_SERVICES)" SESSION=body

monitor-slam:
	@$(MAKE) monitor SERVICES="$(SLAM_SERVICES)" SESSION=slam LAYOUT=v

monitor-nav:
	@$(MAKE) monitor SERVICES="$(NAV_SERVICES)" SESSION=nav

monitor-robot:
	@$(MAKE) monitor \
		SERVICES="$(BODY_SERVICE) $(LIO_SAM_SERVICE) $(NAV2_SERVICE) $(RVIZ_SERVICE)" \
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
	@tmux kill-session -t $(SESSION) 2>/dev/null && \
		echo "==> Сессия '$(SESSION)' закрыта" || \
		echo "Сессия '$(SESSION)' не найдена"

monitor-kill-all:
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
	@echo "║                        ROS2 MAKE TARGETS                         ║"
	@echo "╠══════════════════════════════════════════════════════════════════╣"
	@echo "║  BUILD                                                           ║"
	@echo "║    make build-body           — собрать body сервисы              ║"
	@echo "║    make build-slam           — собрать SLAM сервисы              ║"
	@echo "║    make build-nav            — собрать navigation                ║"
	@echo "║    make build-sim            — собрать симуляцию                 ║"
	@echo "║    make build SERVICES="s1 s2" — собрать конкретные сервисы      ║"
	@echo "╠══════════════════════════════════════════════════════════════════╣"
	@echo "║  UP / DOWN                                                       ║"
	@echo "║    make up-robot             — body + lio-sam + rviz             ║"
	@echo "║    make up-full-sim          — sim + rviz                        ║"
	@echo "║    make up-slam              — все SLAM сервисы                  ║"
	@echo "║    make up-lio-sam           — только lio-sam                    ║"
	@echo "║    make up SERVICES="s1 s2"   — запустить конкретные             ║"
	@echo "║    make down-all             — остановить все                    ║"
	@echo "╠══════════════════════════════════════════════════════════════════╣"
	@echo "║  МОНИТОРИНГ (tmux)                                               ║"
	@echo "║    make monitor              — логи всех сервисов                ║"
	@echo "║    make monitor-robot        — логи robot стека (tiled)          ║"
	@echo "║    make monitor-slam         — логи SLAM (вертикально)           ║"
	@echo "║    make monitor SERVICES="s1 s2" LAYOUT=h|v|tiled                ║"
	@echo "║    make run-and-monitor SERVICES="s1 s2" — up + monitor          ║"
	@echo "║    make monitor-kill         — закрыть сессию ros                ║"
	@echo "║    make monitor-kill SESSION=name — закрыть конкретную           ║"
	@echo "║    make monitor-ls           — список активных сессий            ║"
	@echo "╠══════════════════════════════════════════════════════════════════╣"
	@echo "║  ПРОЧЕЕ                                                          ║"
	@echo "║    make shell SERVICE=<name> — bash внутри контейнера            ║"
	@echo "║    make logs  SERVICE=<name> — логи одного сервиса               ║"
	@echo "║    make ps                   — статус контейнеров                ║"
	@echo "╠══════════════════════════════════════════════════════════════════╣"
	@echo "║  tmux: Ctrl+B → стрелки (панели), 0/1 (вкладки), d (detach)      ║"
	@echo "╚══════════════════════════════════════════════════════════════════╝"
	@echo ""