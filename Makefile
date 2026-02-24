# ============================================================================ #
#                             COMMON VARIABLES                                 #
# ============================================================================ #

MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(dir $(MKFILE_PATH))
ROOT_DIR := $(MKFILE_DIR)

# Параметры окружения (можно переопределить при вызове make)
DISPLAY ?= :0
ROS_DOMAIN_ID ?= 19
RMW_IMPLEMENTATION ?= rmw_cyclonedds_cpp
CYCLONEDDS_URI ?= /dds/cyclonedds.xml

# Выбор компонентов для запуска (через пробел: body slam viz ...)
COMPONENTS ?= body

# Выбор конкретного алгоритма SLAM (lio-sam или lego-loam)
SLAM_SERVICE ?= lio-sam

# Дополнительные файлы docker-compose (можно указать при вызове)
EXTRA_COMPOSE_FILES ?=

# ============================================================================ #
#                     DETECT AVAILABLE COMPOSE FILES                           #
# ============================================================================ #

BODY_COMPOSE := $(wildcard $(ROOT_DIR)/body/docker-compose.yaml)
SLAM_COMPOSE := $(wildcard $(ROOT_DIR)/SLAM/docker-compose.yaml)
NAVIGATION_COMPOSE := $(wildcard $(ROOT_DIR)/navigation/docker-compose.yaml)
SIM_COMPOSE := $(wildcard $(ROOT_DIR)/simulation/docker-compose.yaml)
VIZ_COMPOSE   := $(wildcard $(ROOT_DIR)/visualization/docker-compose.yaml)

# Базовый compose файл (корневой)
COMPOSE_BASE := -f $(ROOT_DIR)/docker-compose.yaml
COMPOSE_FILES := $(COMPOSE_BASE)

# Добавляем файлы для выбранных компонентов
ifneq ($(filter body,$(COMPONENTS)),)
    ifneq ($(BODY_COMPOSE),)
        COMPOSE_FILES += -f $(BODY_COMPOSE)
    endif
endif
ifneq ($(filter slam,$(COMPONENTS)),)
    ifneq ($(SLAM_COMPOSE),)
        COMPOSE_FILES += -f $(SLAM_COMPOSE)
    endif
endif
ifneq ($(filter navigation,$(COMPONENTS)),)
    ifneq ($(NAVIGATION_COMPOSE),)
        COMPOSE_FILES += -f $(NAVIGATION_COMPOSE)
    endif
endif
ifneq ($(filter simulation,$(COMPONENTS)),)
    ifneq ($(SIM_COMPOSE),)
        COMPOSE_FILES += -f $(SIM_COMPOSE)
    endif
endif
ifneq ($(filter viz,$(COMPONENTS)),)
    ifneq ($(VIZ_COMPOSE),)
        COMPOSE_FILES += -f $(VIZ_COMPOSE)
    endif
endif

# ============================================================================ #
#                            DOCKER COMPOSE COMMANDS                           #
# ============================================================================ #

DC := docker compose $(COMPOSE_FILES)
DC_BUILD := $(DC) build
DC_UP := $(DC) up -d
DC_DOWN := $(DC) down
DC_LOGS := $(DC) logs -f
DC_PS := $(DC) ps
DC_EXEC := $(DC) exec

# Экспортируем переменные окружения для docker compose
export DISPLAY ROS_DOMAIN_ID ROOT_DIR #RMW_IMPLEMENTATION CYCLONEDDS_URI

# ============================================================================ #
#                              HELPER TARGETS                                  #
# ============================================================================ #

.PHONY: prepare-x11
prepare-x11:
	@echo "Preparing X11 for visualization..."
	@xhost +local: > /dev/null 2>&1 || true
	@xhost + > /dev/null 2>&1 || true
	@export RCUTILS_COLORIZED_OUTPUT=1

# ============================================================================ #
#                           GENERAL TARGETS                                    #
# ============================================================================ #

.PHONY: build up down logs ps shell

build: prepare-x11
	@echo "Building selected components: $(COMPONENTS)"
	@$(DC_BUILD)

up: prepare-x11
	@echo "Starting selected components: $(COMPONENTS)"
	@$(DC_UP)

down:
	@echo "Stopping selected components: $(COMPONENTS)"
	@$(DC_DOWN)

logs:
	@$(DC_LOGS)

ps:
	@$(DC_PS)

shell:
	@if [ -z "$(SERVICE)" ]; then \
		echo "Please specify SERVICE=..."; \
		exit 1; \
	fi
	@$(DC_EXEC) $(SERVICE) bash

# ============================================================================ #
#                         COMPONENT-SPECIFIC TARGETS                           #
# ============================================================================ #

# Body
.PHONY: build-body up-body down-body run-body
build-body: COMPONENTS = body
build-body: build
up-body: COMPONENTS = body
up-body: prepare-x11
	@$(DC_UP) body
down-body: COMPONENTS = body
down-body:
	@$(DC_DOWN) body

# SLAM
.PHONY: build-slam up-slam down-slam
build-slam: COMPONENTS = slam
build-slam: build
up-slam: COMPONENTS = slam
up-slam: prepare-x11
	@$(DC_UP) $(SLAM_SERVICE)
down-slam: COMPONENTS = slam
down-slam:
	@$(DC_DOWN) $(SLAM_SERVICE)

# Navigation
.PHONY: build-navigation up-navigation down-navigation
build-navigation: COMPONENTS = navigation
build-navigation: build
up-navigation: COMPONENTS = navigation
up-navigation: prepare-x11
	@$(DC_UP)
down-navigation: COMPONENTS = navigation
down-navigation:
	@$(DC_DOWN)

# Stuff
.PHONY: build-stuff up-stuff down-stuff
build-stuff: COMPONENTS = stuff
build-stuff: build
up-stuff: COMPONENTS = stuff
up-stuff: prepare-x11
	@$(DC_UP)
down-stuff: COMPONENTS = stuff
down-stuff:
	@$(DC_DOWN)

# Visualization (новый компонент)
.PHONY: build-viz up-viz down-viz
build-viz: COMPONENTS = viz
build-viz: build
up-viz: COMPONENTS = viz
up-viz: prepare-x11
	@$(DC_UP)
down-viz: COMPONENTS = viz
down-viz:
	@$(DC_DOWN)

# ============================================================================ #
#                          VISUALIZATION TARGETS                               #
# ============================================================================ #

# Удобная цель для запуска rviz2 (использует компонент viz, если он определён)
.PHONY: up-rviz
up-rviz:
	@if [ -n "$(VIZ_COMPOSE)" ]; then \
		$(MAKE) up-viz; \
	else \
		echo "vizualization/docker-compose.yaml not found. Falling back to manual container start."; \
		$(MAKE) prepare-x11; \
		docker run -it --rm \
			--network host \
			-e DISPLAY=$(DISPLAY) \
			-e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) \
			-v /tmp/.X11-unix:/tmp/.X11-unix \
			osrf/ros:jazzy-desktop \
			rviz2; \
	fi

# (Опционально) Цель для Foxglove, если он есть в vizualization
.PHONY: up-foxglove
up-foxglove:
	@if [ -n "$(VIZ_COMPOSE)" ]; then \
		cd $(ROOT_DIR)/vizualization && docker compose up -d foxglove; \
	else \
		echo "Foxglove not configured. Please provide vizualization/docker-compose.yaml"; \
	fi

# ============================================================================ #
#                      BACKWARD COMPATIBILITY TARGETS                         #
# ============================================================================ #

.PHONY: run-body run-brain run-camera run-test run-rviz run

run-body: up-body
run-brain:
	@echo "Brain services are not yet modularized. Use COMPONENTS='body slam' etc."
run-camera:
	@echo "Camera services are part of 'body'. Use up-body."
run-test:
	@echo "Test service is not defined. Use up-slam with SLAM_SERVICE or similar."
run-rviz: up-rviz
run: COMPONENTS = body slam viz   # по умолчанию запускать body, slam и viz
run: up