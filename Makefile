USE_GPU := $(shell grep -E '^USE_GPU=' .env 2>/dev/null | cut -d= -f2 | tr -d '[:space:]')
USE_GPU ?= none

COMPOSE_BASE := docker compose -f docker-compose.yaml

ifeq ($(USE_GPU),nvidia)
  COMPOSE := $(COMPOSE_BASE) -f docker-compose.gpu.nvidia.yaml
else ifeq ($(USE_GPU),amd)
  COMPOSE := $(COMPOSE_BASE) -f docker-compose.gpu.amd.yaml
else
  COMPOSE := $(COMPOSE_BASE)
endif

.PHONY: slam navigation rosbag-replay gazebo-simulation develop \
        build-runtime build-simulation build-develop build-all down

slam:
	$(COMPOSE) up slam

navigation:
	$(COMPOSE) up navigation

rosbag-replay:
	$(COMPOSE) up rosbag-replay

gazebo-simulation:
	$(COMPOSE) up gazebo-simulation

develop:
	$(COMPOSE) up -d develop
	$(COMPOSE) exec develop bash

build-runtime:
	$(COMPOSE_BASE) build slam

build-simulation:
	$(COMPOSE_BASE) build gazebo-simulation

build-develop:
	$(COMPOSE_BASE) build develop

build-all: build-runtime build-simulation build-develop

down:
	$(COMPOSE) down
