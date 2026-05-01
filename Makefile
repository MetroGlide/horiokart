USE_GPU := $(shell grep -E '^USE_GPU=' .env 2>/dev/null | cut -d= -f2 | tr -d '[:space:]')
USE_GPU ?= none

COMPOSE_BASE := docker compose -f compose.yaml

ifeq ($(USE_GPU),nvidia)
  COMPOSE := $(COMPOSE_BASE) -f compose.gpu.nvidia.yaml
else ifeq ($(USE_GPU),amd)
  COMPOSE := $(COMPOSE_BASE) -f compose.gpu.amd.yaml
else
  COMPOSE := $(COMPOSE_BASE)
endif

# make <service> DETACH=1 でバックグラウンド起動
_up_flags = $(if $(DETACH),-d,)

.PHONY: slam navigation rosbag-replay gazebo-simulation develop \
        scenario-test \
        shell shell-develop logs ps restart \
        build build-all build-no-cache \
        _collect-deps \
        rviz2 rviz2-slam rviz2-navigation down xhost config \
        test

# --- サービス起動 ---

slam:
	$(COMPOSE) up $(_up_flags) slam

navigation:
	$(COMPOSE) up $(_up_flags) navigation

rosbag-replay:
	$(COMPOSE) up $(_up_flags) rosbag-replay

gazebo-simulation:
	$(COMPOSE) up $(_up_flags) gazebo-simulation

# make scenario-test
# make scenario-test SCENARIO=/app/mg_scenario_test/scenarios/example_waypoints_file.yaml
# make scenario-test SCENARIO=/app/mg_scenario_test/scenarios/example_inline_goals.yaml HEADLESS=false
scenario-test:
	$(if $(SCENARIO),SCENARIO_FILE=$(SCENARIO) )$(if $(HEADLESS),HEADLESS=$(HEADLESS) )$(COMPOSE) up $(_up_flags) scenario-test

develop:
	$(COMPOSE) up -d develop
	$(if $(ATTACH),$(COMPOSE) exec develop bash,)

# --- シェルアクセス ---

shell:
ifndef svc
	$(error svc is required. Usage: make shell svc=<service-name>)
endif
	$(COMPOSE) exec $(svc) bash

shell-develop:
	$(COMPOSE) up -d develop
	$(COMPOSE) exec develop bash

# --- ログ ---

logs:
ifndef svc
	$(error svc is required. Usage: make logs svc=<service-name>)
endif
	$(COMPOSE) logs -f $(svc)

# --- コンテナ状態 ---

ps:
	$(COMPOSE) ps

restart:
ifndef svc
	$(error svc is required. Usage: make restart svc=<service-name>)
endif
	$(COMPOSE) restart $(svc)

# --- ビルド ---

_collect-deps:
	bash docker/collect_deps.sh

build: _collect-deps
ifndef svc
	$(error svc is required. Usage: make build svc=<service-name>)
endif
	$(COMPOSE_BASE) build $(svc)

build-all: _collect-deps
	$(COMPOSE_BASE) build slam navigation rosbag-replay gazebo-simulation develop rviz2-slam rviz2-navigation rviz2

build-no-cache: _collect-deps
ifndef svc
	$(error svc is required. Usage: make build-no-cache svc=<service-name>)
endif
	$(COMPOSE_BASE) build --no-cache $(svc)

# --- 停止 ---

down:
	$(COMPOSE) down

# --- テスト ---
# 全テスト: make test
# 特定パッケージ: make test pkg=mg_waypoint_navigation
test:
	$(COMPOSE) run --rm --no-deps develop bash -c \
	  "cd /app && \
	   PYTHONPATH=\$$(find /app -maxdepth 1 -mindepth 1 -type d | tr '\n' ':') \
	   python3 -m pytest $(if $(pkg),$(pkg)/test/,) -v"

# --- ユーティリティ ---

rviz2:
	$(COMPOSE) up $(_up_flags) rviz2

rviz2-slam:
	$(COMPOSE) up $(_up_flags) rviz2-slam

rviz2-navigation:
	$(COMPOSE) up $(_up_flags) rviz2-navigation

xhost:
	xhost +local:docker

config:
	$(COMPOSE) config

