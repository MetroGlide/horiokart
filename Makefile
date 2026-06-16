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
        scenario-test scenario-test-full \
        slam-gnss-2d offline-slam-gnss-2d \
        shell shell-develop logs ps restart \
        build build-all build-no-cache \
        _collect-deps \
        rviz2 rviz2-slam rviz2-navigation down xhost config \
        test \
        diagnostics system-manager foxglove-bridge web-ui web-ui-dev tui ui-all ui-dev-all

# --- サービス起動 ---

slam:
	$(COMPOSE) up $(_up_flags) slam

navigation:
	$(COMPOSE) up $(_up_flags) navigation

rosbag-replay:
	$(COMPOSE) run --rm -it rosbag-replay

slam-gnss-2d:
	$(COMPOSE) up $(_up_flags) slam-gnss-2d

offline-slam-gnss-2d:
	$(COMPOSE) up $(_up_flags) offline-slam-gnss-2d

gazebo-simulation:
	$(COMPOSE) up $(_up_flags) gazebo-simulation

# make scenario-test
# make scenario-test SCENARIO=/app/mg_scenario_test/scenarios/example_waypoints_file.yaml
# make scenario-test SCENARIO=/app/mg_scenario_test/scenarios/example_inline_goals.yaml HEADLESS=false
scenario-test:
	$(if $(SCENARIO),SCENARIO_FILE=$(SCENARIO) )$(if $(HEADLESS),HEADLESS=$(HEADLESS) )$(COMPOSE) up $(_up_flags) scenario-test

# make scenario-test-full
# make scenario-test-full SCENARIO=/app/mg_scenario_test/scenarios/example_inline_goals.yaml HEADLESS=false
scenario-test-full:
	$(if $(SCENARIO),SCENARIO_FILE=$(SCENARIO) )$(if $(HEADLESS),HEADLESS=$(HEADLESS) )$(COMPOSE) up $(_up_flags) scenario-test-full

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
	$(COMPOSE_BASE) build slam navigation rosbag-replay gazebo-simulation develop rviz2-slam rviz2-navigation rviz2 web-ui

build-no-cache: _collect-deps
ifndef svc
	$(error svc is required. Usage: make build-no-cache svc=<service-name>)
endif
	$(COMPOSE_BASE) build --no-cache $(svc)

# --- 停止 ---

down:
	$(COMPOSE) down

# --- 再最適化 ---
# 実行例: make reoptimize INPUT_DIR=/app/maps/latest [OUTPUT_DIR=/app/maps/latest_opt] [CONFIG_FILE=/app/mg_slam/params/slam_gnss_2d.yaml] [BAG_PATH=/app/bags/my_bag]
reoptimize:
	$(COMPOSE) run --rm develop bash -c \
	  "source /opt/ros/humble/setup.bash && \
	   source /root/ros2_ws/install/setup.bash && \
	   export PYTHONPATH=/app/mg_slam/scripts:\$$PYTHONPATH && \
	   python3 /app/mg_slam/scripts/slam_gnss_2d/reoptimize_pose_graph.py \
	     --input_dir '$(INPUT_DIR)' \
	     $(if $(OUTPUT_DIR),--output_dir '$(OUTPUT_DIR)',) \
	     $(if $(CONFIG_FILE),--config_file '$(CONFIG_FILE)',) \
	     $(if $(BAG_PATH),--bag_path '$(BAG_PATH)',)"

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

foxglove-bridge:
	$(COMPOSE) up $(_up_flags) foxglove-bridge

diagnostics:
	$(COMPOSE) up $(_up_flags) diagnostics

system-manager:
	$(COMPOSE) up $(_up_flags) system-manager

web-ui:
	$(COMPOSE) up $(_up_flags) web-ui

ui-all:
	$(COMPOSE) up $(_up_flags) system-manager web-ui foxglove-bridge diagnostics

ui-dev-all:
	$(COMPOSE) up $(_up_flags) system-manager web-ui-dev foxglove-bridge diagnostics

web-ui-dev:
	$(COMPOSE) up web-ui-dev

tui:
	$(COMPOSE) run --rm -it develop bash -c \
	  "source /opt/ros/humble/setup.bash && \
	   source /root/ros2_ws/install/setup.bash && \
	   ros2 run mg_tui tui_node.py"

xhost:
	xhost +local:docker

config:
	$(COMPOSE) config

