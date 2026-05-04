#!/usr/bin/env python3
import asyncio
import logging
import math
import os
import subprocess

import docker
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
)
logger = logging.getLogger(__name__)

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class DockerManager:
    def __init__(self) -> None:
        self._client = docker.from_env()
        self._project_dir = os.environ.get("PROJECT_DIR", "/app")
        self._host_project_dir = os.environ.get(
            "HOST_PROJECT_DIR", self._project_dir)
        self._host_home = os.environ.get(
            "HOST_HOME", os.environ.get("HOME", "/root"))
        self._simulation_world = os.environ.get(
            "SIMULATION_WORLD", "warehouse")
        self._simulation_robot_name = os.environ.get(
            "SIMULATION_ROBOT_NAME", "mg")
        logger.info(
            "DockerManager initialized: host_project_dir=%s host_home=%s simulation_world=%s robot=%s",
            self._host_project_dir,
            self._host_home,
            self._simulation_world,
            self._simulation_robot_name,
        )

    def _get_container(self, service: str):
        containers = self._client.containers.list(
            all=True,
            filters={"label": [f"com.docker.compose.service={service}"]},
        )
        container = containers[0] if containers else None
        if container is None:
            logger.warning("container not found for service=%s", service)
        else:
            logger.debug("found container service=%s id=%s status=%s",
                         service, container.short_id, container.status)
        return container

    def _compose_up(self, service: str) -> tuple[bool, str]:
        logger.info("docker compose up -d %s (cwd=%s)",
                    service, self._host_project_dir)
        env = os.environ.copy()
        env["HOME"] = self._host_home
        try:
            result = subprocess.run(
                ["docker", "compose", "up", "-d", service],
                capture_output=True,
                text=True,
                timeout=60,
                cwd=self._host_project_dir,
                env=env,
            )
            stdout = result.stdout.strip()
            stderr = result.stderr.strip()
            if result.returncode == 0:
                logger.info(
                    "compose up succeeded service=%s stdout=%s", service, stdout)
                return True, stdout
            logger.error("compose up failed service=%s rc=%d stderr=%s",
                         service, result.returncode, stderr)
            return False, stderr
        except subprocess.TimeoutExpired:
            logger.error("compose up timed out service=%s", service)
            return False, "command timed out"
        except Exception as e:
            logger.error("compose up exception service=%s: %s", service, e)
            return False, str(e)

    def get_status(self) -> dict[str, str]:
        containers = self._client.containers.list(
            all=True,
            filters={"label": ["com.docker.compose.service"]},
        )
        return {
            c.labels["com.docker.compose.service"]: c.status for c in containers
        }

    def start(self, service: str) -> tuple[bool, str]:
        logger.info("start service=%s", service)
        return self._compose_up(service)

    def stop(self, service: str) -> tuple[bool, str]:
        logger.info("stop service=%s", service)
        container = self._get_container(service)
        if container is None:
            return False, f"container not found: {service}"
        try:
            container.stop()
            logger.info("stopped service=%s", service)
            return True, ""
        except Exception as e:
            logger.error("stop failed service=%s: %s", service, e)
            return False, str(e)

    def save_map(self) -> tuple[bool, str]:
        logger.info("save_map")
        container = self._get_container("slam")
        if container is None:
            return False, "slam container not found"
        try:
            result = container.exec_run(
                [
                    "bash", "-c",
                    "source /opt/ros/humble/setup.bash && "
                    "source /root/ros2_ws/install/setup.bash && "
                    "ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "
                    '"{map_topic: map, map_url: /root/ros2_data/map, image_format: pgm, '
                    'map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"',
                ]
            )
            output = result.output.decode(errors="replace")
            ok = result.exit_code == 0
            if ok:
                logger.info("save_map succeeded: %s", output[:200])
            else:
                logger.error("save_map failed exit_code=%d: %s",
                             result.exit_code, output[:200])
            return ok, output
        except Exception as e:
            logger.error("save_map exception: %s", e)
            return False, str(e)

    def start_waypoint_editor(self) -> tuple[bool, str]:
        logger.info("start_waypoint_editor")
        container = self._get_container("develop")
        if container is None:
            return False, "develop container not found"
        try:
            container.exec_run(
                [
                    "bash", "-c",
                    "source /opt/ros/humble/setup.bash && "
                    "source /root/ros2_ws/install/setup.bash && "
                    "ros2 run mg_waypoint_navigation waypoint_editor_node.py",
                ],
                detach=True,
            )
            logger.info("start_waypoint_editor detached")
            return True, ""
        except Exception as e:
            logger.error("start_waypoint_editor exception: %s", e)
            return False, str(e)

    def reset_sim_robot_pose(
        self, x: float, y: float, z: float, yaw: float
    ) -> tuple[bool, str]:
        logger.info("reset_sim_robot_pose x=%s y=%s z=%s yaw=%s", x, y, z, yaw)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        request = (
            f'name: "{self._simulation_robot_name}" '
            f"position {{ x: {x} y: {y} z: {z} }} "
            f"orientation {{ x: 0.0 y: 0.0 z: {qz} w: {qw} }}"
        )
        command = (
            f"ign service "
            f"-s /world/{self._simulation_world}/set_pose "
            f"--reqtype ignition.msgs.Pose "
            f"--reptype ignition.msgs.Boolean "
            f"--timeout 5000 "
            f"--req '{request}'"
        )
        container = self._get_container("gazebo-simulation")
        if container is None:
            return False, "gazebo-simulation container not found"
        try:
            result = container.exec_run(["bash", "-lc", command])
            output = result.output.decode(errors="replace")
            ok = result.exit_code == 0
            if ok:
                logger.info("reset_sim_robot_pose succeeded: %s", output[:200])
            else:
                logger.error(
                    "reset_sim_robot_pose failed exit_code=%d: %s", result.exit_code, output[:200])
            return ok, output
        except Exception as e:
            logger.error("reset_sim_robot_pose exception: %s", e)
            return False, str(e)


manager = DockerManager()


def _result(ok: bool, msg: str) -> dict:
    level = logging.INFO if ok else logging.WARNING
    logging.getLogger(__name__).log(
        level, "response success=%s message=%s", ok, msg[:200] if msg else "")
    return {"success": ok, "message": msg}


@app.get("/status")
def get_status():
    return manager.get_status()


@app.post("/slam/start")
def start_slam():
    ok, msg = manager.start("slam")
    return _result(ok, msg)


@app.post("/slam/stop")
def stop_slam():
    ok, msg = manager.stop("slam")
    return _result(ok, msg)


@app.post("/navigation/start")
def start_navigation():
    ok, msg = manager.start("navigation")
    return _result(ok, msg)


@app.post("/navigation/stop")
def stop_navigation():
    ok, msg = manager.stop("navigation")
    return _result(ok, msg)


@app.post("/map/save")
async def save_map():
    loop = asyncio.get_event_loop()
    ok, msg = await loop.run_in_executor(None, manager.save_map)
    return _result(ok, msg)


@app.post("/waypoint-editor/start")
def start_waypoint_editor():
    ok, msg = manager.start_waypoint_editor()
    return _result(ok, msg)


@app.post("/scenario-test/start")
def start_scenario_test():
    ok, msg = manager.start("scenario-test")
    return _result(ok, msg)


@app.post("/scenario-test/stop")
def stop_scenario_test():
    ok, msg = manager.stop("scenario-test")
    return _result(ok, msg)


class ResetPoseRequest(BaseModel):
    x: float
    y: float
    z: float
    yaw: float


@app.post("/simulation/reset-pose")
async def reset_sim_robot_pose(body: ResetPoseRequest):
    loop = asyncio.get_event_loop()
    ok, msg = await loop.run_in_executor(
        None, manager.reset_sim_robot_pose, body.x, body.y, body.z, body.yaw
    )
    return _result(ok, msg)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8001)
