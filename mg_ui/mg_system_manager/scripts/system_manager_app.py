#!/usr/bin/env python3
import asyncio
import json
import logging
import math
import os
import re
import subprocess
from pathlib import Path
from typing import Any

import docker
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s %(message)s",
)
logger = logging.getLogger(__name__)

DEFAULT_ALLOWED_ORIGINS = [
    "http://localhost:3000",
    "http://127.0.0.1:3000",
]


def _get_allowed_origins() -> list[str]:
    configured_origins = os.environ.get("SYSTEM_MANAGER_ALLOW_ORIGINS")
    if not configured_origins:
        return DEFAULT_ALLOWED_ORIGINS

    origins = [
        origin.strip()
        for origin in configured_origins.split(",")
        if origin.strip()
    ]
    if origins:
        return origins
    return DEFAULT_ALLOWED_ORIGINS


app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=_get_allowed_origins(),
    allow_methods=["*"],
    allow_headers=["*"],
)

COMPOSE_SERVICES: dict[str, str] = {
    "slam": "slam",
    "navigation": "navigation",
    "waypoint-editor": "waypoint-editor",
    "foxglove-bridge": "foxglove-bridge",
    "diagnostics": "diagnostics",
    "scenario-test": "scenario-test",
    "map-preview": "map-preview",
    "gazebo-simulation": "gazebo-simulation",
    "rviz2": "rviz2",
    "rviz2-navigation": "rviz2-navigation",
    "rviz2-slam": "rviz2-slam",
    "reoptimize-slam": "reoptimize-slam",
}


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

    def exec_in_container(
        self, service: str, cmd: list[str]
    ) -> tuple[bool, str]:
        container = self._get_container(service)
        if container is None:
            return False, f"container not found: {service}"
        try:
            result = container.exec_run(cmd)
            output = result.output.decode(errors="replace")
            ok = result.exit_code == 0
            if ok:
                logger.info("exec_in_container succeeded service=%s: %s",
                            service, output[:200])
            else:
                logger.error(
                    "exec_in_container failed service=%s exit_code=%d: %s",
                    service, result.exit_code, output[:200])
            return ok, output
        except Exception as e:
            logger.error(
                "exec_in_container exception service=%s: %s", service, e)
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

    def restart(self, service: str) -> tuple[bool, str]:
        logger.info("docker compose restart %s (cwd=%s)",
                    service, self._host_project_dir)
        env = os.environ.copy()
        env["HOME"] = self._host_home
        try:
            result = subprocess.run(
                ["docker", "compose", "restart", service],
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
                    "compose restart succeeded service=%s stdout=%s", service, stdout)
                return True, stdout
            logger.error("compose restart failed service=%s rc=%d stderr=%s",
                         service, result.returncode, stderr)
            return False, stderr
        except subprocess.TimeoutExpired:
            logger.error("compose restart timed out service=%s", service)
            return False, "command timed out"
        except Exception as e:
            logger.error(
                "compose restart exception service=%s: %s", service, e)
            return False, str(e)

    def start_rosbag(self, file: str, topics: list[str]) -> tuple[bool, str]:
        logger.info("start_rosbag file=%s topics=%s", file, topics)
        env = os.environ.copy()
        env["HOME"] = self._host_home
        env["ROSBAG_FILE"] = file
        env["ROSBAG_TOPICS"] = " ".join(topics) if topics else ""
        try:
            result = subprocess.run(
                ["docker", "compose", "up", "-d", "rosbag-replay"],
                capture_output=True,
                text=True,
                timeout=60,
                cwd=self._host_project_dir,
                env=env,
            )
            stdout = result.stdout.strip()
            stderr = result.stderr.strip()
            if result.returncode == 0:
                logger.info("rosbag start succeeded stdout=%s", stdout)
                return True, stdout
            logger.error("rosbag start failed rc=%d stderr=%s", result.returncode, stderr)
            return False, stderr
        except Exception as e:
            logger.error("rosbag start exception: %s", e)
            return False, str(e)

    def start_slam_gnss_2d_preview(self, slam_map_path: str) -> tuple[bool, str]:
        logger.info("start_slam_gnss_2d_preview slam_map_path=%s", slam_map_path)
        env = os.environ.copy()
        env["HOME"] = self._host_home
        env["SLAM_MAP_DIR"] = slam_map_path
        try:
            result = subprocess.run(
                ["docker", "compose", "up", "-d", "map-preview"],
                capture_output=True,
                text=True,
                timeout=60,
                cwd=self._host_project_dir,
                env=env,
            )
            stdout = result.stdout.strip()
            stderr = result.stderr.strip()
            if result.returncode == 0:
                logger.info("map-preview start succeeded stdout=%s", stdout)
                return True, stdout
            logger.error("map-preview start failed rc=%d stderr=%s", result.returncode, stderr)
            return False, stderr
        except Exception as e:
            logger.error("map-preview start exception: %s", e)
            return False, str(e)

    def start_reoptimize(self, input_dir: str, bag_path: str, save_dir: str) -> tuple[bool, str]:
        logger.info("start_reoptimize input_dir=%s bag_path=%s save_dir=%s", input_dir, bag_path, save_dir)
        env = os.environ.copy()
        env["HOME"] = self._host_home
        env["INPUT_DIR"] = input_dir
        env["BAG_PATH"] = bag_path or ""
        env["SAVE_DIR"] = save_dir or ""
        try:
            result = subprocess.run(
                ["docker", "compose", "up", "-d", "reoptimize-slam"],
                capture_output=True,
                text=True,
                timeout=60,
                cwd=self._host_project_dir,
                env=env,
            )
            stdout = result.stdout.strip()
            stderr = result.stderr.strip()
            if result.returncode == 0:
                logger.info("reoptimize-slam start succeeded stdout=%s", stdout)
                return True, stdout
            logger.error("reoptimize-slam start failed rc=%d stderr=%s", result.returncode, stderr)
            return False, stderr
        except Exception as e:
            logger.error("reoptimize-slam start exception: %s", e)
            return False, str(e)

    def _run_local_ros2_cmd(self, cmd: str) -> tuple[bool, str]:
        """system-managerコンテナ内でROS 2コマンドを実行する"""
        full_cmd = (
            "source /opt/ros/humble/setup.bash && "
            "source /root/ros2_ws/install/setup.bash && "
            f"{cmd}"
        )
        # 親プロセスの Python 環境変数をクリアして ROS 2 の Python 実行環境との衝突を防ぐ
        env = os.environ.copy()
        if "PYTHONPATH" in env:
            del env["PYTHONPATH"]
        if "PYTHONHOME" in env:
            del env["PYTHONHOME"]
            
        try:
            result = subprocess.run(
                ["bash", "-c", full_cmd],
                capture_output=True,
                text=True,
                timeout=30,
                env=env,
            )
            ok = result.returncode == 0
            output = result.stdout.strip() + "\n" + result.stderr.strip()
            return ok, output.strip()
        except Exception as e:
            return False, str(e)

    def save_common_map(self, map_dir: str, map_name: str) -> tuple[bool, str]:
        logger.info("save_common_map map_dir=%s map_name=%s", map_dir, map_name)
        
        pgm_path = Path(map_dir) / f"{map_name}.pgm"
        yaml_path = Path(map_dir) / f"{map_name}.yaml"
        if pgm_path.exists() or yaml_path.exists():
            return False, f"map already exists: {map_dir}/{map_name}"
            
        try:
            os.makedirs(map_dir, exist_ok=True)
        except Exception as e:
            return False, f"Failed to create directory: {e}"

        # map_saver_cliを使用して、サービスに依存せず /map トピックから直接地図画像を書き出す
        cmd = (
            f"ros2 run nav2_map_server map_saver_cli -t map "
            f"-f '{map_dir}/{map_name}' --occ 0.65 --free 0.15 --mode trinary"
        )
        return self._run_local_ros2_cmd(cmd)

    def save_slam_gnss_2d_map(self, slam_map_dir: str) -> tuple[bool, str]:
        logger.info("save_slam_gnss_2d_map dir=%s", slam_map_dir)
        
        try:
            os.makedirs(slam_map_dir, exist_ok=True)
        except Exception as e:
            return False, f"Failed to create directory: {e}"
        
        # 1. map_saver_cliを使用して、サービスに依存せず /map トピックから直接保存する（タイムアウト回避）
        cmd_map_save = (
            f"ros2 run nav2_map_server map_saver_cli -t map "
            f"-f '{slam_map_dir}/map' --occ 0.65 --free 0.15 --mode trinary"
        )
        ok, out = self._run_local_ros2_cmd(cmd_map_save)
        if not ok:
            return False, f"Failed to save map via map_saver_cli: {out}"
        
        # 2. SLAMパラメータに save_dir をセット
        for node in ["/slam_gnss_2d_node", "/slam_gnss_2d_offline_node", "/reoptimize_node"]:
            self._run_local_ros2_cmd(f"ros2 param set {node} save_dir '{slam_map_dir}'")
        
        # 3. SLAMノードの保存サービス呼び出し
        cmd_save_slam = "ros2 service call /slam_gnss_2d/save_slam_map std_srvs/srv/Trigger"
        ok, out = self._run_local_ros2_cmd(cmd_save_slam)
        if not ok:
            return False, f"Failed to call /slam_gnss_2d/save_slam_map: {out}"
        
        return True, "SLAM map saved successfully"

    def list_slam_gnss_2d_maps(self, base_dir: str) -> list[str]:
        logger.info("list_slam_gnss_2d_maps base_dir=%s", base_dir)
        path = Path(base_dir)
        if not path.is_dir():
            return []
        try:
            return [d.name for d in path.iterdir() if d.is_dir()]
        except Exception as e:
            logger.error("list_slam_gnss_2d_maps failed: %s", e)
            return []

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
        return self.exec_in_container(
            "gazebo-simulation", ["bash", "-lc", command]
        )


manager = DockerManager()

SETTINGS_DIR = Path(os.environ.get(
    "UI_DATA_DIR", "/root/ros2_data/mg_ui_local"))
SETTINGS_FILE = SETTINGS_DIR / "ui_settings.json"
SETTINGS_DIR.mkdir(parents=True, exist_ok=True)


def _result(ok: bool, msg: str) -> dict:
    level = logging.INFO if ok else logging.WARNING
    # エラー時は詳細トレースバックをコンソールに全文出力できるように制限を外す
    log_msg = msg if (not ok or not msg) else (msg if len(msg) <= 200 else msg[:200] + "...")
    logging.getLogger(__name__).log(
        level, "response success=%s message=%s", ok, log_msg)
    return {"success": ok, "message": msg}


def _make_start_handler(service: str):
    def handler():
        ok, msg = manager.start(service)
        return _result(ok, msg)
    handler.__name__ = f"start_{service.replace('-', '_')}"
    return handler


def _make_stop_handler(service: str):
    def handler():
        ok, msg = manager.stop(service)
        return _result(ok, msg)
    handler.__name__ = f"stop_{service.replace('-', '_')}"
    return handler


def _make_restart_handler(service: str):
    def handler():
        ok, msg = manager.restart(service)
        return _result(ok, msg)
    handler.__name__ = f"restart_{service.replace('-', '_')}"
    return handler


@app.get("/status")
def get_status():
    return manager.get_status()


@app.get("/settings")
def get_settings():
    if not SETTINGS_FILE.exists():
        return {}
    try:
        return json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
    except Exception as e:
        logger.error("get_settings failed: %s", e)
        return {}


@app.post("/settings")
async def post_settings(request: Request):
    try:
        body = await request.json()
        tmp = SETTINGS_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(body, ensure_ascii=False), encoding="utf-8")
        tmp.rename(SETTINGS_FILE)
        return {"success": True, "message": ""}
    except Exception as e:
        logger.error("post_settings failed: %s", e)
        return {"success": False, "message": str(e)}


for _endpoint, _service in COMPOSE_SERVICES.items():
    app.add_api_route(
        f"/{_endpoint}/start",
        _make_start_handler(_service),
        methods=["POST"],
    )
    app.add_api_route(
        f"/{_endpoint}/stop",
        _make_stop_handler(_service),
        methods=["POST"],
    )
    app.add_api_route(
        f"/{_endpoint}/restart",
        _make_restart_handler(_service),
        methods=["POST"],
    )


_MAP_PATH_RE = re.compile(r"^[a-zA-Z0-9/_\-\.]+$")
_MAP_NAME_RE = re.compile(r"^[a-zA-Z0-9_\-]+$")


class SaveCommonMapRequest(BaseModel):
    map_dir: str = "/root/ros2_data"
    map_name: str = "map"


@app.post("/map/common/save")
async def save_common_map(body: SaveCommonMapRequest):
    if not _MAP_PATH_RE.match(body.map_dir):
        return _result(False, "invalid map_dir")
    if not _MAP_NAME_RE.match(body.map_name):
        return _result(False, "invalid map_name")
    loop = asyncio.get_event_loop()
    ok, msg = await loop.run_in_executor(
        None, manager.save_common_map, body.map_dir, body.map_name
    )
    return _result(ok, msg)


class SaveSlamGnss2DMapRequest(BaseModel):
    output_dir: str = "/root/ros2_data/slam_maps"


@app.post("/slam_gnss_2d/map/save")
async def save_slam_gnss_2d_map(body: SaveSlamGnss2DMapRequest):
    if not _MAP_PATH_RE.match(body.output_dir):
        return _result(False, "invalid output_dir")
    
    import datetime
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    slam_map_dir = os.path.join(body.output_dir, timestamp)

    loop = asyncio.get_event_loop()
    ok, msg = await loop.run_in_executor(
        None, manager.save_slam_gnss_2d_map, slam_map_dir
    )
    return _result(ok, msg)


@app.get("/slam_gnss_2d/maps")
async def list_slam_gnss_2d_maps(base_dir: str = "/root/ros2_data/slam_maps"):
    if not _MAP_PATH_RE.match(base_dir):
        return {"success": False, "message": "invalid base_dir", "maps": []}
    loop = asyncio.get_event_loop()
    maps = await loop.run_in_executor(None, manager.list_slam_gnss_2d_maps, base_dir)
    return {"success": True, "maps": maps}


class SlamGnss2DPreviewStartRequest(BaseModel):
    slam_map_path: str


@app.post("/slam_gnss_2d/preview/start")
async def start_slam_gnss_2d_preview(body: SlamGnss2DPreviewStartRequest):
    if not _MAP_PATH_RE.match(body.slam_map_path):
        return _result(False, "invalid slam_map_path")
    loop = asyncio.get_event_loop()
    ok, msg = await loop.run_in_executor(
        None, manager.start_slam_gnss_2d_preview, body.slam_map_path
    )
    return _result(ok, msg)


@app.post("/slam_gnss_2d/preview/stop")
async def stop_slam_gnss_2d_preview():
    loop = asyncio.get_event_loop()
    ok, msg = await loop.run_in_executor(
        None, manager.stop, "map-preview"
    )
    return _result(ok, msg)


class SlamGnss2DReoptimizeStartRequest(BaseModel):
    input_dir: str
    bag_path: str = ""
    save_dir: str = ""


@app.post("/slam_gnss_2d/reoptimize/start")
async def start_slam_gnss_2d_reoptimize(body: SlamGnss2DReoptimizeStartRequest):
    if not _MAP_PATH_RE.match(body.input_dir):
        return _result(False, "invalid input_dir")
    if body.bag_path and not _ROSBAG_FILE_RE.match(body.bag_path):
        return _result(False, "invalid bag_path")
    if body.save_dir and not _MAP_PATH_RE.match(body.save_dir):
        return _result(False, "invalid save_dir")
    loop = asyncio.get_event_loop()
    ok, msg = await loop.run_in_executor(
        None, manager.start_reoptimize, body.input_dir, body.bag_path, body.save_dir
    )
    return _result(ok, msg)


@app.post("/slam_gnss_2d/reoptimize/stop")
async def stop_slam_gnss_2d_reoptimize():
    loop = asyncio.get_event_loop()
    ok, msg = await loop.run_in_executor(
        None, manager.stop, "reoptimize-slam"
    )
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


_ROSBAG_FILE_RE = re.compile(r"^[a-zA-Z0-9/_\-\.]+$")
_ROSBAG_TOPIC_RE = re.compile(r"^[a-zA-Z0-9/_\-\.]+$")


def _parse_env_file(path: Path) -> dict[str, str]:
    """シンプルな .env パーサー。${VAR} 形式の変数参照を展開する。"""
    env: dict[str, str] = {}
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            key = key.strip()
            value = value.strip()
            value = re.sub(
                r"\$\{([^}]+)\}",
                lambda m: env.get(m.group(1), os.environ.get(m.group(1), "")),
                value,
            )
            env[key] = value
    except Exception as e:
        logger.warning("parse_env_file failed path=%s: %s", path, e)
    return env


@app.get("/rosbag-replay/env")
def rosbag_replay_env():
    env_path = Path(manager._project_dir) / ".env"
    env = _parse_env_file(env_path)
    raw_topics = env.get("ROSBAG_TOPICS", "")
    topics = [t for t in raw_topics.split() if t]
    return {"file": env.get("ROSBAG_FILE", ""), "topics": topics}


class RosbagStartRequest(BaseModel):
    file: str
    topics: list[str] = []


@app.post("/rosbag-replay/start")
async def rosbag_replay_start(body: RosbagStartRequest):
    if not _ROSBAG_FILE_RE.match(body.file):
        return _result(False, "invalid file path")
    for topic in body.topics:
        if not _ROSBAG_TOPIC_RE.match(topic):
            return _result(False, f"invalid topic: {topic}")
    loop = asyncio.get_event_loop()
    ok, msg = await loop.run_in_executor(
        None, manager.start_rosbag, body.file, body.topics
    )
    return _result(ok, msg)


@app.post("/rosbag-replay/stop")
async def rosbag_replay_stop():
    loop = asyncio.get_event_loop()
    ok, msg = await loop.run_in_executor(
        None, manager.stop, "rosbag-replay"
    )
    return _result(ok, msg)


async def _stream_container_logs(
    queue: asyncio.Queue, service: str
) -> None:
    while True:
        container = manager._get_container(service)
        if container is None:
            await queue.put(
                {"service": service, "error": "container not found"}
            )
            await asyncio.sleep(5.0)
            continue

        container_ref = container.name or container.id
        if not container_ref:
            await queue.put(
                {"service": service, "error": "container reference missing"}
            )
            await asyncio.sleep(5.0)
            continue

        proc = await asyncio.create_subprocess_exec(
            "docker",
            "logs",
            "-f",
            "--since",
            "0s",
            container_ref,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )

        try:
            assert proc.stdout is not None
            async for line in proc.stdout:
                text = line.decode(errors="replace").rstrip("\n")
                if text:
                    await queue.put({"service": service, "line": text})
            await proc.wait()
            await asyncio.sleep(5.0)
        except asyncio.CancelledError:
            if proc.returncode is None:
                proc.terminate()
                await proc.wait()
            raise


def _filter_log_services(raw_services: Any) -> set[str]:
    if not isinstance(raw_services, list):
        return set()
    return {
        service
        for service in raw_services
        if isinstance(service, str) and service in COMPOSE_SERVICES
    }


@app.websocket("/logs/stream")
async def logs_stream(websocket: WebSocket):
    origin = websocket.headers.get("origin")
    if origin and origin not in _get_allowed_origins():
        await websocket.close(code=1008)
        return

    await websocket.accept()
    send_queue: asyncio.Queue[dict] = asyncio.Queue()
    tasks: dict[str, asyncio.Task[None]] = {}

    async def _sender() -> None:
        while True:
            msg = await send_queue.get()
            await websocket.send_json(msg)

    sender_task: asyncio.Task[None] = asyncio.create_task(_sender())

    def _bind_done_callback(service: str):
        def _on_done(task: asyncio.Task[None]) -> None:
            if tasks.get(service) is task:
                tasks.pop(service, None)

        return _on_done

    async def _cancel_removed(removed_services: set[str]) -> None:
        removed_tasks: list[asyncio.Task[None]] = []
        for service in removed_services:
            task = tasks.pop(service, None)
            if task is None:
                continue
            task.cancel()
            removed_tasks.append(task)
        if removed_tasks:
            await asyncio.gather(*removed_tasks, return_exceptions=True)

    try:
        while True:
            raw_message = await websocket.receive_text()
            try:
                payload = json.loads(raw_message)
            except json.JSONDecodeError:
                continue

            desired_services = _filter_log_services(payload.get("services"))
            current_services = set(tasks.keys())

            await _cancel_removed(current_services - desired_services)

            for service in desired_services - current_services:
                task = asyncio.create_task(
                    _stream_container_logs(send_queue, service))
                task.add_done_callback(_bind_done_callback(service))
                tasks[service] = task
    except WebSocketDisconnect:
        pass
    finally:
        remaining_tasks = list(tasks.values())
        for task in remaining_tasks:
            task.cancel()
        if remaining_tasks:
            await asyncio.gather(*remaining_tasks, return_exceptions=True)
        sender_task.cancel()
        await asyncio.gather(sender_task, return_exceptions=True)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8001)
