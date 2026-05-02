from __future__ import annotations

import math
import subprocess
from typing import Optional
import urllib.parse

from mg_scenario_test.adapters.base import SimulatorAdapter
from mg_scenario_test.scenario import ModelSpec, PoseSpec


class GazeboAdapter(SimulatorAdapter):
    """Gazebo Fortress 向けシミュレータアダプター。

    gz service CLI をサブプロセス経由で呼び出す。
    ワールドに UserCommands プラグインが有効になっている必要がある。
    """

    _FUEL_BASE_URL = "https://fuel.gazebosim.org/1.0"

    def __init__(self, world_name: str, robot_name: str, timeout_ms: int = 5000):
        self._world = world_name
        self._robot = robot_name
        self._timeout_ms = timeout_ms

    def set_robot_pose(self, name: str, pose: PoseSpec) -> bool:
        qz, qw = self._yaw_to_quat(pose.yaw)
        req = (
            f'name: "{name}" '
            f"position {{ x: {pose.x} y: {pose.y} z: {pose.z} }} "
            f"orientation {{ x: 0.0 y: 0.0 z: {qz} w: {qw} }}"
        )
        return self._call_service(
            f"/world/{self._world}/set_pose",
            "ignition.msgs.Pose",
            "ignition.msgs.Boolean",
            req,
        )

    def spawn_entity(self, name: str, model: ModelSpec, pose: PoseSpec) -> bool:
        sdf_fragment = self._build_sdf_fragment(name, model)
        if sdf_fragment is None:
            return False

        qz, qw = self._yaw_to_quat(pose.yaw)
        pose_str = (
            f"position {{ x: {pose.x} y: {pose.y} z: {pose.z} }} "
            f"orientation {{ x: 0.0 y: 0.0 z: {qz} w: {qw} }}"
        )

        if model.type == "fuel":
            req = (
                f'sdf_filename: "{sdf_fragment}" '
                f'name: "{name}" '
                f"allow_renaming: false "
                f"pose {{ {pose_str} }}"
            )
        else:
            escaped = sdf_fragment.replace('"', '\\"').replace("\n", " ")
            req = (
                f'sdf: "{escaped}" '
                f'name: "{name}" '
                f"allow_renaming: false "
                f"pose {{ {pose_str} }}"
            )

        return self._call_service(
            f"/world/{self._world}/create",
            "ignition.msgs.EntityFactory",
            "ignition.msgs.Boolean",
            req,
        )

    def despawn_entity(self, name: str) -> bool:
        req = f'name: "{name}" type: MODEL'
        return self._call_service(
            f"/world/{self._world}/remove",
            "ignition.msgs.Entity",
            "ignition.msgs.Boolean",
            req,
        )

    def _build_sdf_fragment(self, name: str, model: ModelSpec) -> Optional[str]:
        if model.type == "fuel":
            return self._fuel_url(model.uri)
        if model.type == "local":
            try:
                with open(model.path, "r") as f:
                    return f.read()
            except OSError as e:
                print(
                    f"[GazeboAdapter] Failed to read SDF file '{model.path}': {e}")
                return None
        if model.type == "primitive":
            return self._primitive_sdf(name, model)
        print(f"[GazeboAdapter] Unknown model type: {model.type}")
        return None

    def _fuel_url(self, uri: str) -> str:
        if uri.startswith("http"):
            return uri
        parts = uri.split("/models/", 1)
        if len(parts) == 2:
            owner = parts[0]
            model_name = urllib.parse.quote(parts[1])
            return f"{self._FUEL_BASE_URL}/{owner}/models/{model_name}"
        return uri

    @staticmethod
    def _primitive_sdf(name: str, model: ModelSpec) -> str:
        shape = model.shape
        size = model.size
        if shape == "box":
            sx = size.get("x", 1.0)
            sy = size.get("y", 1.0)
            sz = size.get("z", 1.0)
            geom = f"<box><size>{sx} {sy} {sz}</size></box>"
        elif shape == "cylinder":
            r = size.get("radius", 0.5)
            h = size.get("length", 1.0)
            geom = f"<cylinder><radius>{r}</radius><length>{h}</length></cylinder>"
        elif shape == "sphere":
            r = size.get("radius", 0.5)
            geom = f"<sphere><radius>{r}</radius></sphere>"
        else:
            geom = "<box><size>1 1 1</size></box>"

        return (
            f'<sdf version="1.6">'
            f"<model name=\"{name}\">"
            f"<static>true</static>"
            f"<link name=\"link\">"
            f"<collision name=\"collision\"><geometry>{geom}</geometry></collision>"
            f"<visual name=\"visual\"><geometry>{geom}</geometry></visual>"
            f"</link>"
            f"</model>"
            f"</sdf>"
        )

    def _call_service(
        self, service: str, req_type: str, rep_type: str, req: str
    ) -> bool:
        cmd = [
            "ign", "service",
            "-s", service,
            "--reqtype", req_type,
            "--reptype", rep_type,
            "--timeout", str(self._timeout_ms),
            "--req", req,
        ]
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self._timeout_ms / 1000.0 + 2.0,
            )
            if result.returncode != 0:
                print(
                    f"[GazeboAdapter] Service call failed: {service}\n"
                    f"stderr: {result.stderr.strip()}"
                )
                return False
            if result.stdout.strip() == "data: false":
                print(
                    f"[GazeboAdapter] Service returned data: false: {service}"
                )
                return False
            return True
        except subprocess.TimeoutExpired:
            print(f"[GazeboAdapter] Service call timed out: {service}")
            return False
        except FileNotFoundError:
            print(
                "[GazeboAdapter] 'ign' command not found. Is Gazebo Fortress installed?")
            return False

    @staticmethod
    def _yaw_to_quat(yaw: float):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw
