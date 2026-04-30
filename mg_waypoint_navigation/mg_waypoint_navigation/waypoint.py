from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import yaml
from geometry_msgs.msg import PoseStamped, Point, Quaternion


@dataclass
class NavigationConfig:
    reach_tolerance: float = 0.5
    through_tolerance: float = 3.0
    is_through_point: bool = True


@dataclass
class ActionConfig:
    type: str  # "service" | "publish" | "load_map" | "amcl_reset" | "wait" | "wait_trigger"

    # service
    service: str = ""
    srv_module: str = ""
    srv_class: str = ""
    request: Dict[str, Any] = field(default_factory=dict)

    # publish
    topic: str = ""
    msg_module: str = ""
    msg_class: str = ""
    data: Dict[str, Any] = field(default_factory=dict)

    # load_map
    localization: str = ""
    planning: str = ""

    # wait
    countdown_ms: int = 3000

    def to_dict(self) -> dict:
        d: Dict[str, Any] = {"type": self.type}
        if self.type == "service":
            d["service"] = self.service
            d["srv_module"] = self.srv_module
            d["srv_class"] = self.srv_class
            if self.request:
                d["request"] = self.request
        elif self.type == "publish":
            d["topic"] = self.topic
            d["msg_module"] = self.msg_module
            d["msg_class"] = self.msg_class
            if self.data:
                d["data"] = self.data
        elif self.type == "load_map":
            d["localization"] = self.localization
            d["planning"] = self.planning
        elif self.type == "wait":
            d["countdown_ms"] = self.countdown_ms
        elif self.type == "wait_trigger":
            pass
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "ActionConfig":
        action_type = d["type"]
        if action_type == "service":
            return cls(
                type=action_type,
                service=d.get("service", ""),
                srv_module=d.get("srv_module", ""),
                srv_class=d.get("srv_class", ""),
                request=d.get("request", {}),
            )
        elif action_type == "publish":
            return cls(
                type=action_type,
                topic=d.get("topic", ""),
                msg_module=d.get("msg_module", ""),
                msg_class=d.get("msg_class", ""),
                data=d.get("data", {}),
            )
        elif action_type == "load_map":
            return cls(
                type=action_type,
                localization=d.get("localization", ""),
                planning=d.get("planning", ""),
            )
        elif action_type == "amcl_reset":
            return cls(type=action_type)
        elif action_type == "wait":
            return cls(type=action_type, countdown_ms=d.get("countdown_ms", 3000))
        elif action_type == "wait_trigger":
            return cls(type=action_type)
        else:
            return cls(type=action_type)


@dataclass
class Waypoint:
    index: int
    pose: PoseStamped
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    on_reached_actions: List[ActionConfig] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "index": self.index,
            "pose": {
                "position": {
                    "x": self.pose.pose.position.x,
                    "y": self.pose.pose.position.y,
                    "z": self.pose.pose.position.z,
                },
                "orientation": {
                    "x": self.pose.pose.orientation.x,
                    "y": self.pose.pose.orientation.y,
                    "z": self.pose.pose.orientation.z,
                    "w": self.pose.pose.orientation.w,
                },
            },
            "navigation": {
                "reach_tolerance": self.navigation.reach_tolerance,
                "through_tolerance": self.navigation.through_tolerance,
                "is_through_point": self.navigation.is_through_point,
            },
            "on_reached_actions": [a.to_dict() for a in self.on_reached_actions],
        }


class WaypointList:
    def __init__(self):
        self.waypoints: List[Waypoint] = []

    def add(self, waypoint: Waypoint) -> None:
        self.waypoints.append(waypoint)

    def remove(self, index: int) -> None:
        self.waypoints.pop(index)

    def update_pose(self, index: int, pose: PoseStamped) -> None:
        self.waypoints[index].pose = pose

    def get(self, index: int) -> Waypoint:
        return self.waypoints[index]

    def get_all(self) -> List[Waypoint]:
        return self.waypoints

    def get_size(self) -> int:
        return len(self.waypoints)

    def clear(self) -> None:
        self.waypoints = []

    def get_next_index(self) -> int:
        return self.get_size()

    def sort_by_index(self) -> None:
        self.waypoints.sort(key=lambda w: w.index)


def get_index_from_waypoint_name(waypoint_name: str) -> int:
    return int(waypoint_name.split("_")[1])


class WaypointsLoader:
    def __init__(self, file_path: str):
        self.file_path = file_path

    def load(self) -> WaypointList:
        with open(self.file_path, "r") as f:
            raw = yaml.safe_load(f)

        if not (isinstance(raw, dict) and raw.get("version") == "2.0"):
            raise ValueError(
                f"Unsupported waypoint file format in '{self.file_path}'. "
                "Only version 2.0 is supported. "
                "Run 'migrate_waypoints.py <input.yaml> <output.yaml>' to convert."
            )

        return self._load_v2(raw)

    def _load_v2(self, raw: dict) -> WaypointList:
        defaults_raw = raw.get("defaults", {})
        default_nav = NavigationConfig(
            reach_tolerance=defaults_raw.get("reach_tolerance", 0.5),
            through_tolerance=defaults_raw.get("through_tolerance", 3.0),
            is_through_point=defaults_raw.get("is_through_point", True),
        )

        waypoints = WaypointList()
        for wp_raw in raw.get("waypoints", []):
            nav_raw = wp_raw.get("navigation", {})
            nav = NavigationConfig(
                reach_tolerance=nav_raw.get(
                    "reach_tolerance", default_nav.reach_tolerance
                ),
                through_tolerance=nav_raw.get(
                    "through_tolerance", default_nav.through_tolerance
                ),
                is_through_point=nav_raw.get(
                    "is_through_point", default_nav.is_through_point
                ),
            )
            pose = self._parse_pose(wp_raw["pose"])
            actions = [
                ActionConfig.from_dict(a)
                for a in wp_raw.get("on_reached_actions", [])
            ]
            waypoints.add(
                Waypoint(
                    index=wp_raw["index"],
                    pose=pose,
                    navigation=nav,
                    on_reached_actions=actions,
                )
            )
        waypoints.sort_by_index()
        return waypoints

    @staticmethod
    def _parse_pose(pose_raw: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position = Point(**pose_raw["position"])
        pose.pose.position.z = 1.0
        pose.pose.orientation = Quaternion(**pose_raw["orientation"])
        return pose


class WaypointsSaver:
    def __init__(self, file_path: str):
        self.file_path = file_path

    def save(self, waypoints: WaypointList) -> None:
        output = {
            "version": "2.0",
            "waypoints": [w.to_dict() for w in waypoints.get_all()],
        }
        with open(self.file_path, "w") as f:
            yaml.dump(output, f, allow_unicode=True, default_flow_style=False)
