#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped, Point, Quaternion

from dataclasses import dataclass
from typing import List
import yaml
import enum
import pprint


class OnReachedAction(enum.Enum):
    WAIT_TRIGGER = "wait_trigger"
    RELOAD_MAP = "reload_map"
    FROMT_LIDAR_OFF = "front_lidar_off"
    FRONT_LIDAR_ON = "front_lidar_on"
    AMCL_ON = "amcl_on"
    AMCL_OFF = "amcl_off"
    GPS_ON = "gps_on"
    GPS_OFF = "gps_off"

    @classmethod
    def get_all_values(cls):
        return [action.value for action in cls]


@dataclass
class Waypoint:
    index: int
    pose: PoseStamped

    reach_tolerance: float  # not through only
    on_reached_action: List[OnReachedAction]

    is_through_point: bool = True
    localization_map_yaml: str = ""
    planning_map_yaml: str = ""

    def to_dict(self):
        return {
            'index': self.index,
            'pose': {
                'position': {
                    'x': self.pose.pose.position.x,
                    'y': self.pose.pose.position.y,
                    'z': self.pose.pose.position.z
                },
                'orientation': {
                    'x': self.pose.pose.orientation.x,
                    'y': self.pose.pose.orientation.y,
                    'z': self.pose.pose.orientation.z,
                    'w': self.pose.pose.orientation.w
                }
            },
            'reach_tolerance': self.reach_tolerance,
            'on_reached_action': self._on_reached_action_to_string_list(),
            "is_through_point": self.is_through_point,
            'localization_map_yaml': self.localization_map_yaml,
            'planning_map_yaml': self.planning_map_yaml,
        }

    def _on_reached_action_to_string_list(self):
        return [action.value for action in self.on_reached_action]


class WaypointList:
    def __init__(self):
        self.waypoints = []

    def add(self, waypoint: Waypoint):
        self.waypoints.append(waypoint)

    def remove(self, index: int):
        self.waypoints.pop(index)

    def update_pose(self, index: int, pose: PoseStamped):
        self.waypoints[index].pose = pose

    def get(self, index: int) -> Waypoint:
        return self.waypoints[index]

    def get_all(self) -> list:
        return self.waypoints

    def get_size(self) -> int:
        return len(self.waypoints)

    def clear(self):
        self.waypoints = []

    def get_next_index(self) -> int:
        return self.get_size()

    def sort_by_index(self):
        self.waypoints.sort(key=lambda waypoint: waypoint.index)


def get_index_from_waypoint_name(waypoint_name):
    return int(waypoint_name.split('_')[1])


class WaypointsLoader:
    def __init__(self, file_path):
        self.file_path = file_path

    def load(self) -> WaypointList:
        waypoints = WaypointList()

        with open(self.file_path, 'r') as file:
            waypoints_yaml = yaml.safe_load(file)

        for waypoint in waypoints_yaml:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position = Point(**waypoint['pose']['position'])
            pose.pose.position.z = 1.0
            pose.pose.orientation = Quaternion(
                **waypoint['pose']['orientation'])

            waypoints.add(
                Waypoint(
                    index=waypoint['index'],
                    pose=pose,
                    reach_tolerance=waypoint['reach_tolerance'],
                    on_reached_action=[
                        OnReachedAction(action)
                        for action in waypoint['on_reached_action']
                    ],
                    localization_map_yaml=waypoint['localization_map_yaml'] if 'localization_map_yaml' in waypoint else "",
                    planning_map_yaml=waypoint['planning_map_yaml'] if 'planning_map_yaml' in waypoint else "",
                    is_through_point=waypoint['is_through_point'] if 'is_through_point' in waypoint else True,
                )
            )

        waypoints.sort_by_index()
        return waypoints


class WaypointsSaver:
    def __init__(self, file_path):
        self.file_path = file_path

    def save(self, waypoints: WaypointList):
        waypoints_dict = []
        for waypoint in waypoints.get_all():
            waypoints_dict.append(waypoint.to_dict())

        with open(self.file_path, 'w') as file:
            yaml.dump(waypoints_dict, file)
