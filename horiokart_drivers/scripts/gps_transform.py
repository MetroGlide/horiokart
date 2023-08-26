#!/usr/bin/env python3

from pyproj import Proj
import math
import dataclasses


@dataclasses.dataclass
class Pose:
    x: float
    y: float
    yaw: float


class GpsTransform:
    def __init__(self, offset_yaw: float = 0.0):
        self._default_offset_yaw = offset_yaw

        self.init_parameters()

    def init_parameters(self):
        self.proj = None
        self.origin_utm = None
        self._previous_pose = None

    def to_utm(self, lat, lon) -> Pose:
        if self.proj is None:
            zone = int(lon // 6) + 31
            self.proj = Proj(proj='utm', zone=zone, ellps='WGS84')
        x, y = self.proj(lon, lat)

        return Pose(x, y, 0.0)

    def transform_to_map(self, lat, lon) -> Pose:
        utm_position = self.to_utm(lat, lon)

        # calculate offset from origin
        if self.origin_utm is None:
            self.origin_utm = Pose(
                utm_position.x,
                utm_position.y,
                self._default_offset_yaw)

        map_position = Pose(
            utm_position.x - self.origin_utm.x,
            utm_position.y - self.origin_utm.y,
            self.origin_utm.yaw)

        # rotate
        map_position = self.rotate(map_position)

        # calculate yaw from previous pose
        if self._previous_pose is not None:
            map_position.yaw = math.atan2(
                map_position.y - self._previous_pose.y,
                map_position.x - self._previous_pose.x)

        # update previous pose
        self._previous_pose = Pose(
            map_position.x,
            map_position.y,
            map_position.yaw)

        return map_position

    def rotate(self, pose) -> Pose:
        if self.origin_utm is None:
            return pose

        x_rot = pose.x * \
            math.cos(self.origin_utm.yaw) - pose.y * \
            math.sin(self.origin_utm.yaw)
        y_rot = pose.x * \
            math.sin(self.origin_utm.yaw) + pose.y * \
            math.cos(self.origin_utm.yaw)
        yaw_rot = pose.yaw

        return Pose(x_rot, y_rot, yaw_rot)

    def feedback_pose(self, robot_pose_of_map: Pose):
        if self.origin_utm is None or self._previous_pose is None:
            return

        distance = math.sqrt(
            (robot_pose_of_map.x - self._previous_pose.x)**2 +
            (robot_pose_of_map.y - self._previous_pose.y)**2)
        if distance < 0.3: # unit: m
            return

        current_yaw = math.atan2(
            self._previous_pose.y, self._previous_pose.x)
        actual_yaw = math.atan2(
            robot_pose_of_map.y, robot_pose_of_map.x)
        diff_yaw = actual_yaw - current_yaw

        self.origin_utm.yaw += diff_yaw

