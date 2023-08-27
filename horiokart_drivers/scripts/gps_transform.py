#!/usr/bin/env python3

from pyproj import Proj
import math
import dataclasses
import numpy as np
from scipy.optimize import minimize
import yaml


@dataclasses.dataclass
class Pose:
    x: float
    y: float
    yaw: float


@dataclasses.dataclass
class UTMtoMAP_RefPoint:
    utm: Pose  # utm_position. unit: m. don't use yaw
    map: Pose  # map_position. unit: m. don't use yaw


class GpsTransform:
    UTM2MAP_APPEND_THRESHOLD = 30  # unit: m
    NUM_USE_UTM2MAP_REFPOINT = 3

    def __init__(self, offset_yaw: float = 0.0):
        self._default_offset_yaw = offset_yaw

        self.init_parameters()

    def init_parameters(self):
        self.proj = None
        self.origin_utm = None
        self._previous_pose = None
        self._last_utm = None

        self._utm_to_map_list = []

    @property
    def last_utm(self):
        return self._last_utm if self._last_utm is not None else Pose(0, 0, 0)

    def add_utm_to_map_refpoint(self, utm: Pose, map: Pose):
        if len(self._utm_to_map_list) == 0:
            self._utm_to_map_list.append(UTMtoMAP_RefPoint(utm, map))
        elif (utm.x - self._utm_to_map_list[-1].utm.x)**2 + (utm.y - self._utm_to_map_list[-1].utm.y)**2 > self.UTM2MAP_APPEND_THRESHOLD ** 2:
            self._utm_to_map_list.append(UTMtoMAP_RefPoint(utm, map))

    def save_utm_to_map_list(self, file_path: str):
        with open(file_path, 'w') as f:
            yaml.dump(self._utm_to_map_list, f)

    def load_utm_to_map_list(self, file_path: str):
        with open(file_path, 'r') as f:
            self._utm_to_map_list = yaml.load(f, Loader=yaml.FullLoader)

        print(self._utm_to_map_list)

    def find_point_d(self, a, b, c, ad_distance, bd_distance, cd_distance):
        # 点A、B、Cの座標を取得
        xa, ya = a
        xb, yb = b
        xc, yc = c

        # 点Aから点Dまでのベクトルを計算
        ad_vector = [xa - xb, ya - yb]
        # 点Bから点Dまでのベクトルを計算
        bd_vector = [xb - xa, yb - ya]
        # 点Cから点Dまでのベクトルを計算
        cd_vector = [xc - xa, yc - ya]

        # ベクトルの大きさを求める
        ad_magnitude = math.sqrt(ad_vector[0]**2 + ad_vector[1]**2)
        bd_magnitude = math.sqrt(bd_vector[0]**2 + bd_vector[1]**2)
        cd_magnitude = math.sqrt(cd_vector[0]**2 + cd_vector[1]**2)

        # 各ベクトルを距離に合わせてスケーリング
        ad_scaled = [ad_vector[0] * ad_distance / ad_magnitude,
                     ad_vector[1] * ad_distance / ad_magnitude]
        bd_scaled = [bd_vector[0] * bd_distance / bd_magnitude,
                     bd_vector[1] * bd_distance / bd_magnitude]
        cd_scaled = [cd_vector[0] * cd_distance / cd_magnitude,
                     cd_vector[1] * cd_distance / cd_magnitude]

        # 点Dの座標を計算
        xd = xa + ad_scaled[0] + bd_scaled[0] + cd_scaled[0]
        yd = ya + ad_scaled[1] + bd_scaled[1] + cd_scaled[1]

        return xd, yd

    def distance_error(self, point, known_points, distances):
        errors = []
        for i in range(len(known_points)):
            error = np.linalg.norm(known_points[i] - point) - distances[i]
            errors.append(error)
        return errors

    def estimate_point(self, known_points, distances, initial_guess):
        result = minimize(
            lambda p: np.sum(
                np.square(self.distance_error(p, known_points, distances))),
            initial_guess
        )
        estimated_point = result.x
        return estimated_point

    def calc_utm_to_map_offset(self, utm: Pose) -> Pose:
        if len(self._utm_to_map_list) == 0:
            return Pose(
                utm.x,
                utm.y,
                self._default_offset_yaw)

        # use first data. need yaw
        elif len(self._utm_to_map_list) == 1:
            # calculate offset utm to map from first data
            offset_x = self._utm_to_map_list[0].map.x - \
                self._utm_to_map_list[0].utm.x
            offset_y = self._utm_to_map_list[0].map.y - \
                self._utm_to_map_list[0].utm.y
            offset_yaw = self._utm_to_map_list[0].utm.yaw

            return Pose(offset_x, offset_y, offset_yaw)

        # use multi data. don't use yaw
        elif len(self._utm_to_map_list) >= 2:
            distance_list = []
            for utm_to_map in self._utm_to_map_list:
                distance_list.append(
                    math.sqrt((utm.x - utm_to_map.utm.x)**2 + (utm.y - utm_to_map.utm.y)**2)
                )

            # get sorted index by distance
            sorted_index = sorted(
                range(len(distance_list)),
                key=lambda k: distance_list[k])
            # get top N utm_to_map
            use_utm_to_map_list = []
            use_distance_list = []
            for i in range(min(self.NUM_USE_UTM2MAP_REFPOINT, len(sorted_index))):
                use_utm_to_map_list.append(
                    self._utm_to_map_list[sorted_index[i]])
                use_distance_list.append(distance_list[sorted_index[i]])

            # # calculate offset utm to map each data
            # offset_x_list = []
            # offset_y_list = []
            # for utm_to_map in use_utm_to_map_list:
            #     offset_x_list.append(
            #         utm_to_map.map.x - utm_to_map.utm.x)
            #     offset_y_list.append(
            #         utm_to_map.map.y - utm_to_map.utm.y)
            # # calculate average offset utm to map
            # offset_x = sum(offset_x_list) / len(offset_x_list)
            # offset_y = sum(offset_y_list) / len(offset_y_list)
            offset_x, offset_y = self.estimate_point(
                known_points=np.array([[utm_to_map.map.x, utm_to_map.map.y] for utm_to_map in use_utm_to_map_list]),
                distances=np.array(use_distance_list),
                initial_guess=np.array([0.0, 0.0])
            )

            return Pose(offset_x, offset_y, 0.0)

    def to_utm(self, lat, lon) -> Pose:
        if self.proj is None:
            zone = int(lon // 6) + 31
            self.proj = Proj(proj='utm', zone=zone, ellps='WGS84')
        x, y = self.proj(lon, lat)

        return Pose(x, y, 0.0)

    def transform_to_map(self, lat, lon) -> Pose:
        self._last_utm = self.to_utm(lat, lon)
        if self.origin_utm is None:
            self.origin_utm = Pose(
                self._last_utm.x,
                self._last_utm.y,
                self._default_offset_yaw)

        utm_to_map_offset = self.calc_utm_to_map_offset(self._last_utm)
        print("---------------")
        print(utm_to_map_offset)
        print("---------------")

        return Pose(
            # self._last_utm.x + utm_to_map_offset.x,
            # self._last_utm.y + utm_to_map_offset.y,
            utm_to_map_offset.x,
            utm_to_map_offset.y,
            utm_to_map_offset.y)

    def _transform_to_map(self, lat, lon) -> Pose:
        self._last_utm = self.to_utm(lat, lon)

        # calculate offset from origin
        if self.origin_utm is None:
            self.origin_utm = Pose(
                self._last_utm.x,
                self._last_utm.y,
                self._default_offset_yaw)

        map_position = Pose(
            self._last_utm.x - self.origin_utm.x,
            self._last_utm.y - self.origin_utm.y,
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
        if distance < 0.3:  # unit: m
            return

        current_yaw = math.atan2(
            self._previous_pose.y, self._previous_pose.x)
        actual_yaw = math.atan2(
            robot_pose_of_map.y, robot_pose_of_map.x)
        diff_yaw = actual_yaw - current_yaw

        self.origin_utm.yaw += diff_yaw
