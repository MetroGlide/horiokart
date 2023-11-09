#!/usr/bin/env python3

import os
from pyproj import Proj
import math
import dataclasses
import numpy as np
from scipy.optimize import minimize
import yaml

import enum

import matplotlib.pyplot as plt
import matplotlib.patches as patches

from typing import List


@dataclasses.dataclass
class Pose:
    x: float
    y: float
    yaw: float

    def copy(self):
        return Pose(self.x, self.y, self.yaw)

    def to_dict(self):
        return {
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw
        }

    @classmethod
    def from_dict(cls, dict):
        return cls(
            dict['x'],
            dict['y'],
            dict['yaw']
        )


@dataclasses.dataclass
class UTMtoMAP_RefPoint:
    utm: Pose  # utm_position. unit: m
    map: Pose  # map_position. unit: m
    variance: float = 0.0  # unit: m

    def to_dict(self):
        return {
            'utm': self.utm.to_dict(),
            'map': self.map.to_dict(),
            'variance': self.variance
        }

    @classmethod
    def from_dict(cls, dict):
        return cls(
            Pose.from_dict(dict['utm']),
            Pose.from_dict(dict['map']),
            dict['variance']
        )


class ConverterBase:
    def __init__(self):
        self._previous_pose = None

        self._previous_yaw_calc_pose = None
        self._yaw_calc_threshold = 1.0  # unit: m

        self._odom_yaw = 0.0
        self._previous_odom_yaw = None
        self._odom_offset_yaw = 0.0

    def calc_map_pose(self, utm: Pose, ref_points: List[UTMtoMAP_RefPoint]) -> Pose:
        pass

    def get_yaw(self, estimated_x: float, estimated_y: float) -> float:
        yaw_from_pose = self.calc_yaw_from_pose(estimated_x, estimated_y)
        # return yaw_from_pose

        diff_yaw = yaw_from_pose - self._odom_yaw
        diff_yaw = self._normalize_yaw(diff_yaw)

        coef = 0.01
        self._odom_offset_yaw = diff_yaw * coef + \
            self._odom_offset_yaw * (1.0 - coef)

        
        # self._odom_yaw += self._odom_offset_yaw
        self._odom_yaw += diff_yaw * 0.1
        # self._previous_yaw_calc_pose.yaw = self._odom_yaw

        return self._odom_yaw

    def calc_yaw_from_pose(self, estimated_x: float, estimated_y: float) -> float:
        # calculate yaw from previous pose
        if self._previous_yaw_calc_pose is None:
            yaw = self._odom_yaw

            self._previous_yaw_calc_pose = Pose(
                estimated_x,
                estimated_y,
                yaw)

            return yaw

        # previous pose is not None
        elif (self._previous_yaw_calc_pose.x - estimated_x)**2 + (self._previous_yaw_calc_pose.y - estimated_y)**2 > self._yaw_calc_threshold**2:
            yaw = math.atan2(
                estimated_y - self._previous_yaw_calc_pose.y,
                estimated_x - self._previous_yaw_calc_pose.x)

            self._previous_yaw_calc_pose = Pose(
                estimated_x,
                estimated_y,
                yaw)

            return yaw

        # distance is too short
        else:
            yaw = self._previous_yaw_calc_pose.yaw
            return yaw

    def _normalize_yaw(self, yaw: float) -> float:
        while yaw > math.pi:
            yaw -= math.pi * 2
        while yaw < -math.pi:
            yaw += math.pi * 2
        return yaw

    def odom_yaw_feedback(self, odom_yaw: float):
        # self._odom_yaw = odom_yaw

        if self._previous_odom_yaw is None:
            self._previous_odom_yaw = odom_yaw
            return

        diff_yaw = odom_yaw - self._previous_odom_yaw
        diff_yaw = self._normalize_yaw(diff_yaw)

        self._odom_yaw += diff_yaw
        self._previous_odom_yaw = odom_yaw

    def init_odom_yaw(self, odom_yaw: float):
        self._odom_yaw = odom_yaw
        self._previous_odom_yaw = odom_yaw
        self._odom_offset_yaw = 0.0

        if self._previous_pose is not None:
            self._previous_pose.yaw = odom_yaw


class SingleRefConverter(ConverterBase):
    # 初期位置をゼロとして、角度のみ都度補正する

    def calc_map_pose(self, utm: Pose, ref_points: List[UTMtoMAP_RefPoint]) -> Pose:
        # calculate offset from origin
        if len(ref_points) == 0:
            origin_utm = Pose(
                utm.x,
                utm.y,
                2.75)
        else:
            origin_utm = Pose(
                ref_points[0].utm.x,
                ref_points[0].utm.y,
                # ref_points[0].utm.yaw)
                2.75)

        map_position = Pose(
            utm.x - origin_utm.x,
            utm.y - origin_utm.y,
            origin_utm.yaw)

        # rotate
        map_position = self.rotate(map_position, origin_utm)

        estimated_x = map_position.x
        estimated_y = map_position.y

        # calculate yaw from previous pose
        yaw = self.get_yaw(estimated_x, estimated_y)

        # update previous pose
        self._previous_pose = Pose(
            map_position.x,
            map_position.y,
            yaw)

        return Pose(estimated_x, estimated_y, yaw)

    def rotate(self, pose, origin_utm) -> Pose:
        if origin_utm is None:
            return pose

        x_rot = pose.x * \
            math.cos(origin_utm.yaw) - pose.y * \
            math.sin(origin_utm.yaw)
        y_rot = pose.x * \
            math.sin(origin_utm.yaw) + pose.y * \
            math.cos(origin_utm.yaw)
        yaw_rot = pose.yaw

        return Pose(x_rot, y_rot, yaw_rot)

    def feedback(self, robot_pose_of_map: Pose, origin_utm: Pose) -> Pose:
        if origin_utm is None or self._previous_pose is None:
            return origin_utm

        distance = math.sqrt(
            (robot_pose_of_map.x - self._previous_pose.x)**2 +
            (robot_pose_of_map.y - self._previous_pose.y)**2)
        if distance < 0.3:  # unit: m
            return origin_utm

        current_yaw = math.atan2(
            self._previous_pose.y, self._previous_pose.x)
        actual_yaw = math.atan2(
            robot_pose_of_map.y, robot_pose_of_map.x)
        diff_yaw = actual_yaw - current_yaw

        origin_utm.yaw += diff_yaw

        return origin_utm


class MultiRefConverter(ConverterBase):
    NUM_USE_UTM2MAP_REFPOINT = 5
    VISUALIZE = False
    # VISUALIZE = True

    # 参照点を複数設定し、座標と距離から最適化計算する

    def calc_map_pose(self, utm: Pose, ref_points: List[UTMtoMAP_RefPoint]) -> Pose:
        if len(ref_points) < 2:
            return Pose(0.0, 0.0, 0.0)

        distance_list = []
        for utm_to_map in ref_points:
            distance_list.append(
                math.sqrt((utm.x - utm_to_map.utm.x)**2 +
                          (utm.y - utm_to_map.utm.y)**2)
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
                ref_points[sorted_index[i]])
            use_distance_list.append(distance_list[sorted_index[i]])

        estimated_x, estimated_y = self.estimate_point(
            known_points=np.array(
                [[utm_to_map.map.x, utm_to_map.map.y] for utm_to_map in use_utm_to_map_list]),
            distances=np.array(use_distance_list),
            initial_guess=np.array(
                [use_utm_to_map_list[0].map.x, use_utm_to_map_list[0].map.y])
        )

        # visualize for debug
        if self.VISUALIZE:
            self.visualize_utm_to_map_offset(
                [utm_to_map.map for utm_to_map in use_utm_to_map_list],
                use_distance_list,
                Pose(estimated_x, estimated_y, 0.0)
            )

        # calculate yaw from previous pose
        yaw = self.get_yaw(estimated_x, estimated_y)

        # update previous pose
        self._previous_pose = Pose(
            estimated_x,
            estimated_y,
            yaw)

        return Pose(estimated_x, estimated_y, yaw)

    def estimate_point(self, known_points, distances, initial_guess):
        def inverse_distance_weight(distance, power):
            return 1.0 / np.power(distance, power)

        def distance_error(point, known_points, distances, power):
            errors = []
            for i in range(len(known_points)):
                error = np.linalg.norm(known_points[i] - point) - distances[i]
                error *= inverse_distance_weight(distances[i], power)
                errors.append(error)
            return errors

        power = 1.0
        result = minimize(
            lambda p: np.sum(
                np.square(distance_error(p, known_points, distances, power))),
            initial_guess
        )
        estimated_point = result.x
        return estimated_point

    def visualize_utm_to_map_offset(self, ref_point_list: list, distance_list: list, point: Pose):
        # clear
        plt.clf()

        # visualize ref points and point
        plt.scatter([ref_point.x for ref_point in ref_point_list], [
                    ref_point.y for ref_point in ref_point_list], c='red')
        plt.scatter(point.x, point.y, c='blue')

        limit_min = min(min([ref_point.x for ref_point in ref_point_list]),
                        min([ref_point.y for ref_point in ref_point_list]))
        limit_max = max(max([ref_point.x for ref_point in ref_point_list]),
                        max([ref_point.y for ref_point in ref_point_list]))
        plt.xlim(limit_min - 5, limit_max + 5)
        plt.ylim(limit_min - 5, limit_max + 5)

        # visualize distance as circle centering on ref points
        for i in range(len(ref_point_list)):
            circle = patches.Circle(xy=(
                ref_point_list[i].x, ref_point_list[i].y), radius=distance_list[i], fill=False)
            plt.gcf().gca().add_artist(circle)

        plt.pause(0.001)


class UTMtoMAPConverter:
    UTM2MAP_APPEND_THRESHOLD = 20  # unit: m
    # UTM_VARIANCE_THRESHOLD = 5.5  # unit: m
    UTM_VARIANCE_THRESHOLD = 25.5  # unit: m

    class ConverterType(enum.IntEnum):
        SINGLE = enum.auto()
        MULTI = enum.auto()
        HYBRID = enum.auto()

    def __init__(self, utm_to_map_list: list, converter_type: ConverterType):
        self._utm_to_map_list = utm_to_map_list
        self._converter_type = converter_type

        self._single_ref_converter = SingleRefConverter()
        self._multi_ref_converter = MultiRefConverter()

    def calc_map_pose(self, utm: Pose) -> Pose:
        if self._converter_type == self.ConverterType.SINGLE:
            return self._single_ref_converter.calc_map_pose(
                utm, self._utm_to_map_list)
        elif self._converter_type == self.ConverterType.MULTI:
            return self._multi_ref_converter.calc_map_pose(
                utm, self._utm_to_map_list)
        elif self._converter_type == self.ConverterType.HYBRID:
            if len(self._utm_to_map_list) >= 2:
                single_estimated_map_pose = self._single_ref_converter.calc_map_pose(
                    utm, self._utm_to_map_list)
                return self._multi_ref_converter.calc_map_pose(
                    utm, [*self._utm_to_map_list,
                          UTMtoMAP_RefPoint(utm, single_estimated_map_pose, UTMtoMAPConverter.UTM_VARIANCE_THRESHOLD)]
                )
            else:
                return self._single_ref_converter.calc_map_pose(
                    utm, self._utm_to_map_list)

        else:
            raise NotImplementedError()

    def add_utm_to_map_refpoint(self, utm: Pose, map: Pose, utm_variance: float) -> bool:
        if len(self._utm_to_map_list) == 0:
            self._utm_to_map_list.append(
                UTMtoMAP_RefPoint(utm, map, utm_variance))
            return True

        distance = (utm.x - self._utm_to_map_list[-1].utm.x)**2 + (
            utm.y - self._utm_to_map_list[-1].utm.y)**2
        if distance > self.UTM2MAP_APPEND_THRESHOLD ** 2 and utm_variance < UTMtoMAPConverter.UTM_VARIANCE_THRESHOLD:
            self._utm_to_map_list.append(
                UTMtoMAP_RefPoint(utm, map, utm_variance))
            return True

        return False

    @property
    def utm_to_map_list(self):
        return self._utm_to_map_list.copy()

    @utm_to_map_list.setter
    def utm_to_map_list(self, utm_to_map_list: list):
        self._utm_to_map_list = utm_to_map_list.copy()

    def feedback_map_pose(self, map_pose: Pose):
        if len(self._utm_to_map_list) == 0:
            return

        self._utm_to_map_list[0].utm = self._single_ref_converter.feedback(
            map_pose, self._utm_to_map_list[0].utm)

    def odom_yaw_feedback(self, odom_yaw: float):
        self._single_ref_converter.odom_yaw_feedback(odom_yaw)
        self._multi_ref_converter.odom_yaw_feedback(odom_yaw)

    def init_odom_yaw(self, odom_yaw: float):
        self._single_ref_converter.init_odom_yaw(odom_yaw)
        self._multi_ref_converter.init_odom_yaw(odom_yaw)


class GpsTransform:

    def __init__(self, offset_yaw: float = 0.0):
        self._default_offset_yaw = offset_yaw

        self.init_parameters()

        self._utm_to_map_converter = UTMtoMAPConverter(
            # [], UTMtoMAPConverter.ConverterType.SINGLE
            [], UTMtoMAPConverter.ConverterType.MULTI
            # [], UTMtoMAPConverter.ConverterType.HYBRID
        )

    def init_parameters(self):
        self.proj = None
        self._last_utm = None

    @property
    def last_utm(self):
        return self._last_utm.copy() if self._last_utm is not None else Pose(0, 0, 0)

    def add_utm_to_map_refpoint(self, utm: Pose, map: Pose, utm_variance: float) -> bool:
        return self._utm_to_map_converter.add_utm_to_map_refpoint(utm, map, utm_variance)

    def feedback_pose(self, map_pose: Pose):
        self._utm_to_map_converter.feedback_map_pose(map_pose)

    def save_utm_to_map_list(self, file_path: str):
        with open(file_path, 'w') as f:
            yaml.dump(
                [utm_to_map.to_dict() for utm_to_map in self._utm_to_map_converter.utm_to_map_list], f)

    def load_utm_to_map_list(self, file_path: str):
        if not os.path.exists(file_path):
            return False
        with open(file_path, 'r') as f:
            utm_to_map_dict_list = yaml.load(
                f, Loader=yaml.FullLoader)
            self._utm_to_map_converter.utm_to_map_list = [
                UTMtoMAP_RefPoint.from_dict(utm_to_map_dict) for utm_to_map_dict in utm_to_map_dict_list
            ]

        print(
            f"load utm_to_map_list: {self._utm_to_map_converter.utm_to_map_list}")
        return True

    def to_utm(self, lat, lon) -> Pose:
        if self.proj is None:
            zone = int(lon // 6) + 31
            self.proj = Proj(proj='utm', zone=zone, ellps='WGS84')
        x, y = self.proj(lon, lat)

        return Pose(x, y, 0.0)

    def transform_to_map(self, lat, lon) -> Pose:
        self._last_utm = self.to_utm(lat, lon)

        map_pose = self._utm_to_map_converter.calc_map_pose(self._last_utm)
        # print("---------------")
        # print(map_pose)
        # print("---------------")

        return Pose(
            map_pose.x,
            map_pose.y,
            map_pose.yaw)

    def odom_yaw_feedback(self, odom_yaw: float):
        self._utm_to_map_converter.odom_yaw_feedback(odom_yaw)

    def init_odom_yaw(self, odom_yaw: float):
        self._utm_to_map_converter.init_odom_yaw(odom_yaw)
