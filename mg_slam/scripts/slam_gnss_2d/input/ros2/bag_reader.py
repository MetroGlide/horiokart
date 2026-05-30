from __future__ import annotations

import bisect
import math
from typing import Callable, Optional

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from ..base import GnssSourceBase, OdomSourceBase, ScanSourceBase
from ...data_types import GnssData, OdomData, ScanData


def _quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _open_reader(
    bag_path: str, topics: list[str]
) -> rosbag2_py.SequentialReader:
    if not bag_path:
        raise ValueError(
            'bag_path が設定されていません。'
            'ROSBAG_FILE 環境変数を設定するか bag_path 引数を指定してください。'
        )
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


class BagScanSource(ScanSourceBase):
    """rosbag2 から ScanData をステップ駆動で供給するソース。"""

    def __init__(self, bag_path: str, scan_topic: str) -> None:
        self._bag_path = bag_path
        self._scan_topic = scan_topic
        self._callback: Optional[Callable[[ScanData], None]] = None
        self._lidar_yaw: float = 0.0
        self._reader: Optional[rosbag2_py.SequentialReader] = None
        self._step_count = 0

    def set_scan_callback(self, callback: Callable[[ScanData], None]) -> None:
        self._callback = callback

    def start(self) -> None:
        self._lidar_yaw = self._resolve_lidar_yaw()
        self._reader = _open_reader(self._bag_path, [self._scan_topic])

    def stop(self) -> None:
        self._reader = None

    def step(self) -> bool:
        """次の LaserScan を1件読んでコールバックを同期呼び出しする。

        Returns:
            True: 処理成功。False: bag を読み終えた。
        """
        if self._reader is None or not self._reader.has_next():
            return False
        (_, data, _) = self._reader.read_next()
        msg = deserialize_message(data, LaserScan)
        self._step_count += 1
        if self._callback is not None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._callback(ScanData(
                timestamp=stamp,
                ranges=np.array(msg.ranges, dtype=np.float32),
                angle_min=msg.angle_min + self._lidar_yaw,
                angle_increment=msg.angle_increment,
                range_min=msg.range_min,
                range_max=msg.range_max,
            ))
        return True

    def _resolve_lidar_yaw(self) -> float:
        """bag 内の /tf_static から lidar→base_link の yaw を解決する。

        ROS2ScanSource と同じ lookup_transform('base_link', scan_frame_id) に相当する
        変換を bag から直接取得する。
        """
        reader = _open_reader(
            self._bag_path, ['/tf_static', self._scan_topic])
        tf_map: dict[tuple[str, str], object] = {}
        scan_frame_id: Optional[str] = None

        while reader.has_next():
            (topic, data, _) = reader.read_next()
            if topic == '/tf_static':
                msg = deserialize_message(data, TFMessage)
                for tf in msg.transforms:
                    tf_map[(tf.header.frame_id, tf.child_frame_id)] = \
                        tf.transform.rotation
            elif topic == self._scan_topic and scan_frame_id is None:
                msg = deserialize_message(data, LaserScan)
                scan_frame_id = msg.header.frame_id
                break

        if scan_frame_id is None:
            return 0.0

        # /tf_static は個々のジョイント変換しか持たないため、
        # scan_frame_id から base_link まで TF ツリーをたどって yaw を合成する。
        # 例: top_lrf_link(yaw=π) → top_frame_link(yaw=0) → base_link = π
        child_to_parent: dict[str, tuple[str, object]] = {
            child: (parent, rot)
            for (parent, child), rot in tf_map.items()
        }
        yaw = 0.0
        current = scan_frame_id
        visited: set[str] = set()
        while current != 'base_link':
            if current in visited or current not in child_to_parent:
                return 0.0
            visited.add(current)
            parent, rot = child_to_parent[current]
            yaw += _quaternion_to_yaw(rot)
            current = parent
        return yaw


class BagOdomSource(OdomSourceBase):
    """rosbag2 から全 OdomData を事前ロードして補間検索するソース。"""

    def __init__(self, bag_path: str, odom_topic: str) -> None:
        self._bag_path = bag_path
        self._odom_topic = odom_topic
        self._odom_list: list[OdomData] = []
        self._timestamps: list[float] = []

    def start(self) -> None:
        reader = _open_reader(self._bag_path, [self._odom_topic])
        while reader.has_next():
            (_, data, _) = reader.read_next()
            msg = deserialize_message(data, Odometry)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            yaw = _quaternion_to_yaw(msg.pose.pose.orientation)
            self._odom_list.append(OdomData(
                timestamp=stamp,
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                yaw=yaw,
            ))
        self._odom_list.sort(key=lambda o: o.timestamp)
        self._timestamps = [o.timestamp for o in self._odom_list]

    def stop(self) -> None:
        pass

    def get_odom_at(self, timestamp: float) -> Optional[OdomData]:
        if not self._odom_list:
            return None
        idx = bisect.bisect_left(self._timestamps, timestamp)
        if idx == 0:
            return self._odom_list[0]
        if idx >= len(self._odom_list):
            return self._odom_list[-1]
        prev = self._odom_list[idx - 1]
        next_ = self._odom_list[idx]
        t_span = next_.timestamp - prev.timestamp
        if t_span < 1e-9:
            return prev
        alpha = (timestamp - prev.timestamp) / t_span
        return OdomData(
            timestamp=timestamp,
            x=prev.x + alpha * (next_.x - prev.x),
            y=prev.y + alpha * (next_.y - prev.y),
            yaw=prev.yaw + alpha * _angle_diff(next_.yaw, prev.yaw),
        )


class BagGnssSource(GnssSourceBase):
    """rosbag2 から全 GnssData を事前ロードして提供するソース。

    start() 呼び出し時に NavSatFix メッセージを全件読み込み、
    pyproj で UTM 平面直角座標に変換して内部バッファに格納する。
    UTM zone は最初の有効な fix から自動決定する。
    """

    def __init__(self, bag_path: str, gnss_topic: str) -> None:
        self._bag_path = bag_path
        self._gnss_topic = gnss_topic
        self._gnss_list: list[GnssData] = []
        self._timestamps: list[float] = []

    def start(self) -> None:
        from pyproj import CRS, Transformer
        from sensor_msgs.msg import NavSatFix

        reader = _open_reader(self._bag_path, [self._gnss_topic])
        raw_fixes: list = []
        while reader.has_next():
            (_, data, _) = reader.read_next()
            msg = deserialize_message(data, NavSatFix)
            # STATUS_NO_FIX = -1 を除外する
            if msg.status.status < 0:
                continue
            raw_fixes.append(msg)

        if not raw_fixes:
            return

        # 最初の fix から UTM zone を自動決定して変換器を構築する
        first = raw_fixes[0]
        zone = int((first.longitude + 180.0) / 6.0) + 1
        south = first.latitude < 0.0
        crs_utm = CRS.from_dict({'proj': 'utm', 'zone': zone, 'south': south})
        transformer = Transformer.from_crs(
            'EPSG:4326', crs_utm, always_xy=True)

        for msg in raw_fixes:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            x, y = transformer.transform(msg.longitude, msg.latitude)

            # NavSatFix.position_covariance は ENU 9 要素配列。East/North の 2x2 を取り出す
            cov = msg.position_covariance
            cov_2x2 = np.array([[cov[0], cov[1]], [cov[3], cov[4]]])

            self._gnss_list.append(GnssData(
                timestamp=stamp,
                x=x,
                y=y,
                covariance=cov_2x2,
            ))

        self._gnss_list.sort(key=lambda g: g.timestamp)
        self._timestamps = [g.timestamp for g in self._gnss_list]

    def stop(self) -> None:
        pass

    def get_gnss_at(self, timestamp: float) -> Optional[GnssData]:
        if not self._gnss_list:
            return None
        idx = bisect.bisect_left(self._timestamps, timestamp)
        if idx == 0:
            return self._gnss_list[0]
        if idx >= len(self._gnss_list):
            return self._gnss_list[-1]
        prev = self._gnss_list[idx - 1]
        next_ = self._gnss_list[idx]
        t_span = next_.timestamp - prev.timestamp
        if t_span < 1e-9:
            return prev
        alpha = (timestamp - prev.timestamp) / t_span
        return GnssData(
            timestamp=timestamp,
            x=prev.x + alpha * (next_.x - prev.x),
            y=prev.y + alpha * (next_.y - prev.y),
            covariance=prev.covariance + alpha *
            (next_.covariance - prev.covariance),
        )

    def get_all_gnss(self) -> list[GnssData]:
        return list(self._gnss_list)
