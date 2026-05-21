from __future__ import annotations

import math
from collections import deque
from typing import Callable, Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix

from ..base import GnssSourceBase, OdomSourceBase, ScanSourceBase
from ...data_types import GnssData, OdomData, ScanData

_ODOM_BUFFER_SIZE = 200
_GNSS_BUFFER_SIZE = 1000


def _quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ROS2ScanSource(ScanSourceBase):
    """ROS2 LaserScan トピックから ScanData を供給するアダプター。"""

    def __init__(self, node: Node, topic: str = '/scan') -> None:
        self._node = node
        self._topic = topic
        self._callback: Optional[Callable[[ScanData], None]] = None
        self._sub = None

    def set_scan_callback(self, callback: Callable[[ScanData], None]) -> None:
        self._callback = callback

    def start(self) -> None:
        self._sub = self._node.create_subscription(
            LaserScan, self._topic, self._on_msg, 10
        )

    def stop(self) -> None:
        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
            self._sub = None

    def _on_msg(self, msg: LaserScan) -> None:
        if self._callback is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._callback(ScanData(
            timestamp=stamp,
            ranges=np.array(msg.ranges, dtype=np.float32),
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
        ))


class ROS2OdomSource(OdomSourceBase):
    """ROS2 Odometry トピックから OdomData を供給するアダプター。

    固定長のリングバッファでオドメトリを保持し、タイムスタンプ最近傍検索に対応する。
    topic 引数を '/odom/gnss' に変更するだけで GNSS補正オドメトリに差し替え可能。
    """

    def __init__(self, node: Node, topic: str = '/odom') -> None:
        self._node = node
        self._topic = topic
        self._buffer: deque[OdomData] = deque(maxlen=_ODOM_BUFFER_SIZE)
        self._sub = None

    def start(self) -> None:
        self._sub = self._node.create_subscription(
            Odometry, self._topic, self._on_msg, 10
        )

    def stop(self) -> None:
        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
            self._sub = None

    def get_odom_at(self, timestamp: float) -> Optional[OdomData]:
        if not self._buffer:
            return None
        return min(self._buffer, key=lambda o: abs(o.timestamp - timestamp))

    def _on_msg(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._buffer.append(OdomData(
            timestamp=stamp,
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=_quaternion_to_yaw(msg.pose.pose.orientation),
        ))


class ROS2GnssSource(GnssSourceBase):
    """ROS2 NavSatFix トピックから GnssData を供給するアダプター。Phase 4 で使用する。

    Phase 4 で UTM 変換を行うまでの仮実装として longitude/latitude を x/y に格納する。
    """

    def __init__(self, node: Node, topic: str = '/gps/fix') -> None:
        self._node = node
        self._topic = topic
        self._buffer: deque[GnssData] = deque(maxlen=_GNSS_BUFFER_SIZE)
        self._sub = None

    def start(self) -> None:
        self._sub = self._node.create_subscription(
            NavSatFix, self._topic, self._on_msg, 10
        )

    def stop(self) -> None:
        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
            self._sub = None

    def get_gnss_at(self, timestamp: float) -> Optional[GnssData]:
        if not self._buffer:
            return None
        return min(self._buffer, key=lambda g: abs(g.timestamp - timestamp))

    def get_all_gnss(self) -> list[GnssData]:
        return list(self._buffer)

    def _on_msg(self, msg: NavSatFix) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cov = np.array(
            msg.position_covariance[:4], dtype=np.float64).reshape(2, 2)
        self._buffer.append(GnssData(
            timestamp=stamp,
            x=msg.longitude,
            y=msg.latitude,
            covariance=cov,
        ))
