from __future__ import annotations

import bisect
import math
from collections import deque
from typing import Callable, Optional

import numpy as np
import rclpy
import tf2_ros
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


def _angle_diff(a: float, b: float) -> float:
    """角度差 a - b を [-pi, pi] に正規化して返す。"""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ROS2ScanSource(ScanSourceBase):
    """ROS2 LaserScan トピックから ScanData を供給するアダプター。"""

    def __init__(self, node: Node, topic: str = '/scan') -> None:
        self._node = node
        self._topic = topic
        self._callback: Optional[Callable[[ScanData], None]] = None
        self._sub = None
        self._recv_count = 0
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)
        self._lidar_yaw: float = 0.0
        self._lidar_tf_ready: bool = False

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

        if not self._lidar_tf_ready:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'base_link', msg.header.frame_id, rclpy.time.Time()
                )
                self._lidar_yaw = _quaternion_to_yaw(tf.transform.rotation)
                self._lidar_tf_ready = True
                self._node.get_logger().info(
                    f'ScanSource: TF resolved '
                    f'[{msg.header.frame_id} -> base_link]: '
                    f'yaw={math.degrees(self._lidar_yaw):.1f}deg'
                )
            except tf2_ros.TransformException as e:
                self._node.get_logger().warn(
                    f'ScanSource: TF not yet available, skipping scan ({e})'
                )
                return

        self._recv_count += 1
        if self._recv_count == 1 or self._recv_count % 100 == 0:
            self._node.get_logger().info(
                f'ScanSource [{self._topic}]: #{self._recv_count}, '
                f'{len(msg.ranges)} ranges, '
                f'range=[{msg.range_min:.2f}, {msg.range_max:.2f}]m'
            )
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._callback(ScanData(
            timestamp=stamp,
            ranges=np.array(msg.ranges, dtype=np.float32),
            angle_min=msg.angle_min + self._lidar_yaw,
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
        self._recv_count = 0
        self._empty_warned = False

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
            if not self._empty_warned:
                self._node.get_logger().warn(
                    f'OdomSource [{self._topic}]: buffer is empty'
                )
                self._empty_warned = True
            return None
        buf = sorted(self._buffer, key=lambda o: o.timestamp)
        timestamps = [o.timestamp for o in buf]
        idx = bisect.bisect_left(timestamps, timestamp)
        if idx == 0 or idx >= len(buf):
            best = buf[0] if idx == 0 else buf[-1]
            dt = abs(best.timestamp - timestamp)
            if dt > 0.5:
                self._node.get_logger().warn(
                    f'OdomSource: large time delta {dt:.3f}s '
                    f'(scan={timestamp:.3f}, odom={best.timestamp:.3f})'
                )
            return best
        prev = buf[idx - 1]
        next_ = buf[idx]
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

    def _on_msg(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        yaw = _quaternion_to_yaw(msg.pose.pose.orientation)
        self._recv_count += 1
        if self._recv_count == 1 or self._recv_count % 100 == 0:
            self._node.get_logger().info(
                f'OdomSource [{self._topic}]: #{self._recv_count}, '
                f'x={msg.pose.pose.position.x:.2f} '
                f'y={msg.pose.pose.position.y:.2f} '
                f'yaw={math.degrees(yaw):.1f}deg'
            )
        self._buffer.append(OdomData(
            timestamp=stamp,
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=yaw,
        ))





class ROS2GnssUtmSource(GnssSourceBase):
    """ROS2 NavSatFix を UTM に変換して GnssData を供給するアダプター。"""

    def __init__(self, node: Node, topic: str = '/gps/fix') -> None:
        self._node = node
        self._topic = topic
        self._buffer: deque[GnssData] = deque(maxlen=_GNSS_BUFFER_SIZE)
        self._sub = None
        self._transformer = None
        self._recv_count = 0

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
        if msg.status.status < 0:
            return

        if self._transformer is None:
            from pyproj import CRS, Transformer

            zone = int((msg.longitude + 180.0) / 6.0) + 1
            south = msg.latitude < 0.0
            crs_utm = CRS.from_dict(
                {'proj': 'utm', 'zone': zone, 'south': south})
            self._transformer = Transformer.from_crs(
                'EPSG:4326', crs_utm, always_xy=True)
            self._node.get_logger().info(
                f'GNSS UTM transformer initialized: zone={zone} south={south}'
            )

        x, y = self._transformer.transform(msg.longitude, msg.latitude)
        cov = msg.position_covariance
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            cov_2x2 = np.zeros((2, 2), dtype=np.float64)
        else:
            cov_2x2 = np.array(
                [[cov[0], cov[1]], [cov[3], cov[4]]], dtype=np.float64)

        self._recv_count += 1
        if self._recv_count == 1 or self._recv_count % 100 == 0:
            self._node.get_logger().info(
                f'GnssUtmSource [{self._topic}]: #{self._recv_count}, '
                f'x={x:.2f} y={y:.2f} status={int(msg.status.status)}'
            )

        self._buffer.append(GnssData(
            timestamp=stamp,
            x=x,
            y=y,
            covariance=cov_2x2,
            fix_status=int(msg.status.status),
            latitude=msg.latitude,
            longitude=msg.longitude,
        ))


class ROS2NavpvtSource(GnssSourceBase):
    """ROS2 ublox_msgs/NavPVT を UTM に変換して GnssData を供給するアダプター。"""

    _FLAGS_GNSS_FIX_OK = 0x01
    _FIX_TYPE_2D = 2

    def __init__(self, node: Node, topic: str = '/navpvt', hacc_scale: float = 1.0) -> None:
        self._node = node
        self._topic = topic
        self._hacc_scale = hacc_scale
        self._buffer: deque[GnssData] = deque(maxlen=_GNSS_BUFFER_SIZE)
        self._sub = None
        self._transformer = None
        self._recv_count = 0

    def start(self) -> None:
        from ublox_msgs.msg import NavPVT
        self._sub = self._node.create_subscription(
            NavPVT, self._topic, self._on_msg, 10
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

    def _on_msg(self, msg) -> None:
        if not (msg.flags & self._FLAGS_GNSS_FIX_OK):
            return
        if msg.fix_type < self._FIX_TYPE_2D:
            return

        now = self._node.get_clock().now()
        stamp = now.nanoseconds * 1e-9

        lon = msg.lon * 1e-7
        lat = msg.lat * 1e-7

        if self._transformer is None:
            from pyproj import CRS, Transformer
            zone = int((lon + 180.0) / 6.0) + 1
            south = lat < 0.0
            crs_utm = CRS.from_dict({'proj': 'utm', 'zone': zone, 'south': south})
            self._transformer = Transformer.from_crs('EPSG:4326', crs_utm, always_xy=True)
            self._node.get_logger().info(
                f'GNSS UTM transformer initialized: zone={zone} south={south}'
            )

        x, y = self._transformer.transform(lon, lat)

        if msg.h_acc > 0:
            pos_std = (msg.h_acc / 1000.0) * self._hacc_scale
            pos_var = pos_std * pos_std
            cov_2x2 = np.array([[pos_var, 0.0], [0.0, pos_var]], dtype=np.float64)
        else:
            cov_2x2 = np.zeros((2, 2), dtype=np.float64)

        carr_soln = (msg.flags >> 6) & 0x03

        self._recv_count += 1
        if self._recv_count == 1 or self._recv_count % 100 == 0:
            self._node.get_logger().info(
                f'ROS2NavpvtSource [{self._topic}]: #{self._recv_count}, '
                f'x={x:.2f} y={y:.2f} status={carr_soln}'
            )

        self._buffer.append(GnssData(
            timestamp=stamp,
            x=x,
            y=y,
            covariance=cov_2x2,
            fix_status=carr_soln,
            latitude=lat,
            longitude=lon,
        ))
