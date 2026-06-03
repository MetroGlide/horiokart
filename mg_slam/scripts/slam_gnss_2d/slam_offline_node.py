#!/usr/bin/env python3
"""slam_gnss_2d オフライン処理エントリポイント。

rosbag2 をリアルタイム再生せず、add_scan() 完了後に次スキャンを読み込む
ステップ駆動方式で SLAM を実行する。RViz2 可視化はオンラインと同一トピックを利用する。

bag 読み込み完了後、use_gnss=True の場合は GNSS 2 パス処理を実行する:
  Step 1: bag 再生でポーズグラフを構築（Phase 1〜3 相当）
  Step 2: GNSS 拘束を挿入して再最適化し、マップを地球座標系に整合させる
"""
from __future__ import annotations

import logging
import math

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix as NavSatFixMsg, NavSatStatus

from slam_gnss_2d.config import SlamConfig
from slam_gnss_2d.component_factory import build_gnss_source
from slam_gnss_2d.data_types import ScanData
from slam_gnss_2d.input.base import GnssSourceBase, OdomSourceBase, ScanSourceBase
from slam_gnss_2d.input.ros2.bag_reader import BagOdomSource, BagScanSource
from slam_gnss_2d.slam_node_base import SlamNodeBase


class SlamOfflineNode(SlamNodeBase):
    def __init__(self) -> None:
        super().__init__('slam_gnss_2d_offline_node')
        step_hz: float = self.get_parameter('offline_step_hz').value
        period = 0.001 if step_hz <= 0.0 else 1.0 / step_hz
        self._step_timer = self.create_timer(period, self._process_step)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._gps_fix_pub = self.create_publisher(NavSatFixMsg, '/gps/fix', 10)
        self._last_gps_ts: float = -1.0

    def _declare_params(self) -> None:
        super()._declare_params()
        self.declare_parameter('bag_path', '')
        # 可視化速度 [Hz]。0 以下で最大速度（1ms タイマー）
        self.declare_parameter('offline_step_hz', 30.0)

    def _setup_io(self, cfg: SlamConfig) -> tuple[ScanSourceBase, OdomSourceBase]:
        bag_path: str = self.get_parameter('bag_path').value
        scan_source = BagScanSource(bag_path, cfg.scan_topic)
        odom_source = BagOdomSource(bag_path, cfg.odom_topic)
        self._bag_scan_source = scan_source
        return scan_source, odom_source

    def _setup_gnss_source(self, cfg: SlamConfig) -> GnssSourceBase:
        bag_path: str = self.get_parameter('bag_path').value
        return build_gnss_source(cfg, bag_path)

    def _on_scan(self, scan: ScanData) -> None:
        super()._on_scan(scan)
        odom = self._odom_source.get_odom_at(scan.timestamp)
        if odom is None:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = odom.x
        msg.pose.pose.position.y = odom.y
        msg.pose.pose.orientation.w = math.cos(odom.yaw / 2.0)
        msg.pose.pose.orientation.z = math.sin(odom.yaw / 2.0)
        self._odom_pub.publish(msg)

        if self._use_gnss:
            self._republish_gps_fix(scan.timestamp)

    def _process_step(self) -> None:
        if not self._bag_scan_source.step():
            self._step_timer.cancel()
            if self._use_gnss and self._gnss_mode == 'batch':
                self._run_gnss_phase()
            self.get_logger().info('Bag processing complete')

    def _republish_gps_fix(self, timestamp: float) -> None:
        """bag 内の GPS fix をスキャン処理に同期して /gps/fix に再配信する。"""
        raw_get_fix = getattr(self._gnss_source, 'get_raw_fix_at', None)
        if raw_get_fix is None:
            return
        raw = raw_get_fix(timestamp)
        if raw is None:
            return
        ts, lat, lon, status, cov, cov_type = raw
        if ts == self._last_gps_ts:
            return
        self._last_gps_ts = ts
        fix_msg = NavSatFixMsg()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = 'gps'
        fix_msg.status.status = status
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = lat
        fix_msg.longitude = lon
        fix_msg.altitude = 0.0
        fix_msg.position_covariance = cov
        fix_msg.position_covariance_type = cov_type
        self._gps_fix_pub.publish(fix_msg)


def main(args=None):
    logging.basicConfig(level=logging.INFO,
                        format='%(name)s %(levelname)s: %(message)s')
    rclpy.init(args=args)
    node = SlamOfflineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
