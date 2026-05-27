#!/usr/bin/env python3
"""slam_gnss_2d オフライン処理エントリポイント。

rosbag2 をリアルタイム再生せず、add_scan() 完了後に次スキャンを読み込む
ステップ駆動方式で SLAM を実行する。RViz2 可視化はオンラインと同一トピックを利用する。
"""
from __future__ import annotations

import logging
import math

import rclpy
from nav_msgs.msg import Odometry

from slam_gnss_2d.config import SlamConfig
from slam_gnss_2d.data_types import ScanData
from slam_gnss_2d.input.base import OdomSourceBase, ScanSourceBase
from slam_gnss_2d.input.ros2.bag_reader import BagOdomSource, BagScanSource
from slam_gnss_2d.slam_node_base import SlamNodeBase


class SlamOfflineNode(SlamNodeBase):
    def __init__(self) -> None:
        super().__init__('slam_gnss_2d_offline_node')
        step_hz: float = self.get_parameter('offline_step_hz').value
        period = 0.001 if step_hz <= 0.0 else 1.0 / step_hz
        self._step_timer = self.create_timer(period, self._process_step)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)

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

    def _process_step(self) -> None:
        if not self._bag_scan_source.step():
            self._step_timer.cancel()
            self.get_logger().info('Bag processing complete')


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
