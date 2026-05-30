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

from slam_gnss_2d.config import SlamConfig
from slam_gnss_2d.data_types import ScanData
from slam_gnss_2d.gnss.constraint_inserter import GnssConstraintInserter
from slam_gnss_2d.gnss.kinematic_aligner import KinematicHeadingAligner
from slam_gnss_2d.input.base import OdomSourceBase, ScanSourceBase
from slam_gnss_2d.input.ros2.bag_reader import BagGnssSource, BagOdomSource, BagScanSource
from slam_gnss_2d.optimizer.gtsam_optimizer import GTSAMOptimizer
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
        self._use_gnss = cfg.use_gnss

        if cfg.use_gnss:
            gnss_source = BagGnssSource(bag_path, cfg.gnss_topic)
            gnss_source.start()
            self._gnss_source = gnss_source
            self._gnss_aligner = KinematicHeadingAligner(
                min_speed_ms=cfg.kinematic_min_speed_ms,
            )
            self._gnss_inserter = GnssConstraintInserter(
                default_noise_xy_m=cfg.gnss_noise_xy_m,
                max_time_delta_s=cfg.gnss_max_time_delta_s,
            )
            self._gnss_optimizer = GTSAMOptimizer()

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
            if self._use_gnss:
                self._run_gnss_phase()
            self.get_logger().info('Bag processing complete')

    def _run_gnss_phase(self) -> None:
        """GNSS 2 パス処理: Aligner → Inserter → 再最適化 → rerender。"""
        gnss_list = self._gnss_source.get_all_gnss()
        if not gnss_list:
            self.get_logger().warn('GNSS phase skipped: no valid GNSS fixes in bag')
            return

        nodes = self._pose_graph.get_nodes()
        edges = self._pose_graph.get_edges()
        if not nodes:
            self.get_logger().warn('GNSS phase skipped: pose graph is empty')
            return

        transform = self._gnss_aligner.estimate_transform(nodes, gnss_list)
        tx, ty, rot = transform
        self.get_logger().info(
            f'GNSS align: tx={tx:.2f}m ty={ty:.2f}m rot={math.degrees(rot):.2f}deg'
            f' ({len(gnss_list)} GNSS fixes, {len(nodes)} nodes)'
        )

        priors = self._gnss_inserter.build_priors(nodes, gnss_list, transform)
        self.get_logger().info(
            f'GNSS inserting {len(priors)} prior constraints')

        updated = self._gnss_optimizer.optimize(
            nodes, edges, gnss_priors=priors)
        self._renderer.rerender_all(updated)
        self._rebuild_path(updated)
        self._map_dirty = True
        self.get_logger().info('GNSS phase complete: map re-rendered with GNSS constraints')


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
