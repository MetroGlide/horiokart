#!/usr/bin/env python3
"""slam_gnss_2d ノード基底クラス。

オンライン（ROS2）・オフライン（rosbag2）共通のコアロジックを実装する。
IO ソースの構築のみ派生クラスに委譲する。
"""
from __future__ import annotations

import array
import math
import time
from abc import ABC, abstractmethod

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from slam_gnss_2d.component_factory import build_pose_graph_builder
from slam_gnss_2d.config import SlamConfig
from slam_gnss_2d.data_types import ScanData
from slam_gnss_2d.input.base import OdomSourceBase, ScanSourceBase
from slam_gnss_2d.map_manager.opencv_renderer import OpenCVRenderer


class SlamNodeBase(Node, ABC):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self._declare_params()
        cfg = self._build_config()

        self._scan_source, self._odom_source = self._setup_io(cfg)

        self._pose_graph = build_pose_graph_builder(cfg)
        self._renderer = OpenCVRenderer(
            resolution=cfg.map_resolution,
            expansion_margin=cfg.map_expansion_margin,
        )

        self._map_pub = self.create_publisher(
            OccupancyGrid, 'slam_gnss_2d/map', 1)
        self._path_pub = self.create_publisher(Path, 'slam_gnss_2d/path', 1)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._path_msg = Path()
        self._path_msg.header.frame_id = 'map'
        self._map_dirty = False
        self._map_to_odom_x = 0.0
        self._map_to_odom_y = 0.0
        self._map_to_odom_yaw = 0.0

        self._scan_source.set_scan_callback(self._on_scan)
        self._odom_source.start()
        self._scan_source.start()

        self.create_timer(1.0 / cfg.map_publish_hz, self._publish_map_timer)
        self.create_timer(0.1, self._publish_tf)

        self._scan_recv_count = 0
        self._odom_miss_count = 0
        self._node_count = 0
        self._last_stat_time = time.monotonic()

        self.get_logger().info(
            f'{node_name} started (builder={cfg.pose_graph_builder}, '
            f'matcher={cfg.scan_matcher_type}, ref={cfg.scan_reference})\n'
            f'  scan: {cfg.scan_topic}, odom: {cfg.odom_topic}\n'
            f'  map: dynamic @ {cfg.map_resolution}m/px, margin={cfg.map_expansion_margin}m'
        )

    @abstractmethod
    def _setup_io(self, cfg: SlamConfig) -> tuple[ScanSourceBase, OdomSourceBase]:
        """IO ソース（スキャン・オドメトリ）を構築して返す。start() は呼ばない。"""
        raise NotImplementedError

    def _declare_params(self) -> None:
        # 購読トピック名
        self.declare_parameter('scan_topic', '/scan_top_lidar')
        self.declare_parameter('odom_topic', '/odom')

        # 占有格子マップ設定
        self.declare_parameter('map_resolution', 0.05)         # [m/px]
        self.declare_parameter('map_expansion_margin', 100.0)  # [m]

        # キーフレーム採択閾値
        self.declare_parameter('min_translation', 0.3)  # [m]
        self.declare_parameter('min_rotation', 0.1)     # [rad] ≈ 5.7°

        # 配信
        self.declare_parameter('map_publish_hz', 1.0)  # [Hz]

        # ポーズグラフ構築アルゴリズム
        # "odom_only" | "scan_matching" | "loop_closure"
        self.declare_parameter('pose_graph_builder', 'scan_matching')
        self.declare_parameter('scan_matcher_type',
                               'icp')             # "icp" | "ndt"
        # "scan_to_scan" | "scan_to_local_map"
        self.declare_parameter('scan_reference', 'scan_to_scan')

        # ICP パラメータ（scan_matcher_type == "icp" のとき使用）
        self.declare_parameter('icp_max_iterations', 30)
        # 更新ノルムがこの値未満で収束 [m]
        self.declare_parameter('icp_tolerance', 1e-4)
        self.declare_parameter(
            'icp_max_correspondence_dist', 0.5)  # [m] 大きいと誤対応リスク増

        # NDT パラメータ（scan_matcher_type == "ndt" のとき使用）
        self.declare_parameter('ndt_cell_size', 1.0)  # [m] 大きいほど粗く高速

        # ローカルマップパラメータ（scan_reference == "scan_to_local_map" のとき使用）
        # スライディングウィンドウ幅 [ノード数]
        self.declare_parameter('local_map_window', 20)
        self.declare_parameter('local_map_radius', 15.0)  # 参照点群の抽出半径 [m]

        # 連続失敗上限（この回数連続失敗で odom フォールバック）
        self.declare_parameter('matcher_max_failure_streak', 5)

        # ループクロージャパラメータ（pose_graph_builder == "loop_closure" のとき使用）
        self.declare_parameter('loop_closure_search_radius', 2.0)   # [m]
        self.declare_parameter('loop_closure_min_node_gap', 50)
        self.declare_parameter('loop_closure_max_failure_streak', 3)
        self.declare_parameter('optimize_every_n_loops', 1)

    def _build_config(self) -> SlamConfig:
        return SlamConfig(
            scan_topic=self.get_parameter('scan_topic').value,
            odom_topic=self.get_parameter('odom_topic').value,
            map_resolution=self.get_parameter('map_resolution').value,
            map_expansion_margin=self.get_parameter(
                'map_expansion_margin').value,
            min_translation=self.get_parameter('min_translation').value,
            min_rotation=self.get_parameter('min_rotation').value,
            map_publish_hz=self.get_parameter('map_publish_hz').value,
            pose_graph_builder=self.get_parameter('pose_graph_builder').value,
            scan_matcher_type=self.get_parameter('scan_matcher_type').value,
            scan_reference=self.get_parameter('scan_reference').value,
            icp_max_iterations=self.get_parameter('icp_max_iterations').value,
            icp_tolerance=self.get_parameter('icp_tolerance').value,
            icp_max_correspondence_dist=self.get_parameter(
                'icp_max_correspondence_dist').value,
            ndt_cell_size=self.get_parameter('ndt_cell_size').value,
            local_map_window=self.get_parameter('local_map_window').value,
            local_map_radius=self.get_parameter('local_map_radius').value,
            matcher_max_failure_streak=self.get_parameter(
                'matcher_max_failure_streak').value,
            loop_closure_search_radius=self.get_parameter(
                'loop_closure_search_radius').value,
            loop_closure_min_node_gap=self.get_parameter(
                'loop_closure_min_node_gap').value,
            loop_closure_max_failure_streak=self.get_parameter(
                'loop_closure_max_failure_streak').value,
            optimize_every_n_loops=self.get_parameter(
                'optimize_every_n_loops').value,
        )

    def _on_scan(self, scan: ScanData) -> None:
        self._scan_recv_count += 1
        odom = self._odom_source.get_odom_at(scan.timestamp)
        if odom is None:
            self._odom_miss_count += 1
            self.get_logger().warn(
                f'No odom for scan ts={scan.timestamp:.3f} (miss #{self._odom_miss_count})'
            )
            return

        node = self._pose_graph.add_scan(scan, odom)
        if node is None:
            self.get_logger().debug(
                f'Scan #{self._scan_recv_count} rejected: '
                f'below threshold at ({odom.x:.2f}, {odom.y:.2f})'
            )
            return

        self._update_map_to_odom(node, odom)
        self._node_count += 1
        if self._node_count == 1 or self._node_count % 10 == 0:
            self.get_logger().info(
                f'Node #{node.index}: x={node.x:.2f} y={node.y:.2f} '
                f'yaw={math.degrees(node.yaw):.1f}deg'
            )

        if self._pose_graph.loop_just_closed:
            nodes = self._pose_graph.get_nodes()
            self._renderer.rerender_all(nodes)
            self._rebuild_path(nodes)
            self.get_logger().info(
                f'Loop closed at node #{node.index}: full rerender triggered'
            )
        else:
            if not self._renderer.add_node(node):
                self._renderer.rerender_all(self._pose_graph.get_nodes())
            self._publish_path(node)
        self._map_dirty = True

    def _publish_map_timer(self) -> None:
        now = time.monotonic()
        if now - self._last_stat_time >= 30.0:
            pg = self._pose_graph
            icp_stat = ''
            if hasattr(pg, 'icp_attempt_count') and pg.icp_attempt_count > 0:
                rate = pg.icp_success_count / pg.icp_attempt_count * 100
                icp_stat = (
                    f', icp={rate:.0f}%'
                    f'({pg.icp_success_count}/{pg.icp_attempt_count})'
                    f' fb={pg.odom_fallback_count}'
                )
            loop_stat = ''
            if hasattr(pg, 'loop_attempt_count') and pg.loop_attempt_count > 0:
                rate = pg.loop_success_count / pg.loop_attempt_count * 100
                loop_stat = (
                    f', loop={rate:.0f}%'
                    f'({pg.loop_success_count}/{pg.loop_attempt_count})'
                )
            self.get_logger().info(
                f'[stat] nodes={self._node_count}, '
                f'scans={self._scan_recv_count}, '
                f'odom_miss={self._odom_miss_count}'
                + icp_stat + loop_stat
            )
            self._last_stat_time = now

        if not self._map_dirty:
            return
        self._map_dirty = False

        data, origin_x, origin_y, resolution = self._renderer.to_occupancy_array()
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = resolution
        msg.info.width = int(data.shape[1])
        msg.info.height = int(data.shape[0])
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.data = array.array('b', data.ravel().tobytes())
        self._map_pub.publish(msg)
        self.get_logger().debug(
            f'Map published: occupied={int((data == 100).sum())}, '
            f'free={int((data == 0).sum())} px'
        )

    def _update_map_to_odom(self, node, odom) -> None:
        """最新キーフレームから map->odom 変換を更新する。

        map_T_odom = map_T_base * inv(odom_T_base)
        """
        c_o = math.cos(odom.yaw)
        s_o = math.sin(odom.yaw)
        inv_x = -(c_o * odom.x + s_o * odom.y)
        inv_y = -(-s_o * odom.x + c_o * odom.y)
        c_m = math.cos(node.yaw)
        s_m = math.sin(node.yaw)
        self._map_to_odom_x = node.x + c_m * inv_x - s_m * inv_y
        self._map_to_odom_y = node.y + s_m * inv_x + c_m * inv_y
        self._map_to_odom_yaw = node.yaw - odom.yaw

    def _publish_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self._map_to_odom_x
        t.transform.translation.y = self._map_to_odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = math.cos(self._map_to_odom_yaw / 2.0)
        t.transform.rotation.z = math.sin(self._map_to_odom_yaw / 2.0)
        self._tf_broadcaster.sendTransform(t)

    def _publish_path(self, node) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = node.x
        pose.pose.position.y = node.y
        pose.pose.orientation.w = math.cos(node.yaw / 2.0)
        pose.pose.orientation.z = math.sin(node.yaw / 2.0)
        self._path_msg.header.stamp = pose.header.stamp
        self._path_msg.poses.append(pose)
        self._path_pub.publish(self._path_msg)

    def _rebuild_path(self, nodes: list) -> None:
        """全ノードからパスを再構築する。ループ閉合最適化後に呼び出す。"""
        now = self.get_clock().now().to_msg()
        self._path_msg.header.stamp = now
        self._path_msg.poses.clear()
        for n in nodes:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'map'
            pose.pose.position.x = n.x
            pose.pose.position.y = n.y
            pose.pose.orientation.w = math.cos(n.yaw / 2.0)
            pose.pose.orientation.z = math.sin(n.yaw / 2.0)
            self._path_msg.poses.append(pose)
        self._path_pub.publish(self._path_msg)

    def destroy_node(self) -> None:
        self._scan_source.stop()
        self._odom_source.stop()
        super().destroy_node()
