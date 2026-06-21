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

from rclpy.node import Node

from slam_gnss_2d.core.component_factory import build_pose_graph_builder, build_renderer
from slam_gnss_2d.core.config import SlamConfig
from slam_gnss_2d.core.data_types import PoseNode, ScanData, SensorFrame
from slam_gnss_2d.core.graph_orchestrator import GraphOrchestrator
from slam_gnss_2d.input.base import GnssSourceBase, OdomSourceBase, ScanSourceBase
from slam_gnss_2d.core.sensor_synchronizer import SensorSynchronizer
from slam_gnss_2d.core.config_loader import ConfigLoader
from slam_gnss_2d.ros.slam_visualizer import SlamVisualizer
from slam_gnss_2d.ros.tf_broadcaster import SlamTfBroadcaster
from slam_gnss_2d.ros.map_save_service import MapSaveService

class SlamNodeBase(Node, ABC):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self._declare_params()
        cfg = ConfigLoader.build_config(self)

        self._scan_source, self._odom_source = self._setup_io(cfg)

        self._pose_graph = build_pose_graph_builder(cfg)
        self._renderer = build_renderer(cfg)

        self._enable_scan_matching = cfg.scan_matching.enabled
        self._enable_gnss = cfg.gnss.enabled
        self._enable_loop_closure = cfg.loop_closure.enabled

        self._use_gnss = cfg.gnss.enabled
        self._gnss_source = None
        if self._use_gnss:
            self._gnss_source = self._setup_gnss_source(cfg)

        self._synchronizer = SensorSynchronizer(
            scan_source=self._scan_source,
            odom_source=self._odom_source,
            gnss_source=self._gnss_source,
            logger=self.get_logger()
        )
        self._synchronizer.set_frame_callback(self._on_frame)
        self._synchronizer.start()

        self._orchestrator = GraphOrchestrator(
            logger=self.get_logger(),
            pose_graph=self._pose_graph,
            use_gnss=self._use_gnss,
            isam2_relinearize_threshold=cfg.optimization.isam2.relinearize_threshold,
            anchor_min_fix_status=cfg.gnss.anchor.min_fix_status,
            gnss_fix_sigma_m=cfg.gnss.sigma.fix_m,
            gnss_float_sigma_m=cfg.gnss.sigma.float_m,
            gnss_factor_yaw_variance=cfg.gnss.sigma.factor_yaw_variance,
            gnss_init_distance_m=cfg.gnss.anchor.init_distance_m,
        )

        self._visualizer = SlamVisualizer(self, self._use_gnss)
        self._tf_broadcaster = SlamTfBroadcaster(self)
        self._save_service = MapSaveService(self, self._pose_graph, self._orchestrator)

        self._map_dirty = False
        self.create_timer(1.0 / cfg.map.publish_hz, self._publish_map_timer)
        self.create_timer(0.1, self._publish_tf_timer)

        self._node_count = 0
        self._last_stat_time = time.monotonic()
        self._finalized = False

        self.get_logger().info(
            f'{node_name} started (scan_matching={cfg.scan_matching.enabled}, loop_closure={cfg.loop_closure.enabled}, '
            f'matcher={cfg.scan_matching.type}, ref={cfg.scan_matching.reference})\n'
            f'  scan: {cfg.topics.scan}, odom: {cfg.topics.odom}\n'
            f'  map: dynamic @ {cfg.map.resolution}m/px, margin={cfg.map.expansion_margin}m\n'
        )

    def _declare_params(self) -> None:
        ConfigLoader.declare_params(self)

    @abstractmethod
    def _setup_io(self, cfg: SlamConfig) -> tuple[ScanSourceBase, OdomSourceBase]:
        raise NotImplementedError

    def _setup_gnss_source(self, cfg: SlamConfig) -> GnssSourceBase:
        raise NotImplementedError

    def _on_frame(self, frame: SensorFrame) -> None:
        result = self._orchestrator.process_frame(frame)
        node = result.node
        if node is None:
            self.get_logger().debug(
                f'Scan rejected: '
                f'below threshold at ({frame.odom.x:.2f}, {frame.odom.y:.2f})'
            )
            return

        self._tf_broadcaster.update(node, frame.odom)
        self._node_count += 1
        if self._node_count == 1 or self._node_count % 10 == 0:
            self.get_logger().debug(
                f'Node #{node.index}: x={node.x:.2f} y={node.y:.2f} '
                f'yaw={math.degrees(node.yaw):.1f}deg'
            )

        if result.loop_closed or result.rerender_required:
            if result.loop_closed:
                self._visualizer.publish_path_before_optimize()
            nodes = self._pose_graph.get_nodes()
            self._renderer.rerender_all(nodes)
            self._visualizer.rebuild_path(nodes)
            if result.loop_closed:
                self.get_logger().info(
                    f'Loop closed at node #{node.index}: full rerender triggered'
                )
        else:
            if not self._renderer.add_node(node):
                self._renderer.rerender_all(self._pose_graph.get_nodes())
            self._visualizer.publish_path_increment(node)
            
        self._visualizer.publish_pose_graph_markers(self._pose_graph)
        self._map_dirty = True

    def _publish_map_timer(self) -> None:
        self._visualizer.publish_anchor(self._orchestrator)

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
            stats = self._synchronizer.get_stats()
            self.get_logger().debug(
                f'[stat] nodes={self._node_count}, '
                f'scans={stats["scans"]}, '
                f'odom_miss={stats["odom_miss"]}'
                + icp_stat + loop_stat
            )
            self._last_stat_time = now

        if not self._map_dirty:
            return
        self._map_dirty = False

        self._visualizer.publish_map(self._renderer)

    def _publish_tf_timer(self) -> None:
        self._tf_broadcaster.publish()

    def finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        self.get_logger().info('Finalizing SLAM node...')

        finalize_result = self._orchestrator.finalize()

        if finalize_result.rerender_required:
            nodes = self._pose_graph.get_nodes()
            self._renderer.rerender_all(nodes)
            self._visualizer.rebuild_path(nodes)
            self._map_dirty = True
            self._publish_map_timer()
            self.get_logger().info('Final map optimization and rendering complete.')

    def destroy_node(self) -> None:
        self._synchronizer.stop()
        super().destroy_node()
