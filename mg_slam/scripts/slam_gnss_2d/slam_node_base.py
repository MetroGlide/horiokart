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

from geometry_msgs.msg import Point as RosPoint, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from slam_gnss_2d.component_factory import build_pose_graph_builder
from slam_gnss_2d.config import SlamConfig
from slam_gnss_2d.data_types import PoseNode, ScanData
from slam_gnss_2d.graph_orchestrator import GraphOrchestrator
from slam_gnss_2d.input.base import GnssSourceBase, OdomSourceBase, ScanSourceBase
from slam_gnss_2d.map_manager.opencv_renderer import OpenCVRenderer


class SlamNodeBase(Node, ABC):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self._declare_params()
        cfg = self._build_config()

        self._scan_source, self._odom_source = self._setup_io(cfg)

        self._pose_graph = build_pose_graph_builder(cfg)
        self._renderer = OpenCVRenderer(
            resolution=cfg.map.resolution,
            expansion_margin=cfg.map.expansion_margin,
        )

        self._map_pub = self.create_publisher(
            OccupancyGrid, 'slam_gnss_2d/map', 1)
        self._path_pub = self.create_publisher(Path, 'slam_gnss_2d/path', 1)
        self._pg_marker_pub = self.create_publisher(
            MarkerArray, 'slam_gnss_2d/pose_graph', 1)
        self._path_before_pub = self.create_publisher(
            Path, 'slam_gnss_2d/path_before_optimize', 1)
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

        self._enable_scan_matching = cfg.scan_matching.enabled
        self._enable_gnss = cfg.gnss.enabled
        self._enable_loop_closure = cfg.loop_closure.enabled
        self._enable_incremental_optimizer = cfg.optimization.incremental

        self._gnss_missing_grace_frames = cfg.gnss.validation.missing_grace_frames
        self._gnss_missing_streak = 0
        self._gnss_degraded = False

        self._use_gnss = cfg.gnss.enabled
        self._gnss_source = None
        self._gnss_runner = None
        if self._use_gnss:
            self._gnss_source = self._setup_gnss_source(cfg)
            self._gnss_source.start()
            if self._enable_incremental_optimizer:
                from slam_gnss_2d.gnss.gnss_anchored_runner import GnssAnchoredParams, GnssAnchoredRunner

                params = GnssAnchoredParams(
                    init_distance_m=cfg.gnss.anchor.init_distance_m,
                    anchor_min_fix_status=cfg.gnss.anchor.min_fix_status,
                    anchor_sigma_m=cfg.gnss.anchor.sigma_m,
                    init_yaw_sigma_rad=cfg.gnss.anchor.init_yaw_sigma_rad,
                    gnss_fix_sigma_m=cfg.gnss.sigma.fix_m,
                    gnss_float_sigma_m=cfg.gnss.sigma.float_m,
                    gnss_factor_yaw_variance=cfg.gnss.sigma.factor_yaw_variance,
                    gnss_max_sigma_m=cfg.gnss.validation.max_sigma_m,
                    gnss_rerender_threshold_m=cfg.optimization.rerender_threshold_m,
                )
                if cfg.optimization.backend == 'gtsam':
                    from slam_gnss_2d.optimizer.gtsam_incremental_adapter import GTSAMIncrementalAdapter
                    opt = GTSAMIncrementalAdapter()
                else:
                    from slam_gnss_2d.optimizer.isam2_optimizer import ISAM2Optimizer
                    opt = ISAM2Optimizer(
                        relinearize_threshold=cfg.optimization.isam2.relinearize_threshold)

                self._gnss_runner = GnssAnchoredRunner(
                    params=params,
                    optimizer=opt,
                )

        self._orchestrator = GraphOrchestrator(
            logger=self.get_logger(),
            pose_graph=self._pose_graph,
            use_gnss=self._use_gnss,
            enable_incremental_optimizer=self._enable_incremental_optimizer,
            gnss_missing_grace_frames=self._gnss_missing_grace_frames,
            gnss_source=self._gnss_source,
            gnss_runner=self._gnss_runner,
        )

        self.create_timer(1.0 / cfg.map.publish_hz, self._publish_map_timer)
        self.create_timer(0.1, self._publish_tf)

        self._scan_recv_count = 0
        self._odom_miss_count = 0
        self._node_count = 0
        self._last_stat_time = time.monotonic()
        self._finalized = False

        self.get_logger().info(
            f'{node_name} started (scan_matching={cfg.scan_matching.enabled}, loop_closure={cfg.loop_closure.enabled}, '
            f'matcher={cfg.scan_matching.type}, ref={cfg.scan_matching.reference})\n'
            f'  scan: {cfg.topics.scan}, odom: {cfg.topics.odom}\n'
            f'  map: dynamic @ {cfg.map.resolution}m/px, margin={cfg.map.expansion_margin}m\n'
            f'  incremental={self._enable_incremental_optimizer}'
        )

    @abstractmethod
    def _setup_io(self, cfg: SlamConfig) -> tuple[ScanSourceBase, OdomSourceBase]:
        """IO ソース（スキャン・オドメトリ）を構築して返す。start() は呼ばない。"""
        raise NotImplementedError

    def _setup_gnss_source(self, cfg: SlamConfig) -> GnssSourceBase:
        """GNSS ソースを構築して返す。use_gnss=True の場合のみ呼び出される。"""
        raise NotImplementedError

    def _declare_params(self) -> None:
        self.declare_parameter('topics.scan', '/scan_top_lidar')
        self.declare_parameter('topics.odom', '/odom')

        self.declare_parameter('map.resolution', 0.05)
        self.declare_parameter('map.expansion_margin', 100.0)
        self.declare_parameter('map.publish_hz', 1.0)

        self.declare_parameter('keyframe.min_translation', 1.0)
        self.declare_parameter('keyframe.min_rotation', 0.1)

        self.declare_parameter('scan_matching.enabled', True)
        self.declare_parameter('scan_matching.type', 'ndt')
        self.declare_parameter('scan_matching.reference', 'scan_to_local_map')
        self.declare_parameter('scan_matching.max_failure_streak', 5)
        self.declare_parameter('scan_matching.icp.max_iterations', 100)
        self.declare_parameter('scan_matching.icp.tolerance', 1e-5)
        self.declare_parameter('scan_matching.icp.max_correspondence_dist', 1.0)
        self.declare_parameter('scan_matching.ndt.cell_size', 1.0)
        self.declare_parameter('scan_matching.local_map.window', 30)
        self.declare_parameter('scan_matching.local_map.radius', 30.0)

        self.declare_parameter('loop_closure.enabled', True)
        self.declare_parameter('loop_closure.search_radius', 2.0)
        self.declare_parameter('loop_closure.min_node_gap', 50)
        self.declare_parameter('loop_closure.max_failure_streak', 3)
        self.declare_parameter('loop_closure.matcher_type', 'icp')
        self.declare_parameter('loop_closure.icp.max_iterations', 100)
        self.declare_parameter('loop_closure.icp.tolerance', 1e-5)
        self.declare_parameter('loop_closure.icp.max_correspondence_dist', 1.0)
        self.declare_parameter('loop_closure.ndt.cell_size', 1.0)
        self.declare_parameter('loop_closure.max_dyaw_deg', 145.0)
        self.declare_parameter('loop_closure.crossing_reject_deg', 45.0)
        self.declare_parameter('loop_closure.submap_radius', 5.0)
        self.declare_parameter('loop_closure.max_score', 0.0)

        self.declare_parameter('gnss.enabled', True)
        self.declare_parameter('gnss.source', 'navpvt')
        self.declare_parameter('gnss.topics.fix', '/gps/fix')
        self.declare_parameter('gnss.topics.navpvt', '/navpvt')
        self.declare_parameter('gnss.navpvt_hacc_scale', 1.0)
        self.declare_parameter('gnss.validation.max_sigma_m', 5.0)
        self.declare_parameter('gnss.validation.missing_grace_frames', 30)
        self.declare_parameter('gnss.anchor.min_fix_status', 0)
        self.declare_parameter('gnss.anchor.sigma_m', 0.05)
        self.declare_parameter('gnss.anchor.init_yaw_sigma_rad', 10.0)
        self.declare_parameter('gnss.anchor.init_distance_m', 2.0)
        self.declare_parameter('gnss.sigma.fix_m', 0.02)
        self.declare_parameter('gnss.sigma.float_m', 0.5)
        self.declare_parameter('gnss.sigma.factor_yaw_variance', 1e8)

        self.declare_parameter('optimization.backend', 'gtsam')
        self.declare_parameter('optimization.incremental', True)
        self.declare_parameter('optimization.optimize_every_n_loops', 3)
        self.declare_parameter('optimization.rerender_threshold_m', 0.1)
        self.declare_parameter('optimization.isam2.relinearize_threshold', 0.1)

    def _build_config(self) -> SlamConfig:
        from slam_gnss_2d.config import (
            TopicsConfig, MapConfig, KeyframeConfig, ScanMatchingConfig, IcpConfig, NdtConfig, LocalMapConfig,
            LoopClosureConfig, GnssConfig, GnssTopicsConfig, GnssValidationConfig, GnssAnchorConfig,
            GnssSigmaConfig, OptimizationConfig, Isam2Config
        )

        return SlamConfig(
            topics=TopicsConfig(
                scan=self.get_parameter('topics.scan').value,
                odom=self.get_parameter('topics.odom').value,
            ),
            map=MapConfig(
                resolution=self.get_parameter('map.resolution').value,
                expansion_margin=self.get_parameter('map.expansion_margin').value,
                publish_hz=self.get_parameter('map.publish_hz').value,
            ),
            keyframe=KeyframeConfig(
                min_translation=self.get_parameter('keyframe.min_translation').value,
                min_rotation=self.get_parameter('keyframe.min_rotation').value,
            ),
            scan_matching=ScanMatchingConfig(
                enabled=self.get_parameter('scan_matching.enabled').value,
                type=self.get_parameter('scan_matching.type').value,
                reference=self.get_parameter('scan_matching.reference').value,
                max_failure_streak=self.get_parameter('scan_matching.max_failure_streak').value,
                icp=IcpConfig(
                    max_iterations=self.get_parameter('scan_matching.icp.max_iterations').value,
                    tolerance=self.get_parameter('scan_matching.icp.tolerance').value,
                    max_correspondence_dist=self.get_parameter('scan_matching.icp.max_correspondence_dist').value,
                ),
                ndt=NdtConfig(
                    cell_size=self.get_parameter('scan_matching.ndt.cell_size').value,
                ),
                local_map=LocalMapConfig(
                    window=self.get_parameter('scan_matching.local_map.window').value,
                    radius=self.get_parameter('scan_matching.local_map.radius').value,
                ),
            ),
            loop_closure=LoopClosureConfig(
                enabled=self.get_parameter('loop_closure.enabled').value,
                search_radius=self.get_parameter('loop_closure.search_radius').value,
                min_node_gap=self.get_parameter('loop_closure.min_node_gap').value,
                max_failure_streak=self.get_parameter('loop_closure.max_failure_streak').value,
                matcher_type=self.get_parameter('loop_closure.matcher_type').value,
                icp=IcpConfig(
                    max_iterations=self.get_parameter('loop_closure.icp.max_iterations').value,
                    tolerance=self.get_parameter('loop_closure.icp.tolerance').value,
                    max_correspondence_dist=self.get_parameter('loop_closure.icp.max_correspondence_dist').value,
                ),
                ndt=NdtConfig(
                    cell_size=self.get_parameter('loop_closure.ndt.cell_size').value,
                ),
                max_dyaw_deg=self.get_parameter('loop_closure.max_dyaw_deg').value,
                crossing_reject_deg=self.get_parameter('loop_closure.crossing_reject_deg').value,
                submap_radius=self.get_parameter('loop_closure.submap_radius').value,
                max_score=self.get_parameter('loop_closure.max_score').value,
            ),
            gnss=GnssConfig(
                enabled=self.get_parameter('gnss.enabled').value,
                source=self.get_parameter('gnss.source').value,
                topics=GnssTopicsConfig(
                    fix=self.get_parameter('gnss.topics.fix').value,
                    navpvt=self.get_parameter('gnss.topics.navpvt').value,
                ),
                navpvt_hacc_scale=self.get_parameter('gnss.navpvt_hacc_scale').value,
                validation=GnssValidationConfig(
                    max_sigma_m=self.get_parameter('gnss.validation.max_sigma_m').value,
                    missing_grace_frames=self.get_parameter('gnss.validation.missing_grace_frames').value,
                ),
                anchor=GnssAnchorConfig(
                    min_fix_status=self.get_parameter('gnss.anchor.min_fix_status').value,
                    sigma_m=self.get_parameter('gnss.anchor.sigma_m').value,
                    init_yaw_sigma_rad=self.get_parameter('gnss.anchor.init_yaw_sigma_rad').value,
                    init_distance_m=self.get_parameter('gnss.anchor.init_distance_m').value,
                ),
                sigma=GnssSigmaConfig(
                    fix_m=self.get_parameter('gnss.sigma.fix_m').value,
                    float_m=self.get_parameter('gnss.sigma.float_m').value,
                    factor_yaw_variance=self.get_parameter('gnss.sigma.factor_yaw_variance').value,
                ),
            ),
            optimization=OptimizationConfig(
                backend=self.get_parameter('optimization.backend').value,
                incremental=self.get_parameter('optimization.incremental').value,
                optimize_every_n_loops=self.get_parameter('optimization.optimize_every_n_loops').value,
                rerender_threshold_m=self.get_parameter('optimization.rerender_threshold_m').value,
                isam2=Isam2Config(
                    relinearize_threshold=self.get_parameter('optimization.isam2.relinearize_threshold').value,
                )
            ),
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

        result = self._orchestrator.process_scan(scan, odom)
        node = result.node
        if node is None:
            self.get_logger().debug(
                f'Scan #{self._scan_recv_count} rejected: '
                f'below threshold at ({odom.x:.2f}, {odom.y:.2f})'
            )
            return

        self._update_map_to_odom(node, odom)
        self._node_count += 1
        if self._node_count == 1 or self._node_count % 10 == 0:
            self.get_logger().debug(
                f'Node #{node.index}: x={node.x:.2f} y={node.y:.2f} '
                f'yaw={math.degrees(node.yaw):.1f}deg'
            )

        if result.loop_closed or result.rerender_required:
            if result.loop_closed:
                self._path_before_pub.publish(self._path_msg)
            nodes = self._pose_graph.get_nodes()
            self._renderer.rerender_all(nodes)
            self._rebuild_path(nodes)
            if result.loop_closed:
                self.get_logger().info(
                    f'Loop closed at node #{node.index}: full rerender triggered'
                )
        else:
            if not self._renderer.add_node(node):
                self._renderer.rerender_all(self._pose_graph.get_nodes())
            self._publish_path(node)
        self._publish_pose_graph_markers()
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
            self.get_logger().debug(
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

    def _publish_pose_graph_markers(self) -> None:
        """ポーズグラフノード・エッジを MarkerArray として配信する。"""
        nodes = self._pose_graph.get_nodes()
        if not nodes:
            return
        all_edges = self._pose_graph.get_edges()
        loop_edge_set: set[tuple[int, int]] = set()
        if hasattr(self._pose_graph, 'get_loop_edges'):
            loop_edge_set = {
                (e.from_index, e.to_index)
                for e in self._pose_graph.get_loop_edges()
            }
        node_by_idx = {n.index: n for n in nodes}
        now = self.get_clock().now().to_msg()
        array = MarkerArray()

        node_m = Marker()
        node_m.header.stamp = now
        node_m.header.frame_id = 'map'
        node_m.ns = 'nodes'
        node_m.id = 0
        node_m.type = Marker.SPHERE_LIST
        node_m.action = Marker.ADD
        node_m.scale.x = node_m.scale.y = node_m.scale.z = 0.2
        node_m.color.r = node_m.color.g = node_m.color.b = node_m.color.a = 1.0
        latest_idx = nodes[-1].index
        for n in nodes:
            pt = RosPoint()
            pt.x, pt.y, pt.z = n.x, n.y, 0.0
            node_m.points.append(pt)
            c = ColorRGBA()
            if n.index == latest_idx:
                c.r, c.g, c.b, c.a = 0.0, 1.0, 1.0, 1.0
            else:
                c.r, c.g, c.b, c.a = 1.0, 1.0, 1.0, 0.8
            node_m.colors.append(c)
        array.markers.append(node_m)

        seq_m = Marker()
        seq_m.header.stamp = now
        seq_m.header.frame_id = 'map'
        seq_m.ns = 'seq_edges'
        seq_m.id = 1
        seq_m.type = Marker.LINE_LIST
        seq_m.action = Marker.ADD
        seq_m.scale.x = 0.05
        seq_m.color.r, seq_m.color.g = 0.2, 0.5
        seq_m.color.b, seq_m.color.a = 1.0, 0.9

        loop_m = Marker()
        loop_m.header.stamp = now
        loop_m.header.frame_id = 'map'
        loop_m.ns = 'loop_edges'
        loop_m.id = 2
        loop_m.type = Marker.LINE_LIST
        loop_m.action = Marker.ADD
        loop_m.scale.x = 0.08
        loop_m.color.r, loop_m.color.g = 0.0, 1.0
        loop_m.color.b, loop_m.color.a = 0.4, 1.0

        for edge in all_edges:
            p0 = node_by_idx.get(edge.from_index)
            p1 = node_by_idx.get(edge.to_index)
            if p0 is None or p1 is None:
                continue
            is_loop = (edge.from_index, edge.to_index) in loop_edge_set
            target = loop_m if is_loop else seq_m
            pt_a = RosPoint()
            pt_a.x, pt_a.y, pt_a.z = p0.x, p0.y, 0.0
            pt_b = RosPoint()
            pt_b.x, pt_b.y, pt_b.z = p1.x, p1.y, 0.0
            target.points.append(pt_a)
            target.points.append(pt_b)
        array.markers.append(seq_m)
        array.markers.append(loop_m)
        self._pg_marker_pub.publish(array)

    def finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        self.get_logger().info('Finalizing SLAM node...')

        finalize_result = self._orchestrator.finalize()

        if finalize_result.rerender_required:
            nodes = self._pose_graph.get_nodes()
            self._renderer.rerender_all(nodes)
            self._rebuild_path(nodes)
            self._map_dirty = True
            self._publish_map_timer()
            self.get_logger().info('Final map optimization and rendering complete.')

    def destroy_node(self) -> None:
        if self._use_gnss and self._gnss_source is not None:
            self._gnss_source.stop()
        self._scan_source.stop()
        self._odom_source.stop()
        super().destroy_node()
