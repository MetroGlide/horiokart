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

from slam_gnss_2d.component_factory import build_gnss_aligner, build_pose_graph_builder
from slam_gnss_2d.config import SlamConfig
from slam_gnss_2d.data_types import PoseNode, ScanData
from slam_gnss_2d.graph_orchestrator import GnssBatchResult, GraphOrchestrator
from slam_gnss_2d.gnss.constraint_inserter import GnssConstraintInserter
from slam_gnss_2d.input.base import GnssSourceBase, OdomSourceBase, ScanSourceBase
from slam_gnss_2d.map_manager.opencv_renderer import OpenCVRenderer
from slam_gnss_2d.optimizer.gtsam_optimizer import GTSAMOptimizer


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
        self._pg_marker_pub = self.create_publisher(
            MarkerArray, 'slam_gnss_2d/pose_graph', 1)
        self._path_before_pub = self.create_publisher(
            Path, 'slam_gnss_2d/path_before_optimize', 1)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._gnss_raw_pub = None
        self._gnss_prior_pub = None

        self._path_msg = Path()
        self._path_msg.header.frame_id = 'map'
        self._map_dirty = False
        self._map_to_odom_x = 0.0
        self._map_to_odom_y = 0.0
        self._map_to_odom_yaw = 0.0

        self._scan_source.set_scan_callback(self._on_scan)
        self._odom_source.start()
        self._scan_source.start()

        self._enable_scan_matching = cfg.enable_scan_matching
        self._enable_gnss = cfg.enable_gnss
        self._enable_loop_closure = cfg.enable_loop_closure
        self._enable_incremental_optimizer = cfg.enable_incremental_optimizer
        self._enable_batch_optimizer = cfg.enable_batch_optimizer
        self._batch_trigger_mode = cfg.batch_trigger_mode
        self._batch_trigger_on_loop_close = cfg.batch_trigger_on_loop_close
        self._batch_trigger_every_n_nodes = cfg.batch_trigger_every_n_nodes
        self._batch_optimize_on_finalize = cfg.batch_optimize_on_finalize
        self._gnss_missing_grace_frames = cfg.gnss_missing_grace_frames
        self._gnss_missing_streak = 0
        self._gnss_degraded = False

        self._use_gnss = cfg.use_gnss and self._enable_gnss
        self._gnss_mode = self._normalize_gnss_mode(cfg.gnss_mode)
        self._gnss_source = None
        self._gnss_aligner = None
        self._gnss_inserter = None
        self._gnss_optimizer = None
        self._gnss_runner = None
        if self._use_gnss:
            self._gnss_source = self._setup_gnss_source(cfg)
            self._gnss_source.start()
            self._gnss_aligner = build_gnss_aligner(cfg)
            self._gnss_inserter = GnssConstraintInserter(
                default_noise_xy_m=cfg.gnss_noise_xy_m,
                max_time_delta_s=cfg.gnss_max_time_delta_s,
            )
            self._gnss_optimizer = GTSAMOptimizer()
            self._gnss_raw_pub = self.create_publisher(
                MarkerArray, 'slam_gnss_2d/gnss_raw_markers', 1)
            self._gnss_prior_pub = self.create_publisher(
                MarkerArray, 'slam_gnss_2d/gnss_prior_markers', 1)
            if self._gnss_mode == 'integrated' and self._enable_incremental_optimizer:
                from slam_gnss_2d.gnss.gnss_anchored_runner import GnssAnchoredParams, GnssAnchoredRunner

                params = GnssAnchoredParams(
                    init_distance_m=cfg.gnss_init_distance_m,
                    anchor_min_fix_status=cfg.gnss_anchor_min_fix_status,
                    anchor_sigma_m=cfg.gnss_anchor_sigma_m,
                    init_yaw_sigma_rad=cfg.gnss_init_yaw_sigma_rad,
                    gnss_fix_sigma_m=cfg.gnss_fix_sigma_m,
                    gnss_float_sigma_m=cfg.gnss_float_sigma_m,
                    gnss_factor_yaw_variance=cfg.gnss_factor_yaw_variance,
                    gnss_max_sigma_m=cfg.gnss_max_sigma_m,
                    gnss_rerender_threshold_m=cfg.gnss_rerender_threshold_m,
                )
                if cfg.gnss_optimizer == 'gtsam':
                    from slam_gnss_2d.optimizer.gtsam_incremental_adapter import GTSAMIncrementalAdapter
                    opt = GTSAMIncrementalAdapter()
                else:
                    from slam_gnss_2d.optimizer.isam2_optimizer import ISAM2Optimizer
                    opt = ISAM2Optimizer(
                        relinearize_threshold=cfg.isam2_relinearize_threshold)

                self._gnss_runner = GnssAnchoredRunner(
                    params=params,
                    optimizer=opt,
                )

        self._orchestrator = GraphOrchestrator(
            logger=self.get_logger(),
            pose_graph=self._pose_graph,
            use_gnss=self._use_gnss,
            gnss_mode=self._gnss_mode,
            enable_incremental_optimizer=self._enable_incremental_optimizer,
            enable_batch_optimizer=self._enable_batch_optimizer,
            batch_trigger_mode=self._batch_trigger_mode,
            batch_trigger_on_loop_close=self._batch_trigger_on_loop_close,
            batch_trigger_every_n_nodes=self._batch_trigger_every_n_nodes,
            batch_optimize_on_finalize=self._batch_optimize_on_finalize,
            gnss_missing_grace_frames=self._gnss_missing_grace_frames,
            gnss_source=self._gnss_source,
            gnss_runner=self._gnss_runner,
            gnss_aligner=self._gnss_aligner,
            gnss_inserter=self._gnss_inserter,
            gnss_optimizer=self._gnss_optimizer,
        )

        self.create_timer(1.0 / cfg.map_publish_hz, self._publish_map_timer)
        self.create_timer(0.1, self._publish_tf)

        self._scan_recv_count = 0
        self._odom_miss_count = 0
        self._node_count = 0
        self._last_stat_time = time.monotonic()
        self._finalized = False

        self.get_logger().info(
            f'{node_name} started (builder={cfg.pose_graph_builder}, '
            f'matcher={cfg.scan_matcher_type}, ref={cfg.scan_reference})\n'
            f'  scan: {cfg.scan_topic}, odom: {cfg.odom_topic}\n'
            f'  map: dynamic @ {cfg.map_resolution}m/px, margin={cfg.map_expansion_margin}m\n'
            f'  gnss_mode: {self._gnss_mode}, incremental={self._enable_incremental_optimizer}, '
            f'batch={self._enable_batch_optimizer}'
        )

    @abstractmethod
    def _setup_io(self, cfg: SlamConfig) -> tuple[ScanSourceBase, OdomSourceBase]:
        """IO ソース（スキャン・オドメトリ）を構築して返す。start() は呼ばない。"""
        raise NotImplementedError

    def _setup_gnss_source(self, cfg: SlamConfig) -> GnssSourceBase:
        """GNSS ソースを構築して返す。use_gnss=True の場合のみ呼び出される。"""
        raise NotImplementedError

    @staticmethod
    def _normalize_gnss_mode(mode: str) -> str:
        if mode == 'gnss_anchored':
            return 'integrated'
        if mode == 'batch':
            return 'split_align'
        return mode

    def _is_split_align_mode(self) -> bool:
        return self._gnss_mode == 'split_align'

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
        self.declare_parameter(
            'loop_closure_matcher_type', 'icp')  # "icp" | "ndt"
        self.declare_parameter('loop_closure_max_dyaw_deg', 90.0)   # [deg]
        # パス交差 false positive 排除: |dyaw| が [crossing_reject, 180-crossing_reject] 帯域なら拒否
        self.declare_parameter(
            'loop_closure_crossing_reject_deg', 0.0)  # [deg], 0.0で無効
        self.declare_parameter('loop_closure_submap_radius', 5.0)   # [m]
        # ループ辺スコア上限。ICP:平均点対線残差[m] / NDT:平均負対数尤度。0.0で無効
        self.declare_parameter('loop_closure_max_score', 0.0)

        # 比較実験向け機能トグル
        self.declare_parameter('enable_scan_matching', True)
        self.declare_parameter('enable_gnss', True)
        self.declare_parameter('enable_loop_closure', True)

        # 最適化トグル
        self.declare_parameter('enable_incremental_optimizer', True)
        self.declare_parameter('enable_batch_optimizer', True)
        # "manual" | "event" | "periodic"
        self.declare_parameter('batch_trigger_mode', 'event')
        self.declare_parameter('batch_trigger_on_loop_close', True)
        self.declare_parameter('batch_trigger_every_n_nodes', 100)
        self.declare_parameter('batch_optimize_on_finalize', True)
        self.declare_parameter('gnss_missing_grace_frames', 30)

        # GNSS 拘束（use_gnss == True のとき slam_offline_node.py が使用する）
        self.declare_parameter('use_gnss', False)
        # "integrated" | "split_align" (legacy: "gnss_anchored", "batch")
        self.declare_parameter('gnss_mode', 'integrated')
        self.declare_parameter('gnss_topic', '/gps/fix')
        self.declare_parameter('gnss_noise_xy_m', 3.0)        # [m]
        # "kinematic_heading" | "precision_weighted"
        self.declare_parameter('gnss_aligner', 'kinematic_heading')
        self.declare_parameter('kinematic_min_speed_ms', 0.5)  # [m/s]
        self.declare_parameter('gnss_max_time_delta_s', 5.0)   # [s]
        # "navsat_fix" | "navpvt"
        self.declare_parameter('gnss_source_type', 'navsat_fix')
        self.declare_parameter('gnss_navpvt_topic', '/ublox/navpvt')
        self.declare_parameter('navpvt_hacc_scale', 1.0)
        self.declare_parameter('gnss_init_distance_m', 0.5)
        self.declare_parameter('gnss_anchor_min_fix_status', 0)
        self.declare_parameter('gnss_anchor_sigma_m', 0.05)
        self.declare_parameter('gnss_init_yaw_sigma_rad', 10.0)
        self.declare_parameter('gnss_fix_sigma_m', 0.02)
        self.declare_parameter('gnss_float_sigma_m', 0.5)
        self.declare_parameter('gnss_factor_yaw_variance', 1e8)
        self.declare_parameter('isam2_relinearize_threshold', 0.1)
        self.declare_parameter('gnss_max_sigma_m', 2.0)  # [m]
        self.declare_parameter('gnss_rerender_threshold_m', 0.1)  # [m]
        self.declare_parameter('gnss_optimizer', 'isam2')

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
            loop_closure_matcher_type=self.get_parameter(
                'loop_closure_matcher_type').value,
            loop_closure_max_dyaw_deg=self.get_parameter(
                'loop_closure_max_dyaw_deg').value,
            loop_closure_crossing_reject_deg=self.get_parameter(
                'loop_closure_crossing_reject_deg').value,
            loop_closure_submap_radius=self.get_parameter(
                'loop_closure_submap_radius').value,
            loop_closure_max_score=self.get_parameter(
                'loop_closure_max_score').value,
            enable_scan_matching=self.get_parameter(
                'enable_scan_matching').value,
            enable_gnss=self.get_parameter('enable_gnss').value,
            enable_loop_closure=self.get_parameter(
                'enable_loop_closure').value,
            enable_incremental_optimizer=self.get_parameter(
                'enable_incremental_optimizer').value,
            enable_batch_optimizer=self.get_parameter(
                'enable_batch_optimizer').value,
            batch_trigger_mode=self.get_parameter('batch_trigger_mode').value,
            batch_trigger_on_loop_close=self.get_parameter(
                'batch_trigger_on_loop_close').value,
            batch_trigger_every_n_nodes=self.get_parameter(
                'batch_trigger_every_n_nodes').value,
            batch_optimize_on_finalize=self.get_parameter(
                'batch_optimize_on_finalize').value,
            gnss_missing_grace_frames=self.get_parameter(
                'gnss_missing_grace_frames').value,
            use_gnss=self.get_parameter('use_gnss').value,
            gnss_mode=self._normalize_gnss_mode(
                self.get_parameter('gnss_mode').value),
            gnss_topic=self.get_parameter('gnss_topic').value,
            gnss_noise_xy_m=self.get_parameter('gnss_noise_xy_m').value,
            gnss_aligner=self.get_parameter('gnss_aligner').value,
            kinematic_min_speed_ms=self.get_parameter(
                'kinematic_min_speed_ms').value,
            gnss_max_time_delta_s=self.get_parameter(
                'gnss_max_time_delta_s').value,
            gnss_source_type=self.get_parameter('gnss_source_type').value,
            gnss_navpvt_topic=self.get_parameter('gnss_navpvt_topic').value,
            navpvt_hacc_scale=self.get_parameter('navpvt_hacc_scale').value,
            gnss_init_distance_m=self.get_parameter(
                'gnss_init_distance_m').value,
            gnss_anchor_min_fix_status=self.get_parameter(
                'gnss_anchor_min_fix_status').value,
            gnss_anchor_sigma_m=self.get_parameter(
                'gnss_anchor_sigma_m').value,
            gnss_init_yaw_sigma_rad=self.get_parameter(
                'gnss_init_yaw_sigma_rad').value,
            gnss_fix_sigma_m=self.get_parameter('gnss_fix_sigma_m').value,
            gnss_float_sigma_m=self.get_parameter('gnss_float_sigma_m').value,
            gnss_factor_yaw_variance=self.get_parameter(
                'gnss_factor_yaw_variance').value,
            isam2_relinearize_threshold=self.get_parameter(
                'isam2_relinearize_threshold').value,
            gnss_max_sigma_m=self.get_parameter('gnss_max_sigma_m').value,
            gnss_rerender_threshold_m=self.get_parameter(
                'gnss_rerender_threshold_m').value,
            gnss_optimizer=self.get_parameter('gnss_optimizer').value,
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

        if result.batch_result is not None:
            self._apply_batch_result(result.batch_result)
            self._publish_pose_graph_markers()
            return

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

    def _apply_batch_result(self, result: GnssBatchResult) -> None:
        self._publish_gnss_raw_markers(result.gnss_list, result.transform)
        self._path_before_pub.publish(self._path_msg)
        self._renderer.rerender_all(result.updated_nodes)
        self._rebuild_path(result.updated_nodes)
        self._publish_gnss_prior_markers(result.updated_nodes, result.priors)
        self._map_dirty = True
        self.get_logger().info('GNSS batch phase complete: map re-rendered with GNSS constraints')

    def _run_split_align_batch_phase(self) -> bool:
        """split_align 例外モードのGNSSバッチ処理を明示的に実行する。"""
        result = self._orchestrator.run_split_align_batch()
        if result is None:
            return False
        self._apply_batch_result(result)
        self._publish_pose_graph_markers()
        return True

    def _publish_gnss_raw_markers(self, gnss_list, transform) -> None:
        """GNSS点群を SLAM 座標系に変換してマゼンタ色の SPHERE_LIST で配信する。"""
        if self._gnss_raw_pub is None:
            return
        tx, ty, rot = transform
        cos_r = math.cos(rot)
        sin_r = math.sin(rot)
        array = MarkerArray()
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.ns = 'gnss_raw'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.5
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 0.9
        for gnss in gnss_list:
            x_slam = cos_r * gnss.x - sin_r * gnss.y + tx
            y_slam = sin_r * gnss.x + cos_r * gnss.y + ty
            pt = RosPoint()
            pt.x, pt.y, pt.z = x_slam, y_slam, 0.0
            m.points.append(pt)
        array.markers.append(m)
        self._gnss_raw_pub.publish(array)

    def _publish_gnss_prior_markers(self, updated_nodes, priors) -> None:
        """最適化後ノード位置と GNSS 拘束座標を線分で接続して配信する。"""
        if self._gnss_prior_pub is None:
            return
        node_by_idx = {n.index: n for n in updated_nodes}
        array = MarkerArray()
        line_m = Marker()
        line_m.header.stamp = self.get_clock().now().to_msg()
        line_m.header.frame_id = 'map'
        line_m.ns = 'gnss_connections'
        line_m.id = 0
        line_m.type = Marker.LINE_LIST
        line_m.action = Marker.ADD
        line_m.scale.x = 0.05
        line_m.color.r, line_m.color.g = 0.8, 0.0
        line_m.color.b, line_m.color.a = 0.8, 0.8
        for prior in priors:
            node = node_by_idx.get(prior.node_index)
            if node is None:
                continue
            pt_node = RosPoint()
            pt_node.x, pt_node.y, pt_node.z = node.x, node.y, 0.0
            pt_gnss = RosPoint()
            pt_gnss.x, pt_gnss.y, pt_gnss.z = prior.x, prior.y, 0.0
            line_m.points.append(pt_node)
            line_m.points.append(pt_gnss)
        array.markers.append(line_m)
        self._gnss_prior_pub.publish(array)

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
        if finalize_result.batch_result is not None:
            self._apply_batch_result(finalize_result.batch_result)
            finalize_result.rerender_required = False

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
