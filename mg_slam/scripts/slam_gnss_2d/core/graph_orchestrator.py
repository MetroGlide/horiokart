from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from slam_gnss_2d.core.data_types import GnssData, OdomData, PoseEdge, PoseNode, ScanData, SensorFrame
from slam_gnss_2d.pose_graph.base import PoseGraphBuilderBase
from slam_gnss_2d.optimizer.isam2_optimizer import ISAM2Optimizer
from slam_gnss_2d.gnss.anchor_manager import GnssAnchorManager


@dataclass
class ScanProcessResult:
    node: Optional[PoseNode]
    loop_closed: bool
    rerender_required: bool


@dataclass
class FinalizeResult:
    rerender_required: bool


class GraphOrchestrator:
    """単一パイプラインのオーケストレーター。初期方位推定（GNSSアライメント）機能を内包する。"""

    def __init__(
        self,
        *,
        logger: Any,
        pose_graph: PoseGraphBuilderBase,
        use_gnss: bool,
        isam2_relinearize_threshold: float = 0.1,
        anchor_min_fix_status: int = 0,
        gnss_fix_sigma_m: float = 0.02,
        gnss_float_sigma_m: float = 0.5,
        gnss_factor_yaw_variance: float = 1e8,
        gnss_init_distance_m: float = 2.0,
        gnss_max_sigma_m: float = 5.0,
    ) -> None:
        self._logger = logger
        self._pose_graph = pose_graph
        self._use_gnss = use_gnss

        # 常に稼働する単一インクリメンタルオプティマイザ
        self._optimizer = ISAM2Optimizer(
            relinearize_threshold=isam2_relinearize_threshold)

        self._anchor_manager = GnssAnchorManager() if use_gnss else None

        self._anchor_min_fix_status = anchor_min_fix_status
        self._gnss_fix_sigma_m = gnss_fix_sigma_m
        self._gnss_float_sigma_m = gnss_float_sigma_m
        self._gnss_factor_yaw_variance = gnss_factor_yaw_variance
        self._gnss_init_distance_m = gnss_init_distance_m
        self._gnss_max_sigma_m = gnss_max_sigma_m

        self._last_node_index = -1
        self._last_loop_edge_count = 0
        self._initialized = False

        self._state = 'INITIALIZING' if use_gnss else 'RUNNING'
        self._init_rotation = 0.0

    @property
    def anchor_latlon(self) -> tuple[float, float] | None:
        if self._anchor_manager:
            return self._anchor_manager.anchor_latlon
        return None

    @property
    def anchor(self) -> tuple[float, float] | None:
        if self._anchor_manager:
            return self._anchor_manager.anchor_utm
        return None

    @property
    def init_rotation(self) -> float | None:
        return self._init_rotation

    def process_frame(self, frame: SensorFrame) -> ScanProcessResult:
        node = self._pose_graph.add_scan(frame.scan, frame.odom)
        if node is None:
            return ScanProcessResult(node=None, loop_closed=False, rerender_required=False)

        # 1. INITIALIZING 時のアンカー設定と初期方位推定
        if self._use_gnss and self._state == 'INITIALIZING' and frame.gnss is not None and self._anchor_manager:
            if not self._anchor_manager.is_initialized:
                self._anchor_manager.try_set_anchor(
                    frame.gnss, self._anchor_min_fix_status)

            if self._anchor_manager.is_initialized:
                lx, ly = self._anchor_manager.to_local(frame.gnss)
                dist = math.hypot(lx, ly)
                if dist >= self._gnss_init_distance_m:
                    theta0 = math.atan2(ly, lx)
                    nodes = self._pose_graph.get_nodes()
                    node0 = nodes[0]
                    rot = theta0 - node0.yaw
                    self._init_rotation = rot

                    c = math.cos(rot)
                    s = math.sin(rot)

                    self._optimizer.initialize(
                        node0.index, 0.0, 0.0, theta0, 0.05, 10.0)
                    for n in nodes:
                        dx = n.x - node0.x
                        dy = n.y - node0.y
                        n.x = c * dx - s * dy
                        n.y = s * dx + c * dy
                        n.yaw = n.yaw + rot
                        if n.index != node0.index:
                            self._optimizer.add_initial_estimate(
                                n.index, n.x, n.y, n.yaw)

                    edges = self._pose_graph.get_edges()
                    for e in edges:
                        self._optimizer.add_between_factor(
                            e.from_index, e.to_index, e.dx, e.dy, e.dyaw, e.information)

                    self._state = 'RUNNING'
                    self._last_node_index = nodes[-1].index
                    self._initialized = True
                    self._logger.info(
                        f"Graph initialized and aligned to UTM with rotation {rot:.3f} rad")

        # 2. 状態による早期リターン
        if self._state == 'INITIALIZING':
            # まだ初期方位が確定していないためオプティマイザには入れず、ローカルに蓄積するのみ
            return ScanProcessResult(node=node, loop_closed=False, rerender_required=False)

        # 3. RUNNING ステートの処理
        if not self._initialized:
            # GNSS無効時の初回初期化
            self._optimizer.initialize(
                node.index, node.x, node.y, node.yaw, 0.05, 10.0)
            self._initialized = True
            self._last_node_index = node.index

        latest_seq_edge = self._get_latest_seq_edge(node.index)
        if latest_seq_edge is not None and node.index > self._last_node_index:
            prev_pose = self._optimizer.get_pose(latest_seq_edge.from_index)
            if prev_pose is not None:
                px, py, pyaw = prev_pose
                c = math.cos(pyaw)
                s = math.sin(pyaw)
                x = px + c * latest_seq_edge.dx - s * latest_seq_edge.dy
                y = py + s * latest_seq_edge.dx + c * latest_seq_edge.dy
                yaw = pyaw + latest_seq_edge.dyaw
                self._optimizer.add_initial_estimate(node.index, x, y, yaw)
                self._optimizer.add_between_factor(
                    latest_seq_edge.from_index, latest_seq_edge.to_index,
                    latest_seq_edge.dx, latest_seq_edge.dy, latest_seq_edge.dyaw, latest_seq_edge.information
                )
            self._last_node_index = node.index

        loop_closed = False
        if hasattr(self._pose_graph, 'get_loop_edges'):
            loops = self._pose_graph.get_loop_edges()
            if len(loops) > self._last_loop_edge_count:
                for i in range(self._last_loop_edge_count, len(loops)):
                    edge = loops[i]
                    self._optimizer.add_between_factor(
                        edge.from_index, edge.to_index,
                        edge.dx, edge.dy, edge.dyaw, edge.information
                    )
                self._last_loop_edge_count = len(loops)
                loop_closed = True

        if self._use_gnss and frame.gnss is not None and self._anchor_manager and self._anchor_manager.is_initialized:
            sigma_xy = self._sigma_from_gnss(frame.gnss)
            if 0 < sigma_xy <= self._gnss_max_sigma_m:
                gx, gy = self._anchor_manager.to_local(frame.gnss)
                # グラフ全体がGNSSに合わせて回転・平行移動済みなので、gx, gy をそのまま投入する
                self._optimizer.add_gnss_prior(
                    node.index, gx, gy, sigma_xy, self._gnss_factor_yaw_variance
                )

        self._optimizer.update()

        all_poses = self._optimizer.get_all_poses()
        rerender_required = loop_closed

        # オプティマイザの結果をノードに反映
        nodes = self._pose_graph.get_nodes()
        # 最新のポーズが INITIALIZING 後に大きく飛んだ場合（回転等）、
        # マップ全体を再描画する必要があるため、rerender_required を True にする
        if self._use_gnss and self._last_node_index == node.index and self._init_rotation != 0.0 and len(nodes) > 1 and not hasattr(self, '_first_render_done'):
            rerender_required = True
            self._first_render_done = True

        for n in nodes:
            if n.index in all_poses:
                n.x, n.y, n.yaw = all_poses[n.index]

        return ScanProcessResult(
            node=node,
            loop_closed=loop_closed,
            rerender_required=rerender_required,
        )

    def _get_latest_seq_edge(self, node_index: int) -> Optional[PoseEdge]:
        all_edges = self._pose_graph.get_edges()
        if hasattr(self._pose_graph, 'get_loop_edges'):
            loop_set = {(e.from_index, e.to_index)
                        for e in self._pose_graph.get_loop_edges()}
            seq_edges = [e for e in all_edges if (
                e.from_index, e.to_index) not in loop_set]
        else:
            seq_edges = all_edges
        if not seq_edges:
            return None
        candidate = seq_edges[-1]
        if candidate.to_index == node_index:
            return candidate
        return None

    def _sigma_from_gnss(self, gnss: GnssData) -> float:
        cov_xx = float(gnss.covariance[0, 0]
                       ) if gnss.covariance is not None else 0.0
        if cov_xx > 0.0:
            return math.sqrt(cov_xx)
        if gnss.fix_status >= 2:
            return self._gnss_fix_sigma_m
        if gnss.fix_status >= 0:
            return self._gnss_float_sigma_m
        return -1.0

    def finalize(self) -> FinalizeResult:
        self._logger.info("Running offline batch optimization (Placeholder)")
        return FinalizeResult(rerender_required=True)
