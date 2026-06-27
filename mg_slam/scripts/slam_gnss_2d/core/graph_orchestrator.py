from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass
from typing import Any, Optional

from slam_gnss_2d.core.data_types import (
    GnssData,
    GnssPrior,
    PoseEdge,
    PoseNode,
    ScanProcessResult,
    SensorFrame,
)
from slam_gnss_2d.pose_graph.base import PoseGraphBuilderBase
from slam_gnss_2d.optimizer.isam2_optimizer import ISAM2Optimizer
from slam_gnss_2d.gnss.anchor_manager import GnssAnchorManager


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
            return ScanProcessResult(node=None, loop_closed=False, rerender_required=False, new_loop_edges=[])

        self._initialize_with_gnss_if_ready(frame)

        if self._state == 'INITIALIZING':
            return ScanProcessResult(
                node=node,
                loop_closed=False,
                rerender_required=False,
                new_seq_edge=self._get_latest_seq_edge(node.index),
                new_loop_edges=[],
            )

        self._initialize_optimizer_if_needed(node)
        new_seq_edge = self._add_latest_seq_edge(node)
        new_loop_edges, loop_closed = self._add_new_loop_edges()
        new_gnss_prior = self._add_gnss_prior(frame, node)
        self._optimizer.update()
        rerender_required = self._apply_optimized_poses(node, loop_closed)

        return ScanProcessResult(
            node=node,
            loop_closed=loop_closed,
            rerender_required=rerender_required,
            new_seq_edge=new_seq_edge,
            new_loop_edges=new_loop_edges,
            new_gnss_prior=new_gnss_prior,
        )

    def _initialize_with_gnss_if_ready(self, frame: SensorFrame) -> None:
        if (
            not self._use_gnss
            or self._state != 'INITIALIZING'
            or frame.gnss is None
            or self._anchor_manager is None
        ):
            return

        if not self._anchor_manager.is_initialized:
            self._anchor_manager.try_set_anchor(
                frame.gnss, self._anchor_min_fix_status)

        if not self._anchor_manager.is_initialized:
            return

        lx, ly = self._anchor_manager.to_local(frame.gnss)
        if math.hypot(lx, ly) < self._gnss_init_distance_m:
            return

        theta0 = math.atan2(ly, lx)
        nodes = self._pose_graph.get_nodes()
        node0 = nodes[0]
        rot = theta0 - node0.yaw
        self._init_rotation = rot

        c = math.cos(rot)
        s = math.sin(rot)

        self._optimizer.initialize(node0.index, 0.0, 0.0, theta0, 0.05, 10.0)
        for node in nodes:
            dx = node.x - node0.x
            dy = node.y - node0.y
            node.x = c * dx - s * dy
            node.y = s * dx + c * dy
            node.yaw = node.yaw + rot
            if node.index != node0.index:
                self._optimizer.add_initial_estimate(
                    node.index, node.x, node.y, node.yaw)

        for edge in self._pose_graph.get_edges():
            self._optimizer.add_between_factor(
                edge.from_index,
                edge.to_index,
                edge.dx,
                edge.dy,
                edge.dyaw,
                edge.information,
            )

        self._state = 'RUNNING'
        self._last_node_index = nodes[-1].index
        self._initialized = True
        self._logger.info(
            f"Graph initialized and aligned to UTM with rotation {rot:.3f} rad")

    def _initialize_optimizer_if_needed(self, node: PoseNode) -> None:
        if self._initialized:
            return
        self._optimizer.initialize(
            node.index, node.x, node.y, node.yaw, 0.05, 10.0)
        self._initialized = True
        self._last_node_index = node.index

    def _add_latest_seq_edge(self, node: PoseNode) -> Optional[PoseEdge]:
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
                return latest_seq_edge
            self._last_node_index = node.index
        return None

    def _add_new_loop_edges(self) -> tuple[list[PoseEdge], bool]:
        new_loop_edges: list[PoseEdge] = []
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
                    new_loop_edges.append(edge)
                self._last_loop_edge_count = len(loops)
                loop_closed = True
        return new_loop_edges, loop_closed

    def _add_gnss_prior(
        self,
        frame: SensorFrame,
        node: PoseNode,
    ) -> Optional[GnssPrior]:
        if (
            not self._use_gnss
            or frame.gnss is None
            or self._anchor_manager is None
            or not self._anchor_manager.is_initialized
        ):
            return None

        sigma_xy = self._sigma_from_gnss(frame.gnss)
        if sigma_xy <= 0 or sigma_xy > self._gnss_max_sigma_m:
            return None

        gx, gy = self._anchor_manager.to_local(frame.gnss)
        self._optimizer.add_gnss_prior(
            node.index, gx, gy, sigma_xy, self._gnss_factor_yaw_variance)
        info_2x2 = np.zeros((2, 2), dtype=np.float64)
        inv_var = 1.0 / max(sigma_xy * sigma_xy, 1e-12)
        info_2x2[0, 0] = inv_var
        info_2x2[1, 1] = inv_var
        return GnssPrior(
            node_index=node.index,
            x=gx,
            y=gy,
            information=info_2x2,
        )

    def _apply_optimized_poses(self, node: PoseNode, loop_closed: bool) -> bool:
        all_poses = self._optimizer.get_all_poses()
        rerender_required = loop_closed

        nodes = self._pose_graph.get_nodes()
        if (
            self._use_gnss
            and self._last_node_index == node.index
            and self._init_rotation != 0.0
            and len(nodes) > 1
            and not hasattr(self, '_first_render_done')
        ):
            rerender_required = True
            self._first_render_done = True

        for n in nodes:
            if n.index in all_poses:
                n.x, n.y, n.yaw = all_poses[n.index]
        return rerender_required

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
