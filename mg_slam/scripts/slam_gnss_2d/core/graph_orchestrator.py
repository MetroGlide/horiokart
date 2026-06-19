from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from slam_gnss_2d.core.data_types import GnssData, GnssPrior, OdomData, PoseEdge, PoseNode, ScanData
from slam_gnss_2d.gnss.gnss_anchored_runner import GnssAnchoredRunner
from slam_gnss_2d.input.base import GnssSourceBase
from slam_gnss_2d.pose_graph.base import PoseGraphBuilderBase


@dataclass
class ScanProcessResult:
    node: Optional[PoseNode]
    loop_closed: bool
    rerender_required: bool


@dataclass
class FinalizeResult:
    rerender_required: bool


class GraphOrchestrator:
    """ポーズグラフ更新・GNSS統合・最適化トリガーを統括する。"""

    def __init__(
        self,
        *,
        logger: Any,
        pose_graph: PoseGraphBuilderBase,
        use_gnss: bool,
        enable_incremental_optimizer: bool,
        gnss_missing_grace_frames: int,
        gnss_source: GnssSourceBase | None = None,
        gnss_runner: GnssAnchoredRunner | None = None,
    ) -> None:
        self._logger = logger
        self._pose_graph = pose_graph

        self._use_gnss = use_gnss
        self._enable_incremental_optimizer = enable_incremental_optimizer
        self._gnss_missing_grace_frames = gnss_missing_grace_frames

        self._gnss_source = gnss_source
        self._gnss_runner = gnss_runner

        self._gnss_missing_streak = 0
        self._gnss_degraded = False

        self._wire_shared_optimizer()

    def _wire_shared_optimizer(self) -> None:
        from slam_gnss_2d.pose_graph.loop_closure_builder import LoopClosureBuilder
        if not isinstance(self._pose_graph, LoopClosureBuilder):
            return
        if self._gnss_runner is None:
            return
        shared_optimizer = self._gnss_runner._optimizer
        self._pose_graph.set_incremental_optimizer(shared_optimizer)
        self._logger.info('Shared IncrementalOptimizer wired: loop closure will use GNSS-integrated optimization')

    def process_scan(self, scan: ScanData, odom: OdomData) -> ScanProcessResult:
        node = self._pose_graph.add_scan(scan, odom)
        if node is None:
            return ScanProcessResult(
                node=None,
                loop_closed=False,
                rerender_required=False,
            )

        latest_edge = self._get_latest_edge(node.index)
        rerender_required = False

        if self._use_gnss and self._enable_incremental_optimizer:
            rerender_required = self._apply_integrated_incremental(
                scan=scan,
                latest_node=node,
                latest_edge=latest_edge,
            )

            # 遅延されたループ最適化があれば実行
            if hasattr(self._pose_graph, 'optimize_pending') and self._pose_graph.optimize_pending:
                self._pose_graph.run_optimize_pending()
                rerender_required = True

        loop_closed = self._pose_graph.loop_just_closed

        return ScanProcessResult(
            node=node,
            loop_closed=loop_closed,
            rerender_required=rerender_required,
        )

    def finalize(self) -> FinalizeResult:
        rerender_required = False

        if hasattr(self._pose_graph, '_run_optimize'):
            self._pose_graph._run_optimize()
            rerender_required = True

        if self._use_gnss and self._gnss_runner is not None:
            self._gnss_runner._optimizer.update()
            self._gnss_runner._optimizer.update()
            all_poses = self._gnss_runner._optimizer.get_all_poses()
            if all_poses:
                nodes = self._pose_graph.get_nodes()
                for node in nodes:
                    pose = all_poses.get(node.index)
                    if pose is None:
                        continue
                    node.x, node.y, node.yaw = pose
                rerender_required = True

        return FinalizeResult(
            rerender_required=rerender_required,
        )

    def _get_latest_edge(self, node_index: int) -> Optional[PoseEdge]:
        all_edges = self._pose_graph.get_edges()
        if hasattr(self._pose_graph, 'get_loop_edges'):
            loop_edges = self._pose_graph.get_loop_edges()
            loop_set = {(e.from_index, e.to_index) for e in loop_edges}
            seq_edges = [e for e in all_edges if (e.from_index, e.to_index) not in loop_set]
        else:
            seq_edges = all_edges

        if not seq_edges:
            return None
        candidate = seq_edges[-1]
        if candidate.to_index == node_index:
            return candidate
        return None

    def _apply_integrated_incremental(
        self,
        *,
        scan: ScanData,
        latest_node: PoseNode,
        latest_edge: PoseEdge | None,
    ) -> bool:
        if self._gnss_runner is None or self._gnss_source is None:
            return False

        gnss = self._gnss_source.get_gnss_at(scan.timestamp)
        if gnss is None:
            self._gnss_missing_streak += 1
            if self._gnss_missing_streak >= self._gnss_missing_grace_frames:
                if not self._gnss_degraded:
                    self._gnss_degraded = True
                    self._logger.warn(
                        f'GNSS missing for {self._gnss_missing_streak} frames: '
                        'degrading to scan/odom only mode'
                    )
                return False
        else:
            if self._gnss_degraded:
                self._logger.info(
                    'GNSS recovered: re-enabling integrated GNSS factors')
            self._gnss_degraded = False
            self._gnss_missing_streak = 0

        if self._gnss_degraded:
            return False

        anchored_now = self._gnss_runner.on_gnss(gnss)
        if anchored_now:
            anchor = self._gnss_runner.anchor
            if anchor is not None:
                self._logger.info(
                    f'GNSS anchor set: E={anchor[0]:.3f}, N={anchor[1]:.3f}'
                )

        nodes = self._pose_graph.get_nodes()
        edges = self._pose_graph.get_edges()
        updates, rerender_required = self._gnss_runner.process(
            nodes=nodes,
            edges=edges,
            latest_node=latest_node,
            latest_edge=latest_edge,
        )
        if not updates:
            return False

        for node in nodes:
            pose = updates.get(node.index)
            if pose is None:
                continue
            node.x, node.y, node.yaw = pose

        if rerender_required:
            self._logger.info('GNSS anchored mode entered RUNNING state')

        return rerender_required
