from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from .data_types import GnssData, GnssPrior, OdomData, PoseEdge, PoseNode, ScanData
from .gnss.constraint_inserter import GnssConstraintInserter
from .gnss.gnss_anchored_runner import GnssAnchoredRunner
from .gnss.aligner_base import GnssAlignerBase
from .input.base import GnssSourceBase
from .optimizer.gtsam_optimizer import GTSAMOptimizer
from .pose_graph.base import PoseGraphBuilderBase


@dataclass
class GnssBatchResult:
    gnss_list: list[GnssData]
    transform: tuple[float, float, float]
    priors: list[GnssPrior]
    updated_nodes: list[PoseNode]


@dataclass
class ScanProcessResult:
    node: Optional[PoseNode]
    loop_closed: bool
    rerender_required: bool
    batch_result: Optional[GnssBatchResult]


@dataclass
class FinalizeResult:
    rerender_required: bool
    batch_result: Optional[GnssBatchResult]


class GraphOrchestrator:
    """ポーズグラフ更新・GNSS統合・最適化トリガーを統括する。"""

    def __init__(
        self,
        *,
        logger: Any,
        pose_graph: PoseGraphBuilderBase,
        use_gnss: bool,
        gnss_mode: str,
        enable_incremental_optimizer: bool,
        enable_batch_optimizer: bool,
        batch_trigger_mode: str,
        batch_trigger_on_loop_close: bool,
        batch_trigger_every_n_nodes: int,
        batch_optimize_on_finalize: bool,
        gnss_missing_grace_frames: int,
        gnss_source: GnssSourceBase | None = None,
        gnss_runner: GnssAnchoredRunner | None = None,
        gnss_aligner: GnssAlignerBase | None = None,
        gnss_inserter: GnssConstraintInserter | None = None,
        gnss_optimizer: GTSAMOptimizer | None = None,
    ) -> None:
        self._logger = logger
        self._pose_graph = pose_graph

        self._use_gnss = use_gnss
        self._gnss_mode = gnss_mode
        self._enable_incremental_optimizer = enable_incremental_optimizer
        self._enable_batch_optimizer = enable_batch_optimizer
        self._batch_trigger_mode = batch_trigger_mode
        self._batch_trigger_on_loop_close = batch_trigger_on_loop_close
        self._batch_trigger_every_n_nodes = batch_trigger_every_n_nodes
        self._batch_optimize_on_finalize = batch_optimize_on_finalize
        self._gnss_missing_grace_frames = gnss_missing_grace_frames

        self._gnss_source = gnss_source
        self._gnss_runner = gnss_runner
        self._gnss_aligner = gnss_aligner
        self._gnss_inserter = gnss_inserter
        self._gnss_optimizer = gnss_optimizer

        self._gnss_missing_streak = 0
        self._gnss_degraded = False

    def process_scan(self, scan: ScanData, odom: OdomData) -> ScanProcessResult:
        node = self._pose_graph.add_scan(scan, odom)
        if node is None:
            return ScanProcessResult(
                node=None,
                loop_closed=False,
                rerender_required=False,
                batch_result=None,
            )

        latest_edge = self._get_latest_edge(node.index)
        rerender_required = False

        if self._use_gnss and self._is_integrated_mode() and self._enable_incremental_optimizer:
            rerender_required = self._apply_integrated_incremental(
                scan=scan,
                latest_node=node,
                latest_edge=latest_edge,
            )

        loop_closed = self._pose_graph.loop_just_closed
        batch_result = None
        if self._use_gnss and self._is_integrated_mode() and self._should_run_batch_optimize(node.index, loop_closed):
            batch_result = self._run_gnss_batch()
            if batch_result is not None:
                rerender_required = True

        return ScanProcessResult(
            node=node,
            loop_closed=loop_closed,
            rerender_required=rerender_required,
            batch_result=batch_result,
        )

    def run_split_align_batch(self) -> Optional[GnssBatchResult]:
        if not self._use_gnss or not self._is_split_align_mode() or not self._enable_batch_optimizer:
            return None
        return self._run_gnss_batch()

    def finalize(self) -> FinalizeResult:
        rerender_required = False

        if hasattr(self._pose_graph, '_run_optimize'):
            self._pose_graph._run_optimize()
            rerender_required = True

        if self._use_gnss and self._is_integrated_mode() and self._gnss_runner is not None:
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

        batch_result = None
        if (
            self._use_gnss
            and self._is_integrated_mode()
            and self._enable_batch_optimizer
            and self._batch_optimize_on_finalize
            and self._batch_trigger_mode != 'manual'
        ):
            self._logger.info('Running final GNSS batch optimization')
            batch_result = self._run_gnss_batch()
            if batch_result is not None:
                rerender_required = True

        return FinalizeResult(
            rerender_required=rerender_required,
            batch_result=batch_result,
        )

    def _get_latest_edge(self, node_index: int) -> Optional[PoseEdge]:
        edges = self._pose_graph.get_edges()
        if not edges:
            return None
        candidate = edges[-1]
        if candidate.to_index == node_index:
            return candidate
        return None

    def _is_integrated_mode(self) -> bool:
        return self._gnss_mode == 'integrated'

    def _is_split_align_mode(self) -> bool:
        return self._gnss_mode == 'split_align'

    def _should_run_batch_optimize(self, node_index: int, loop_closed: bool) -> bool:
        if not self._enable_batch_optimizer:
            return False

        if self._batch_trigger_on_loop_close and loop_closed:
            return True

        if self._batch_trigger_mode == 'manual':
            return False
        if self._batch_trigger_mode == 'event':
            return loop_closed
        if self._batch_trigger_mode == 'periodic':
            n = self._batch_trigger_every_n_nodes
            return n > 0 and node_index > 0 and node_index % n == 0

        return False

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

    def _run_gnss_batch(self) -> Optional[GnssBatchResult]:
        if (
            self._gnss_source is None
            or self._gnss_aligner is None
            or self._gnss_inserter is None
            or self._gnss_optimizer is None
        ):
            self._logger.warn(
                'GNSS batch skipped: components are not initialized')
            return None

        gnss_list = self._gnss_source.get_all_gnss()
        if not gnss_list:
            self._logger.warn('GNSS batch skipped: no valid GNSS fixes')
            return None

        nodes = self._pose_graph.get_nodes()
        edges = self._pose_graph.get_edges()
        if not nodes:
            self._logger.warn('GNSS batch skipped: pose graph is empty')
            return None

        transform = self._gnss_aligner.estimate_transform(nodes, gnss_list)
        tx, ty, rot = transform
        self._logger.info(
            f'GNSS align: tx={tx:.2f}m ty={ty:.2f}m rot={math.degrees(rot):.2f}deg'
            f' ({len(gnss_list)} GNSS fixes, {len(nodes)} nodes)'
        )

        priors = self._gnss_inserter.build_priors(nodes, gnss_list, transform)
        self._logger.info(f'GNSS inserting {len(priors)} prior constraints')

        updated_nodes = self._gnss_optimizer.optimize(
            nodes, edges, gnss_priors=priors)
        self._apply_updated_nodes(updated_nodes)

        return GnssBatchResult(
            gnss_list=gnss_list,
            transform=transform,
            priors=priors,
            updated_nodes=updated_nodes,
        )

    def _apply_updated_nodes(self, updated_nodes: list[PoseNode]) -> None:
        nodes = self._pose_graph.get_nodes()
        updated_by_idx = {node.index: node for node in updated_nodes}
        for node in nodes:
            updated = updated_by_idx.get(node.index)
            if updated is None:
                continue
            node.x = updated.x
            node.y = updated.y
            node.yaw = updated.yaw
