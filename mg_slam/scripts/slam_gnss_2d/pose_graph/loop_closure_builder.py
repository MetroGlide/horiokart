from __future__ import annotations

import logging
import math
from typing import Optional

import numpy as np

from .base import PoseGraphBuilderBase
from .scan_matching_builder import ScanMatchingBuilder
from ..data_types import OdomData, PoseEdge, PoseNode, ScanData
from ..scan_matching.base import ScanMatcherBase
from ..optimizer.gtsam_optimizer import GTSAMOptimizer

_logger = logging.getLogger(__name__)


def _scan_to_points(scan: ScanData) -> np.ndarray:
    """有効レンジのみを2D点群 (N, 2) に変換する。"""
    n = len(scan.ranges)
    angles = scan.angle_min + np.arange(n) * scan.angle_increment
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    valid = (ranges >= scan.range_min) & (ranges <= scan.range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def _angle_diff(a: float, b: float) -> float:
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


class LoopClosureBuilder(PoseGraphBuilderBase):
    """ループクロージャ検出 + GTSAM グラフ最適化を行うポーズグラフ構築実装。

    連続フレームのスキャンマッチング処理は ScanMatchingBuilder に委譲する（コンポジション）。
    新しいノードが追加されるたびにループ候補を探索し、
    検証マッチングが収束した場合はグラフにループ辺を追加して最適化を実行する。

    ループ検証マッチャーは連続フレーム用と同じ scan_matcher_type を使用する。
    """

    def __init__(
        self,
        inner: ScanMatchingBuilder,
        loop_matcher: ScanMatcherBase,
        optimizer: GTSAMOptimizer,
        loop_closure_search_radius: float = 2.0,
        loop_closure_min_node_gap: int = 50,
        loop_closure_max_failure_streak: int = 3,
        optimize_every_n_loops: int = 1,
    ) -> None:
        self._inner = inner
        self._loop_matcher = loop_matcher
        self._optimizer = optimizer
        self._search_radius = loop_closure_search_radius
        self._min_node_gap = loop_closure_min_node_gap
        self._max_failure_streak = loop_closure_max_failure_streak
        self._optimize_every_n_loops = optimize_every_n_loops
        self._loop_edges: list[PoseEdge] = []
        self._loop_failure_streak: int = 0
        self._loop_just_closed_flag: bool = False
        self.loop_attempt_count: int = 0
        self.loop_success_count: int = 0

    def add_scan(self, scan: ScanData, odom: OdomData) -> Optional[PoseNode]:
        node = self._inner.add_scan(scan, odom)
        if node is None:
            return None

        self._loop_just_closed_flag = False
        candidates = self._find_loop_candidates(node)
        new_loops_added = 0
        for candidate in candidates:
            if self._try_add_loop_edge(node, candidate):
                new_loops_added += 1

        if new_loops_added > 0:
            total_loops = len(self._loop_edges)
            if total_loops % self._optimize_every_n_loops == 0:
                self._run_optimize()

        return node

    def _find_loop_candidates(self, node: PoseNode) -> list[PoseNode]:
        """ループ候補ノードを返す。

        距離が search_radius 以内かつインデックス差が min_node_gap 以上のノードを対象にする。
        scan データを持たないノードは除外する。
        """
        candidates: list[PoseNode] = []
        for candidate in self._inner.get_nodes():
            if node.index - candidate.index < self._min_node_gap:
                continue
            if candidate.scan is None:
                continue
            dist = math.hypot(node.x - candidate.x, node.y - candidate.y)
            if dist <= self._search_radius:
                candidates.append(candidate)
        return candidates

    def _try_add_loop_edge(self, node: PoseNode, candidate: PoseNode) -> bool:
        """候補ノードとのループ辺を検証して追加する。

        candidate のスキャン点群を src_pts として、node のスキャンに対してマッチングを実行する。
        初期値は pose graph 上の2ノード間の相対ポーズとする。
        streak fallback 設計（設計ルール7に準拠）。

        Returns:
            True: ループ辺が追加された
            False: 未収束またはスキップ
        """
        src_pts = _scan_to_points(candidate.scan)
        if len(src_pts) == 0:
            return False

        # 初期値: candidate ボディフレームから見た node の相対ポーズ
        c = math.cos(-candidate.yaw)
        s = math.sin(-candidate.yaw)
        dx_w = node.x - candidate.x
        dy_w = node.y - candidate.y
        initial_guess = OdomData(
            timestamp=node.timestamp,
            x=c * dx_w - s * dy_w,
            y=s * dx_w + c * dy_w,
            yaw=_angle_diff(node.yaw, candidate.yaw),
        )

        self.loop_attempt_count += 1
        result = self._loop_matcher.match(
            src_pts=src_pts,
            dst=node.scan,
            initial_guess=initial_guess,
        )

        if not result.converged:
            self._loop_failure_streak += 1
            _logger.debug(
                f'Loop match failed: node {node.index} <- candidate {candidate.index} '
                f'(streak={self._loop_failure_streak}/{self._max_failure_streak})'
            )
            if self._loop_failure_streak >= self._max_failure_streak:
                # 連続失敗上限: ループ失敗連鎖を脱出してリセット
                _logger.warning(
                    f'Loop failure streak limit reached at node {node.index}; resetting streak'
                )
                self._loop_failure_streak = 0
            return False

        self._loop_failure_streak = 0
        self.loop_success_count += 1
        self._loop_edges.append(PoseEdge(
            from_index=candidate.index,
            to_index=node.index,
            dx=result.dx,
            dy=result.dy,
            dyaw=result.dyaw,
            information=result.information,
        ))
        _logger.info(
            f'Loop edge added: {candidate.index} -> {node.index} '
            f'(dx={result.dx:.3f} dy={result.dy:.3f} '
            f'dyaw={math.degrees(result.dyaw):.1f}deg)'
        )
        return True

    def _run_optimize(self) -> None:
        """全ノード・全エッジでグラフ最適化を実行し、内部ノードリストを更新する。"""
        all_nodes = self._inner.get_nodes()
        all_edges = self._inner.get_edges() + self._loop_edges
        updated = self._optimizer.optimize(all_nodes, all_edges)
        self._inner.replace_nodes(updated)
        self._loop_just_closed_flag = True
        _logger.info(
            f'Graph optimized: {len(all_nodes)} nodes, '
            f'{len(self._inner.get_edges())} seq edges, '
            f'{len(self._loop_edges)} loop edges'
        )

    def get_nodes(self) -> list[PoseNode]:
        return self._inner.get_nodes()

    def get_edges(self) -> list[PoseEdge]:
        return self._inner.get_edges() + self._loop_edges

    def reset(self) -> None:
        self._inner.reset()
        self._loop_edges.clear()
        self._loop_failure_streak = 0
        self._loop_just_closed_flag = False
        self.loop_attempt_count = 0
        self.loop_success_count = 0

    @property
    def loop_just_closed(self) -> bool:
        result = self._loop_just_closed_flag
        self._loop_just_closed_flag = False
        return result
