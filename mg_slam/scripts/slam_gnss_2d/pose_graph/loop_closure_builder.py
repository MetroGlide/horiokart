from __future__ import annotations

import logging
import math
from typing import Optional

import numpy as np

from slam_gnss_2d.pose_graph.base import PoseGraphBuilderBase
from slam_gnss_2d.pose_graph.scan_matching_builder import ScanMatchingBuilder
from slam_gnss_2d.core.data_types import OdomData, PoseEdge, PoseNode, ScanData
from slam_gnss_2d.core.geometry import (
    angle_diff,
    points_local_to_world,
    points_world_to_local,
    scan_to_points,
    world_delta_to_local,
)
from slam_gnss_2d.scan_matching.base import ScanMatcherBase

_logger = logging.getLogger(__name__)


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
        loop_closure_search_radius: float,
        loop_closure_min_node_gap: int,
        loop_closure_max_failure_streak: int,
        max_loop_dyaw_deg: float,
        loop_closure_crossing_reject_deg: float,
        loop_closure_submap_radius: float,
        loop_closure_max_score: float,
    ) -> None:
        self._inner = inner
        self._loop_matcher = loop_matcher
        self._search_radius = loop_closure_search_radius
        self._min_node_gap = loop_closure_min_node_gap
        self._max_failure_streak = loop_closure_max_failure_streak
        self._max_loop_dyaw_rad = math.radians(max_loop_dyaw_deg)
        # パス交差排除: 0.0 のとき無効。有効時は [reject_rad, π - reject_rad] 帯域を拒否する。
        self._crossing_reject_rad = math.radians(
            loop_closure_crossing_reject_deg) if loop_closure_crossing_reject_deg > 0.0 else 0.0
        self._submap_radius = loop_closure_submap_radius
        self._max_score = loop_closure_max_score
        self._loop_edges: list[PoseEdge] = []
        self._loop_failure_streak: int = 0
        self._loop_just_closed_flag: bool = False
        self.loop_attempt_count: int = 0
        self.loop_success_count: int = 0
        self.loop_success_count: int = 0
        self._all_nodes_cache: list[PoseNode] = []

    def add_scan(self, scan: ScanData, odom: OdomData) -> Optional[PoseNode]:
        node = self._inner.add_scan(scan, odom)
        if node is None:
            return None

        self._all_nodes_cache.append(node)
        self._loop_just_closed_flag = False
        candidates = self._find_loop_candidates(node)
        new_loops_added = 0
        for candidate in candidates:
            if self._try_add_loop_edge(node, candidate):
                new_loops_added += 1

        if new_loops_added > 0:
            self._loop_just_closed_flag = True

        return node

    def _find_loop_candidates(self, node: PoseNode) -> list[PoseNode]:
        """ループ候補ノードを返す。

        距離が search_radius 以内かつインデックス差が min_node_gap 以上のノードを対象にする。
        scan データを持たないノードは除外する。
        """
        all_nodes = self._all_nodes_cache
        if not all_nodes:
            return []

        indices = np.array([n.index for n in all_nodes], dtype=np.int64)
        has_scan = np.array([n.scan is not None for n in all_nodes])
        xs = np.array([n.x for n in all_nodes])
        ys = np.array([n.y for n in all_nodes])

        gap_mask = (node.index - indices) >= self._min_node_gap
        dist_mask = np.hypot(node.x - xs, node.y - ys) <= self._search_radius
        combined = gap_mask & has_scan & dist_mask

        return [all_nodes[i] for i in np.where(combined)[0]]

    def _build_candidate_submap(self, candidate: PoseNode) -> np.ndarray:
        """候補ノード周辺の複数スキャンを合成した点群を候補のボディフレームで返す。

        候補ノードから _submap_radius 以内の全ノードのスキャンをワールド座標で合成し、
        候補ノードのボディフレームに変換して返す。_submap_radius が 0 以下の場合は
        候補ノード1枚のスキャンのみを使用する。
        """
        if self._submap_radius <= 0.0:
            return scan_to_points(candidate.scan) if candidate.scan is not None else np.empty((0, 2))

        world_pts_list: list[np.ndarray] = []
        for node in self._all_nodes_cache:
            if node.scan is None:
                continue
            if math.hypot(node.x - candidate.x, node.y - candidate.y) > self._submap_radius:
                continue
            pts = scan_to_points(node.scan)
            if len(pts) == 0:
                continue
            world_pts_list.append(
                points_local_to_world(pts, node.x, node.y, node.yaw)
            )

        if not world_pts_list:
            return scan_to_points(candidate.scan) if candidate.scan is not None else np.empty((0, 2))

        world_pts = np.concatenate(world_pts_list, axis=0)

        # 候補ノードのボディフレームに変換
        return points_world_to_local(world_pts, candidate.x, candidate.y, candidate.yaw)

    def _try_add_loop_edge(self, node: PoseNode, candidate: PoseNode) -> bool:
        """候補ノードとのループ辺を検証して追加する。

        candidate 周辺の合成スキャン点群を src_pts として、node のスキャンに対してマッチングを実行する。
        初期値は pose graph 上の2ノード間の相対ポーズとする。
        streak fallback 設計（設計ルール7に準拠）。

        Returns:
            True: ループ辺が追加された
            False: 未収束またはスキップ
        """
        # 同一 candidate からのループ辺が既に存在する場合はスキップ
        if any(e.from_index == candidate.index for e in self._loop_edges):
            _logger.debug(
                f'Loop edge skipped: candidate {candidate.index} already has '
                f'a loop edge (target node {node.index})'
            )
            return False

        src_pts = self._build_candidate_submap(candidate)
        if len(src_pts) == 0:
            return False

        # 初期値: candidate ボディフレームから見た node の相対ポーズ
        dx_w = node.x - candidate.x
        dy_w = node.y - candidate.y
        dx_local, dy_local = world_delta_to_local(dx_w, dy_w, candidate.yaw)
        initial_guess = OdomData(
            timestamp=node.timestamp,
            x=dx_local,
            y=dy_local,
            yaw=angle_diff(node.yaw, candidate.yaw),
        )

        self.loop_attempt_count += 1
        self._loop_matcher.set_target_cloud(src_pts)
        result = self._loop_matcher.match(
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

        # dyaw バリデーション: ICP/NDT の局所解による false positive を排除する
        abs_dyaw = abs(result.dyaw)
        if abs_dyaw > self._max_loop_dyaw_rad:
            _logger.warning(
                f'Loop edge dyaw check failed (over limit): node {node.index} <- candidate {candidate.index} '
                f'(dyaw={math.degrees(result.dyaw):.1f}deg, limit={math.degrees(self._max_loop_dyaw_rad):.1f}deg)'
            )
            return False
        # パス交差帯域フィルタ: |dyaw| が [crossing_reject, π - crossing_reject] の範囲を拒否する。
        # 同方向ループ（≤reject）と Uターンループ（≥π-reject）のみ許容する。
        if (self._crossing_reject_rad > 0.0
                and self._crossing_reject_rad <= abs_dyaw
                <= math.pi - self._crossing_reject_rad):
            _logger.warning(
                f'Loop edge dyaw check failed (crossing band): node {node.index} <- candidate {candidate.index} '
                f'(dyaw={math.degrees(result.dyaw):.1f}deg, '
                f'forbidden=[{math.degrees(self._crossing_reject_rad):.0f}deg, '
                f'{180.0 - math.degrees(self._crossing_reject_rad):.0f}deg])'
            )
            return False

        # score バリデーション: マッチング局所解（路径交差点等での誤対応）による false positive を排除する
        if self._max_score > 0.0 and result.score > self._max_score:
            _logger.warning(
                f'Loop edge score check failed: node {node.index} <- candidate {candidate.index} '
                f'(score={result.score:.6f}, limit={self._max_score:.6f})'
            )
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
            score=result.score,
        ))
        _logger.info(
            f'Loop edge added: {candidate.index} -> {node.index} '
            f'(dx={result.dx:.3f} dy={result.dy:.3f} '
            f'dyaw={math.degrees(result.dyaw):.1f}deg score={result.score:.6f})'
        )
        return True

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
        self._all_nodes_cache.clear()

    def get_loop_edges(self) -> list[PoseEdge]:
        return list(self._loop_edges)

    @property
    def loop_just_closed(self) -> bool:
        result = self._loop_just_closed_flag
        self._loop_just_closed_flag = False
        return result
