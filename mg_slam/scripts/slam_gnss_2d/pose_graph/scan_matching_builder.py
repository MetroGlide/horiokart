from __future__ import annotations

import logging
import math
from typing import Optional

import numpy as np

from .base import PoseGraphBuilderBase
from ..data_types import OdomData, PoseEdge, PoseNode, ScanData
from ..scan_matching.base import ScanMatcherBase
from ..scan_matching.reference_provider.base import ReferenceProviderBase

_logger = logging.getLogger(__name__)

_ODOM_INFORMATION = np.diag([100.0, 100.0, 50.0])
# streak 超過時の odom フォールバックに使用する低信頼度情報行列
_ODOM_FALLBACK_INFORMATION = np.diag([10.0, 10.0, 5.0])


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ScanMatchingBuilder(PoseGraphBuilderBase):
    """スキャンマッチング補正を加えたポーズグラフ構築実装。

    オドメトリを初期値として ScanMatcherBase 実装（ICP 等）で補正し、
    より高精度な相対移動量をノードに記録する。
    converged=False のフォールバック時はオドメトリ値を使用する。
    """

    def __init__(
        self,
        matcher: ScanMatcherBase,
        provider: ReferenceProviderBase,
        min_translation: float = 0.3,
        min_rotation: float = 0.1,
        max_failure_streak: int = 5,
    ) -> None:
        self._matcher = matcher
        self._provider = provider
        self._min_translation = min_translation
        self._min_rotation = min_rotation
        self._max_failure_streak = max_failure_streak
        self._nodes: list[PoseNode] = []
        self._last_odom: Optional[OdomData] = None
        self._edges: list[PoseEdge] = []
        self._failure_streak: int = 0
        self.icp_attempt_count: int = 0
        self.icp_success_count: int = 0
        self.odom_fallback_count: int = 0

    def add_scan(self, scan: ScanData, odom: OdomData) -> Optional[PoseNode]:
        if not self._nodes:
            node = PoseNode(
                index=0,
                timestamp=scan.timestamp,
                x=odom.x,
                y=odom.y,
                yaw=odom.yaw,
                scan=scan,
            )
            self._nodes.append(node)
            self._provider.update(node)
            self._last_odom = odom
            return node

        # キーフレーム判定
        dx_w = odom.x - self._last_odom.x
        dy_w = odom.y - self._last_odom.y
        dist = math.hypot(dx_w, dy_w)
        dyaw = abs(_angle_diff(odom.yaw, self._last_odom.yaw))
        if dist < self._min_translation and dyaw < self._min_rotation:
            return None

        prev_node = self._nodes[-1]

        # odom デルタをロボット体軸フレーム（prev_node 基準）に変換する。
        # dx_w/dy_w は odom ワールドフレームのベクトルなので、
        # odom フレームでの prev_node の方向角 (_last_odom.yaw) で回転する。
        # prev_node.yaw (SLAM フレーム) を使うと ICP 補正量がずれ込み初期値が劣化する。
        c = math.cos(-self._last_odom.yaw)
        s = math.sin(-self._last_odom.yaw)
        dx_local = c * dx_w - s * dy_w
        dy_local = s * dx_w + c * dy_w
        dyaw_delta = _angle_diff(odom.yaw, self._last_odom.yaw)

        initial_guess = OdomData(
            timestamp=scan.timestamp,
            x=dx_local,
            y=dy_local,
            yaw=dyaw_delta,
        )

        # スキャンマッチング
        src_pts = self._provider.get_reference_pts()
        if src_pts is not None:
            self.icp_attempt_count += 1
            result = self._matcher.match(
                src_pts=src_pts,
                dst=scan,
                initial_guess=initial_guess,
            )
            if not result.converged:
                self._failure_streak += 1
                _logger.warning(
                    f'Matcher did not converge at node {len(self._nodes)} '
                    f'(init dx={initial_guess.x:.3f}m dy={initial_guess.y:.3f}m '
                    f'dyaw={math.degrees(initial_guess.yaw):.1f}deg, '
                    f'streak={self._failure_streak}/{self._max_failure_streak})'
                )
                if self._failure_streak < self._max_failure_streak:
                    return None
                _logger.warning(
                    f'Failure streak limit reached at node {len(self._nodes)}; '
                    'falling back to odometry'
                )
                dx_icp, dy_icp, dyaw_icp = dx_local, dy_local, dyaw_delta
                edge_info = _ODOM_FALLBACK_INFORMATION.copy()
                self._failure_streak = 0
                self.odom_fallback_count += 1
            else:
                self._failure_streak = 0
                self.icp_success_count += 1
                dx_icp, dy_icp, dyaw_icp = result.dx, result.dy, result.dyaw
                edge_info = result.information
        else:
            dx_icp, dy_icp, dyaw_icp = dx_local, dy_local, dyaw_delta
            edge_info = _ODOM_INFORMATION.copy()

        # ICP 結果（prev_node ローカルフレーム）をワールド座標に変換して絶対ポーズを計算
        c_p = math.cos(prev_node.yaw)
        s_p = math.sin(prev_node.yaw)
        new_x = prev_node.x + c_p * dx_icp - s_p * dy_icp
        new_y = prev_node.y + s_p * dx_icp + c_p * dy_icp
        new_yaw = prev_node.yaw + dyaw_icp
        while new_yaw > math.pi:
            new_yaw -= 2.0 * math.pi
        while new_yaw < -math.pi:
            new_yaw += 2.0 * math.pi

        node = PoseNode(
            index=len(self._nodes),
            timestamp=scan.timestamp,
            x=new_x,
            y=new_y,
            yaw=new_yaw,
            scan=scan,
        )
        self._nodes.append(node)
        self._provider.update(node)
        self._edges.append(PoseEdge(
            from_index=prev_node.index,
            to_index=node.index,
            dx=dx_icp,
            dy=dy_icp,
            dyaw=dyaw_icp,
            information=edge_info,
        ))
        self._last_odom = odom
        return node

    def get_nodes(self) -> list[PoseNode]:
        return list(self._nodes)

    def get_edges(self) -> list[PoseEdge]:
        return list(self._edges)

    def reset(self) -> None:
        self._nodes.clear()
        self._edges.clear()
        self._last_odom = None
        self._failure_streak = 0

    @property
    def loop_just_closed(self) -> bool:
        return False

    def replace_nodes(self, nodes: list[PoseNode]) -> None:
        """最適化後のノードリストで内部ノードリストを置き換える。

        LoopClosureBuilder が GTSAMOptimizer.optimize() 呼び出し後に使用する。
        ノードの index・timestamp・scan は保持されていること。
        """
        self._nodes = list(nodes)
        self._provider.invalidate_cache()
