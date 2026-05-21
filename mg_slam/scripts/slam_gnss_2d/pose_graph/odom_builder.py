from __future__ import annotations

import math
from typing import Optional

from .base import PoseGraphBuilderBase
from ..data_types import OdomData, PoseNode, ScanData


def _angle_diff(a: float, b: float) -> float:
    """2つの角度の差を [-pi, pi] に正規化して返す。"""
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


class OdomOnlyBuilder(PoseGraphBuilderBase):
    """オドメトリのみを使用してポーズグラフを構築する Phase 1 実装。

    スキャンマッチングなしの最もシンプルな構成。
    OdomSource の topic 引数を '/odom/gnss' に変更するだけで
    GNSS補正オドメトリベースの構築にも対応できる。
    """

    def __init__(
        self,
        min_translation: float = 0.3,
        min_rotation: float = 0.1,
    ) -> None:
        """
        Args:
            min_translation: ノード追加の最小移動距離 [m]
            min_rotation: ノード追加の最小回転量 [rad]
        """
        self._min_translation = min_translation
        self._min_rotation = min_rotation
        self._nodes: list[PoseNode] = []
        self._last_odom: Optional[OdomData] = None

    def add_scan(self, scan: ScanData, odom: OdomData) -> Optional[PoseNode]:
        if self._last_odom is not None:
            dx = odom.x - self._last_odom.x
            dy = odom.y - self._last_odom.y
            dist = math.hypot(dx, dy)
            dyaw = abs(_angle_diff(odom.yaw, self._last_odom.yaw))
            if dist < self._min_translation and dyaw < self._min_rotation:
                return None

        node = PoseNode(
            index=len(self._nodes),
            timestamp=scan.timestamp,
            x=odom.x,
            y=odom.y,
            yaw=odom.yaw,
            scan=scan,
        )
        self._nodes.append(node)
        self._last_odom = odom
        return node

    def get_nodes(self) -> list[PoseNode]:
        return list(self._nodes)

    def reset(self) -> None:
        self._nodes.clear()
        self._last_odom = None
