from __future__ import annotations

import math
from typing import Optional

import numpy as np

from slam_gnss_2d.pose_graph.base import PoseGraphBuilderBase
from slam_gnss_2d.core.data_types import OdomData, PoseEdge, PoseNode, ScanData
from slam_gnss_2d.core.geometry import angle_diff

_ODOM_INFORMATION = np.diag([100.0, 100.0, 50.0])


class OdomOnlyBuilder(PoseGraphBuilderBase):
    """オドメトリのみを使用してポーズグラフを構築する Phase 1 実装。

    スキャンマッチングなしの最もシンプルな構成。
    OdomSource の topic 引数を '/odom/gnss' に変更するだけで
    GNSS補正オドメトリベースの構築にも対応できる。
    """

    def __init__(
        self,
        min_translation: float,
        min_rotation: float,
    ) -> None:
        """
        Args:
            min_translation: ノード追加の最小移動距離 [m]
            min_rotation: ノード追加の最小回転量 [rad]
        """
        self._min_translation = min_translation
        self._min_rotation = min_rotation
        self._nodes: list[PoseNode] = []
        self._edges: list[PoseEdge] = []
        self._last_odom: Optional[OdomData] = None

    def add_scan(self, scan: ScanData, odom: OdomData) -> Optional[PoseNode]:
        if self._last_odom is not None:
            dx = odom.x - self._last_odom.x
            dy = odom.y - self._last_odom.y
            dist = math.hypot(dx, dy)
            dyaw = abs(angle_diff(odom.yaw, self._last_odom.yaw))
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

        if len(self._nodes) > 1:
            prev = self._nodes[-2]
            c = math.cos(-prev.yaw)
            s = math.sin(-prev.yaw)
            dx_w = node.x - prev.x
            dy_w = node.y - prev.y
            self._edges.append(PoseEdge(
                from_index=prev.index,
                to_index=node.index,
                dx=c * dx_w - s * dy_w,
                dy=s * dx_w + c * dy_w,
                dyaw=angle_diff(node.yaw, prev.yaw),
                information=_ODOM_INFORMATION.copy(),
            ))

        return node

    def get_nodes(self) -> list[PoseNode]:
        return list(self._nodes)

    def get_edges(self) -> list[PoseEdge]:
        return list(self._edges)

    def reset(self) -> None:
        self._nodes.clear()
        self._edges.clear()
        self._last_odom = None

    @property
    def loop_just_closed(self) -> bool:
        return False
