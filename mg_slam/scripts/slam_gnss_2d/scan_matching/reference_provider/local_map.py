from __future__ import annotations
from collections import deque
from typing import Optional

import numpy as np

from slam_gnss_2d.core.data_types import PoseNode, ScanData
from slam_gnss_2d.core.geometry import (
    points_local_to_world,
    points_world_to_local,
    scan_to_points,
)
from slam_gnss_2d.scan_matching.reference_provider.base import ReferenceProviderBase


class LocalMapProvider(ReferenceProviderBase):
    """直近 window ノードのスキャンを世界フレームで蓄積し、
    ローカルマップ点群として供給する。

    get_reference_pts() は最後ノードのボディフレームで点群を返す。
    抽出半径 radius [m] 以内の点のみ返すことで、マッチングに無関係な
    遠方の点を除外する。
    """

    def __init__(self, window: int, radius: float) -> None:
        self._window = window
        self._radius = radius
        self._nodes: deque[tuple[PoseNode, np.ndarray]] = deque(maxlen=window)
        self._last_node: Optional[PoseNode] = None

    def update(self, node: PoseNode) -> None:
        if node.scan is not None:
            self._nodes.append((node, scan_to_points(node.scan)))
        self._last_node = node

    def invalidate_cache(self) -> None:
        """グラフ最適化後に呼び、古いノード位置に基づくローカルマップキャッシュを無効化する。

        replace_nodes() でノード位置が更新された後、次フレームのスキャンマッチングで
        無効なローカルマップが参照されることを防ぐ。
        """
        self._nodes.clear()

    def get_reference_pts(self) -> Optional[np.ndarray]:
        if not self._nodes or self._last_node is None:
            return None

        last = self._last_node
        entries = list(self._nodes)

        yaws = np.array([n.yaw for n, _ in entries])

        world_pts_list = []
        for i, (n, local_pts) in enumerate(entries):
            world_pts_list.append(
                points_local_to_world(local_pts, n.x, n.y, float(yaws[i]))
            )

        if not world_pts_list:
            return None

        world_pts = np.concatenate(world_pts_list, axis=0)

        # 最後ノードの位置から radius 以内の点のみ残す
        d_sq = np.sum((world_pts - np.array([last.x, last.y])) ** 2, axis=1)
        world_pts = world_pts[d_sq <= self._radius ** 2]

        if len(world_pts) == 0:
            return None

        # 最後ノードのボディフレームに変換
        return points_world_to_local(world_pts, last.x, last.y, last.yaw)
