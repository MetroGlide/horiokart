from __future__ import annotations

import math
from collections import deque
from typing import Optional

import numpy as np

from ...data_types import PoseNode, ScanData
from .base import ReferenceProviderBase


def _scan_to_points(scan: ScanData) -> np.ndarray:
    """有効レンジのみを2D点群 (N, 2) に変換する。"""
    n = len(scan.ranges)
    angles = scan.angle_min + np.arange(n) * scan.angle_increment
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    valid = (ranges >= scan.range_min) & (ranges <= scan.range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


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
        self._nodes: deque[PoseNode] = deque(maxlen=window)
        self._last_node: Optional[PoseNode] = None

    def update(self, node: PoseNode) -> None:
        if node.scan is not None:
            self._nodes.append(node)
        self._last_node = node

    def get_reference_pts(self) -> Optional[np.ndarray]:
        if not self._nodes or self._last_node is None:
            return None

        last = self._last_node

        # 各ノードのスキャンを世界フレームに変換して蓄積
        world_pts_list = []
        for n in self._nodes:
            if n.scan is None:
                continue
            local_pts = _scan_to_points(n.scan)
            c, s = math.cos(n.yaw), math.sin(n.yaw)
            R = np.array([[c, -s], [s, c]])
            world_pts_list.append((R @ local_pts.T).T + np.array([n.x, n.y]))

        if not world_pts_list:
            return None

        world_pts = np.concatenate(world_pts_list, axis=0)

        # 最後ノードの位置から radius 以内の点のみ残す
        d_sq = np.sum((world_pts - np.array([last.x, last.y])) ** 2, axis=1)
        world_pts = world_pts[d_sq <= self._radius ** 2]

        if len(world_pts) == 0:
            return None

        # 最後ノードのボディフレームに変換
        c, s = math.cos(last.yaw), math.sin(last.yaw)
        R_inv = np.array([[c, s], [-s, c]])  # R^T (回転行列の逆)
        return (R_inv @ (world_pts - np.array([last.x, last.y])).T).T
