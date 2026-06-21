from __future__ import annotations

from typing import Optional

import numpy as np

from slam_gnss_2d.core.data_types import PoseNode, ScanData
from slam_gnss_2d.scan_matching.reference_provider.base import ReferenceProviderBase


def _scan_to_points(scan: ScanData) -> np.ndarray:
    """有効レンジのみを2D点群 (N, 2) に変換する。"""
    n = len(scan.ranges)
    angles = scan.angle_min + np.arange(n) * scan.angle_increment
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    valid = (ranges >= scan.range_min) & (ranges <= scan.range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


class ScanToScanProvider(ReferenceProviderBase):
    """直前の成功ノードのスキャン1枚を参照点群として供給する。"""

    def __init__(self) -> None:
        self._last_pts: Optional[np.ndarray] = None

    def update(self, node: PoseNode) -> None:
        if node.scan is not None:
            self._last_pts = _scan_to_points(node.scan)

    def get_reference_pts(self) -> Optional[np.ndarray]:
        return self._last_pts
