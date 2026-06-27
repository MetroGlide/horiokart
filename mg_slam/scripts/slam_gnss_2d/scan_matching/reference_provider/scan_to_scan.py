from __future__ import annotations

from typing import Optional

import numpy as np

from slam_gnss_2d.core.data_types import PoseNode, ScanData
from slam_gnss_2d.core.geometry import scan_to_points
from slam_gnss_2d.scan_matching.reference_provider.base import ReferenceProviderBase


class ScanToScanProvider(ReferenceProviderBase):
    """直前の成功ノードのスキャン1枚を参照点群として供給する。"""

    def __init__(self) -> None:
        self._last_pts: Optional[np.ndarray] = None

    def update(self, node: PoseNode) -> None:
        if node.scan is not None:
            self._last_pts = scan_to_points(node.scan)

    def get_reference_pts(self) -> Optional[np.ndarray]:
        return self._last_pts
