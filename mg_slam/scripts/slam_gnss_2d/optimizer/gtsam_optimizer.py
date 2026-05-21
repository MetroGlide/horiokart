from __future__ import annotations

from .base import GraphOptimizerBase
from ..data_types import PoseNode


class GTSAMOptimizer(GraphOptimizerBase):
    """GTSAM LevenbergMarquardt による 2D ポーズグラフ最適化。Phase 3 で実装予定。"""

    def optimize(self, nodes: list[PoseNode]) -> list[PoseNode]:
        raise NotImplementedError("GTSAMOptimizer は Phase 3 で実装予定です")
