from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Sequence

from ..data_types import GnssPrior, PoseEdge, PoseNode


class GraphOptimizerBase(ABC):
    """ポーズグラフ最適化の抽象基底クラス。GTSAM 等に差し替え可能。"""

    @abstractmethod
    def optimize(
        self,
        nodes: list[PoseNode],
        edges: list[PoseEdge],
        gnss_priors: Sequence[GnssPrior] = (),
    ) -> list[PoseNode]:
        """グラフ最適化を実行し、更新された PoseNode のリストを返す。

        ノードの順序・インデックスは入力と同一であること。
        edges: 連続辺・ループ辺を含むポーズグラフの全拘束。GTSAM BetweenFactor として使用する。
        gnss_priors: GNSS 絶対位置拘束。空の場合は GNSS なしの最適化（Phase 1〜3 相当）。
        """
        raise NotImplementedError
