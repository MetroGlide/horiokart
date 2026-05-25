from __future__ import annotations

from abc import ABC, abstractmethod

from ..data_types import PoseEdge, PoseNode


class GraphOptimizerBase(ABC):
    """ポーズグラフ最適化の抽象基底クラス。GTSAM 等に差し替え可能。"""

    @abstractmethod
    def optimize(
        self,
        nodes: list[PoseNode],
        edges: list[PoseEdge],
    ) -> list[PoseNode]:
        """グラフ最適化を実行し、更新された PoseNode のリストを返す。

        ノードの順序・インデックスは入力と同一であること。
        edges: 連続辺・ループ辺を含むポーズグラフの全拘束。GTSAM BetweenFactor として使用する。
        """
        raise NotImplementedError
