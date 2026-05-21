from __future__ import annotations

from abc import ABC, abstractmethod

from ..data_types import PoseNode


class GraphOptimizerBase(ABC):
    """ポーズグラフ最適化の抽象基底クラス。GTSAM 等に差し替え可能。"""

    @abstractmethod
    def optimize(self, nodes: list[PoseNode]) -> list[PoseNode]:
        """グラフ最適化を実行し、更新された PoseNode のリストを返す。

        ノードの順序・インデックスは入力と同一であること。
        """
        raise NotImplementedError
