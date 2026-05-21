from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from ..data_types import OdomData, PoseNode, ScanData


class PoseGraphBuilderBase(ABC):
    """ポーズグラフを構築するコンポーネントの抽象基底クラス。

    OdomOnly / ScanMatching / LoopClosure / SlamToolboxAdapter 等に差し替え可能。
    """

    @abstractmethod
    def add_scan(self, scan: ScanData, odom: OdomData) -> Optional[PoseNode]:
        """新しいスキャンを追加する。

        移動量が閾値未満でノードをスキップした場合は None を返す。
        新しいノードを作成した場合は PoseNode を返す。
        """
        raise NotImplementedError

    @abstractmethod
    def get_nodes(self) -> list[PoseNode]:
        """現在のすべてのノードを時系列順に返す。"""
        raise NotImplementedError

    @abstractmethod
    def reset(self) -> None:
        """グラフをリセットする。"""
        raise NotImplementedError
