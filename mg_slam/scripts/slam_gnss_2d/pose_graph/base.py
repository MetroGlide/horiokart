from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from slam_gnss_2d.core.data_types import OdomData, PoseEdge, PoseNode, ScanData


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
    def get_edges(self) -> list[PoseEdge]:
        """ノード間の拘束辺をすべて返す。Phase 3 の GTSAMOptimizer が使用する。"""
        raise NotImplementedError

    @abstractmethod
    def reset(self) -> None:
        """グラフをリセットする。"""
        raise NotImplementedError

    @property
    @abstractmethod
    def loop_just_closed(self) -> bool:
        """今フレームでループ閉合最適化が実行された場合 True を返し、次呼び出しで False にリセットされる。

        slam_node.py がこのフラグを確認し、True のとき rerender_all() をトリガーする。
        LoopClosureBuilder のみが True を返す。他の実装は常に False を返す。
        """
        raise NotImplementedError
