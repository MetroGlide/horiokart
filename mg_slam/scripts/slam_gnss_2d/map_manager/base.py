from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from ..data_types import PoseNode


class MapRendererBase(ABC):
    """占有格子マップを生成・管理する抽象基底クラス。

    OpenCV ray-casting・他実装に差し替え可能。
    """

    @abstractmethod
    def add_node(self, node: PoseNode) -> None:
        """新しいポーズノードを受け取り、インクリメンタルにマップを更新する（オンライン用）。"""
        raise NotImplementedError

    @abstractmethod
    def rerender_all(self, nodes: list[PoseNode]) -> None:
        """全ノードからマップを再描画する（グラフ最適化後のバッチ更新用）。"""
        raise NotImplementedError

    @abstractmethod
    def to_occupancy_array(self) -> tuple[np.ndarray, float, float, float]:
        """ROS OccupancyGrid に変換可能な形式でマップデータを返す。

        Returns:
            (data, origin_x, origin_y, resolution)
            data: int8 の 2D 配列、値は -1(unknown) / 0(free) / 100(occupied)
        """
        raise NotImplementedError
