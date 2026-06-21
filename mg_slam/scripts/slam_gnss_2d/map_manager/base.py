from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from slam_gnss_2d.core.data_types import PoseNode


class MapRendererBase(ABC):
    """占有格子マップを生成・管理する抽象基底クラス。

    OpenCV ray-casting・他実装に差し替え可能。
    """

    @abstractmethod
    def add_node(self, node: PoseNode) -> bool:
        """新しいポーズノードを受け取り、インクリメンタルにマップを更新する（オンライン用）。

        Returns:
            True: レンダリング成功
            False: ロボット位置がマップ範囲外（呼び出し元は rerender_all() を呼ぶこと）
        """
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

    @abstractmethod
    def apply_trajectory_mask(self, nodes: list[PoseNode], radius_m: float, filter_type: str = 'clear') -> None:
        """ポーズグラフの軌跡周辺に対してノイズ除去処理を適用する。

        Args:
            nodes: ポーズグラフの全ノード
            radius_m: 軌跡からノイズ除去を適用する半径 [m]
            filter_type: 'clear' の場合は強制的に空き(Free)にする。'attenuate'の場合はヒットカウントを減衰させるなど。
        """
        raise NotImplementedError
