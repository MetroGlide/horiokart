from __future__ import annotations

from abc import ABC, abstractmethod

from ..data_types import GnssData, PoseNode


class GnssAlignerBase(ABC):
    """GNSS 軌跡とポーズグラフの座標系を整合させる抽象基底クラス。

    GNSSなしで作成したポーズグラフに対して、GNSS 軌跡との変換
    （平行移動 + 回転）を推定する。
    推定された変換で GNSS 座標を SLAM 座標系に変換してから
    GNSS 拘束を投入することで手作業なしのグローバル整合が可能になる。
    """

    @abstractmethod
    def estimate_transform(
        self,
        nodes: list[PoseNode],
        gnss_list: list[GnssData],
    ) -> tuple[float, float, float]:
        """GNSS → SLAM 座標系への変換 (tx, ty, rotation_rad) を推定して返す。

        Args:
            nodes: ポーズグラフの全ノード（時系列順）
            gnss_list: GNSS 測位の全データ（時系列順）

        Returns:
            (tx, ty, rotation_rad): 平行移動量と回転量
        """
        raise NotImplementedError
