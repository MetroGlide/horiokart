from __future__ import annotations

from ..data_types import GnssData, PoseNode


class GnssConstraintInserter:
    """GNSS 拘束（PriorFactor）をポーズグラフに挿入するクラス。Phase 4 で実装予定。

    GnssAlignerBase で推定した変換で全 GNSS 座標を SLAM 座標系に変換し、
    GTSAM の GPSFactor2D または独自 Factor としてグラフへ一括投入する。
    """

    def insert(
        self,
        graph,
        nodes: list[PoseNode],
        gnss_list: list[GnssData],
        transform: tuple[float, float, float],
    ) -> None:
        raise NotImplementedError("GnssConstraintInserter は Phase 4 で実装予定です")
