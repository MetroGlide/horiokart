from __future__ import annotations

from .aligner_base import GnssAlignerBase
from ..data_types import GnssData, PoseNode


class KinematicHeadingAligner(GnssAlignerBase):
    """ロボットの運動ベクトルから初期方位を推定する GNSS 整合実装。Phase 4 で実装予定。

    ロボットが一定速度以上で移動した区間の GNSS 変位ベクトルと
    ポーズグラフの移動ベクトルを照合し、座標系の回転オフセットを求める。
    これにより「地球の北」と「SLAM の X 軸」のずれを手作業なしで自動推定できる。
    """

    def estimate_transform(
        self,
        nodes: list[PoseNode],
        gnss_list: list[GnssData],
    ) -> tuple[float, float, float]:
        raise NotImplementedError("KinematicHeadingAligner は Phase 4 で実装予定です")
