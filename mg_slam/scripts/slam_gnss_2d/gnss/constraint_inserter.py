from __future__ import annotations

import bisect
import math
from typing import Optional

import numpy as np

from ..data_types import GnssData, GnssPrior, PoseNode


def _nearest_node(
    nodes: list[PoseNode],
    timestamps: list[float],
    query_ts: float,
) -> Optional[PoseNode]:
    if not nodes:
        return None
    idx = bisect.bisect_left(timestamps, query_ts)
    if idx == 0:
        return nodes[0]
    if idx >= len(nodes):
        return nodes[-1]
    prev, next_ = nodes[idx - 1], nodes[idx]
    return prev if abs(prev.timestamp - query_ts) <= abs(next_.timestamp - query_ts) else next_


class GnssConstraintInserter:
    """GNSS座標から GnssPrior リストを生成するクラス。

    GnssAlignerBase で推定した変換で全 GNSS 座標を SLAM 座標系に変換し、
    各 GNSS 測位に最近傍の PoseNode に対する GnssPrior を返す。
    GTSAM への依存は持たない。optimizer 層が GnssPrior を PriorFactorPose2 に変換する。
    """

    def __init__(self, default_noise_xy_m: float = 3.0) -> None:
        self._default_noise_xy_m = default_noise_xy_m

    def build_priors(
        self,
        nodes: list[PoseNode],
        gnss_list: list[GnssData],
        transform: tuple[float, float, float],
    ) -> list[GnssPrior]:
        """GNSS座標を SLAM 座標系に変換し、各 GNSS 測位に対応する GnssPrior のリストを返す。

        Args:
            nodes:     ポーズグラフの全ノード（時系列順）
            gnss_list: GNSS 測位の全データ（時系列順）
            transform: GnssAlignerBase.estimate_transform() が返した (tx, ty, rotation_rad)

        Returns:
            list[GnssPrior]: 最適化層へ渡す GNSS 拘束リスト
        """
        if not nodes or not gnss_list:
            return []

        tx, ty, rotation_rad = transform
        cos_r = math.cos(rotation_rad)
        sin_r = math.sin(rotation_rad)
        R = np.array([[cos_r, -sin_r], [sin_r, cos_r]])
        default_info = np.eye(2) * (1.0 / self._default_noise_xy_m ** 2)

        node_timestamps = [n.timestamp for n in nodes]

        priors: list[GnssPrior] = []
        for gnss in gnss_list:
            x_slam = cos_r * gnss.x - sin_r * gnss.y + tx
            y_slam = sin_r * gnss.x + cos_r * gnss.y + ty

            # 共分散行列を SLAM 座標系に回転変換し情報行列を算出する
            cov_slam = R @ gnss.covariance @ R.T
            det = cov_slam[0, 0] * cov_slam[1, 1] - \
                cov_slam[0, 1] * cov_slam[1, 0]
            information = default_info if det < 1e-9 else np.linalg.inv(
                cov_slam)

            node = _nearest_node(nodes, node_timestamps, gnss.timestamp)
            if node is None:
                continue

            priors.append(GnssPrior(
                node_index=node.index,
                x=x_slam,
                y=y_slam,
                information=information,
            ))

        return priors
