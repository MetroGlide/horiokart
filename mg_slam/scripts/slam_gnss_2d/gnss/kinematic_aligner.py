from __future__ import annotations

import bisect
import math
from typing import Optional

from .aligner_base import GnssAlignerBase
from ..data_types import GnssData, PoseNode


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


class KinematicHeadingAligner(GnssAlignerBase):
    """ロボットの運動ベクトルから GNSS↔SLAM 座標系の変換を推定する整合実装。

    ロボットが一定速度以上で移動した区間の GNSS 変位ベクトルと
    ポーズグラフの移動ベクトルを照合し、座標系の回転オフセットを求める。
    これにより「地球の北」と「SLAM の X 軸」のずれを手作業なしで自動推定できる。

    Args:
        min_speed_ms: この速度未満の区間は回転推定サンプルから除外する [m/s]。
                      低速区間は GPS ノイズの影響が相対的に大きく方位推定に不向き。
    """

    def __init__(self, min_speed_ms: float = 0.5) -> None:
        self._min_speed_ms = min_speed_ms

    def estimate_transform(
        self,
        nodes: list[PoseNode],
        gnss_list: list[GnssData],
    ) -> tuple[float, float, float]:
        """GNSS → SLAM 座標系への変換 (tx, ty, rotation_rad) を推定して返す。

        SLAM_xy = R(rotation_rad) * GNSS_xy + (tx, ty) となる変換を求める。

        Returns:
            (tx, ty, rotation_rad): 平行移動量 [m] と回転量 [rad]
        """
        if not nodes or not gnss_list:
            return 0.0, 0.0, 0.0

        node_timestamps = [n.timestamp for n in nodes]

        # 連続する GNSS ペアから回転サンプルを収集する
        sin_sum = 0.0
        cos_sum = 0.0
        for i in range(len(gnss_list) - 1):
            g0, g1 = gnss_list[i], gnss_list[i + 1]
            dt = g1.timestamp - g0.timestamp
            if dt < 1e-6:
                continue

            dg_x = g1.x - g0.x
            dg_y = g1.y - g0.y
            speed = math.hypot(dg_x, dg_y) / dt
            if speed < self._min_speed_ms:
                continue

            n0 = _nearest_node(nodes, node_timestamps, g0.timestamp)
            n1 = _nearest_node(nodes, node_timestamps, g1.timestamp)
            if n0 is None or n1 is None:
                continue
            ds_x = n1.x - n0.x
            ds_y = n1.y - n0.y
            if math.hypot(ds_x, ds_y) < 0.01:
                continue

            # SLAM_xy = R * GNSS_xy + t なので heading_slam = heading_gnss + rotation_rad
            heading_gnss = math.atan2(dg_y, dg_x)
            heading_slam = math.atan2(ds_y, ds_x)
            rot = heading_slam - heading_gnss
            sin_sum += math.sin(rot)
            cos_sum += math.cos(rot)

        if sin_sum == 0.0 and cos_sum == 0.0:
            return 0.0, 0.0, 0.0

        rotation_rad = math.atan2(sin_sum, cos_sum)
        cos_r = math.cos(rotation_rad)
        sin_r = math.sin(rotation_rad)

        # 全 GNSS 測位と最近傍ノードのペアから平行移動量を推定する
        # t = SLAM_xy - R * GNSS_xy の平均
        tx_sum = 0.0
        ty_sum = 0.0
        count = 0
        for gnss in gnss_list:
            node = _nearest_node(nodes, node_timestamps, gnss.timestamp)
            if node is None:
                continue
            rx = cos_r * gnss.x - sin_r * gnss.y
            ry = sin_r * gnss.x + cos_r * gnss.y
            tx_sum += node.x - rx
            ty_sum += node.y - ry
            count += 1

        if count == 0:
            return 0.0, 0.0, rotation_rad

        return tx_sum / count, ty_sum / count, rotation_rad
