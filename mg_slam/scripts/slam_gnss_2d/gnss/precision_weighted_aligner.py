from __future__ import annotations

import bisect
import logging
import math
from typing import Optional

import numpy as np

from .aligner_base import GnssAlignerBase
from ..data_types import GnssData, PoseNode

_logger = logging.getLogger(__name__)

_MIN_RELIABLE_SAMPLES = 5


def _nearest_node(
    nodes: list[PoseNode],
    timestamps: list[float],
    query_ts: float,
    max_time_delta_s: float = float('inf'),
) -> Optional[PoseNode]:
    if not nodes:
        return None
    idx = bisect.bisect_left(timestamps, query_ts)
    if idx == 0:
        nearest = nodes[0]
    elif idx >= len(nodes):
        nearest = nodes[-1]
    else:
        prev, next_ = nodes[idx - 1], nodes[idx]
        nearest = prev if abs(prev.timestamp - query_ts) <= abs(next_.timestamp - query_ts) else next_
    if abs(nearest.timestamp - query_ts) > max_time_delta_s:
        return None
    return nearest


def _covariance_to_scalar_noise(cov: np.ndarray, default_noise_m: float) -> float:
    """2x2 共分散行列からスカラーノイズ量 [m] を返す。

    行列式が不正な場合はフォールバック値を返す。
    """
    det = cov[0, 0] * cov[1, 1] - cov[0, 1] * cov[1, 0]
    if det < 1e-9:
        return default_noise_m
    return math.sqrt(math.sqrt(det))


class PrecisionWeightedAligner(GnssAlignerBase):
    """GNSS 精度（共分散）を重みとして使用する加重アライナー。

    GNSS を正として SLAM 座標系をフィットさせる 2 段階アルゴリズム:

    Stage 1 — 回転推定:
        連続 GNSS ペアの変位ベクトルと対応 SLAM ノード間の変位ベクトルを照合し、
        座標系間の回転を推定する。
        重み = GNSS 変位量 / GNSS ノイズ（大きく動いて精度が高い区間を優先）。

    Stage 2 — 平行移動推定:
        推定回転で GNSS 座標を SLAM 座標系に変換し、各測位で残差を算出する。
        重み = GNSS 情報量のトレース（精度の高い測位ほど強く引く）。
        これにより GNSS を正とした加重最小二乗フィッティングを実現する。

    GNSS は 1 機のため方位は単独では定まらない（連続変位ベクトルで解決するため
    2 段階構造を維持する）。

    Args:
        min_speed_ms: この速度未満の GNSS 区間は回転推定から除外する [m/s]。
        default_noise_xy_m: GNSS の covariance が不定なときのフォールバックノイズ [m]。
        max_time_delta_s: GNSS 測位とノードのタイムスタンプ差の上限 [s]。
    """

    def __init__(
        self,
        min_speed_ms: float = 0.5,
        default_noise_xy_m: float = 3.0,
        max_time_delta_s: float = 5.0,
    ) -> None:
        self._min_speed_ms = min_speed_ms
        self._default_noise_xy_m = default_noise_xy_m
        self._max_time_delta_s = max_time_delta_s

    def estimate_transform(
        self,
        nodes: list[PoseNode],
        gnss_list: list[GnssData],
    ) -> tuple[float, float, float]:
        """GNSS → SLAM 座標系への変換 (tx, ty, rotation_rad) を推定して返す。

        SLAM_xy = R(rotation_rad) * GNSS_xy + (tx, ty) となる変換を求める。
        GNSS 精度（共分散逆行列）を各サンプルの重みとして使用する。

        Returns:
            (tx, ty, rotation_rad): 平行移動量 [m] と回転量 [rad]
        """
        if not nodes or not gnss_list:
            return 0.0, 0.0, 0.0

        node_timestamps = [n.timestamp for n in nodes]
        rotation_rad = self._estimate_rotation(nodes, node_timestamps, gnss_list)
        tx, ty = self._estimate_translation(nodes, node_timestamps, gnss_list, rotation_rad)
        return tx, ty, rotation_rad

    def _estimate_rotation(
        self,
        nodes: list[PoseNode],
        node_timestamps: list[float],
        gnss_list: list[GnssData],
    ) -> float:
        """GNSS変位ベクトルとSLAM変位ベクトルの照合により回転を推定する。

        重み = GNSS変位量 / GNSSノイズ: 大きく動いていて精度が高い区間を優先する。
        """
        sin_sum = 0.0
        cos_sum = 0.0
        sample_count = 0

        for i in range(len(gnss_list) - 1):
            g0, g1 = gnss_list[i], gnss_list[i + 1]
            dt = g1.timestamp - g0.timestamp
            if dt < 1e-6:
                continue

            dg_x = g1.x - g0.x
            dg_y = g1.y - g0.y
            displacement = math.hypot(dg_x, dg_y)
            speed = displacement / dt
            if speed < self._min_speed_ms:
                continue

            n0 = _nearest_node(nodes, node_timestamps, g0.timestamp, self._max_time_delta_s)
            n1 = _nearest_node(nodes, node_timestamps, g1.timestamp, self._max_time_delta_s)
            if n0 is None or n1 is None or n0.index == n1.index:
                continue
            ds_x = n1.x - n0.x
            ds_y = n1.y - n0.y
            if math.hypot(ds_x, ds_y) < 0.01:
                continue

            heading_gnss = math.atan2(dg_y, dg_x)
            heading_slam = math.atan2(ds_y, ds_x)
            rot = heading_slam - heading_gnss

            # 平均ノイズを使って重みを計算する（2 測位点の共分散の平均から取得）
            mean_cov = (g0.covariance + g1.covariance) / 2.0
            noise = _covariance_to_scalar_noise(mean_cov, self._default_noise_xy_m)
            w = displacement / (noise + 1e-9)

            sin_sum += w * math.sin(rot)
            cos_sum += w * math.cos(rot)
            sample_count += 1

        if sin_sum == 0.0 and cos_sum == 0.0:
            _logger.warning(
                'PrecisionWeightedAligner: no valid samples for rotation estimation '
                f'(min_speed_ms={self._min_speed_ms}). Returning zero rotation.'
            )
            return 0.0

        if sample_count < _MIN_RELIABLE_SAMPLES:
            _logger.warning(
                f'PrecisionWeightedAligner: only {sample_count} valid samples '
                f'(< {_MIN_RELIABLE_SAMPLES}) for rotation estimation. '
                'Result may be noisy.'
            )

        return math.atan2(sin_sum, cos_sum)

    def _estimate_translation(
        self,
        nodes: list[PoseNode],
        node_timestamps: list[float],
        gnss_list: list[GnssData],
        rotation_rad: float,
    ) -> tuple[float, float]:
        """推定回転を用い、GNSS精度加重最小二乗で平行移動を推定する。

        重み = GNSS 情報行列のトレース: 精度の高い測位ほど強く引き寄せる。
        GNSS を正として SLAM 座標系をフィットさせる。
        """
        cos_r = math.cos(rotation_rad)
        sin_r = math.sin(rotation_rad)

        tx_sum = 0.0
        ty_sum = 0.0
        w_sum = 0.0

        for gnss in gnss_list:
            node = _nearest_node(nodes, node_timestamps, gnss.timestamp, self._max_time_delta_s)
            if node is None:
                continue

            # GNSS 情報量のスカラー重み（精度の逆数の代わりに情報量トレースを使用）
            det = gnss.covariance[0, 0] * gnss.covariance[1, 1] - gnss.covariance[0, 1] ** 2
            if det < 1e-9:
                # フォールバック: デフォルトノイズから情報量を算出
                w = 1.0 / (self._default_noise_xy_m ** 2)
            else:
                try:
                    info = np.linalg.inv(gnss.covariance)
                    w = float(np.trace(info))
                except np.linalg.LinAlgError:
                    w = 1.0 / (self._default_noise_xy_m ** 2)

            # GNSS を R で回転後の残差: SLAM_xy - R * GNSS_xy = t
            rx = cos_r * gnss.x - sin_r * gnss.y
            ry = sin_r * gnss.x + cos_r * gnss.y
            tx_sum += w * (node.x - rx)
            ty_sum += w * (node.y - ry)
            w_sum += w

        if w_sum < 1e-12:
            return 0.0, 0.0

        return tx_sum / w_sum, ty_sum / w_sum
