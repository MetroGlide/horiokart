from __future__ import annotations

import math
from typing import Optional

import numpy as np

from .base import ScanMatcherBase
from ..data_types import MatchResult, OdomData, ScanData

_N_MIN_CORRESPONDENCES = 5
_EXPONENT_CUTOFF = -3.0  # exp(x) < e^-3 ≈ 0.05 の点は除外


def _scan_to_points(scan: ScanData) -> np.ndarray:
    """有効レンジのみを2D点群 (N, 2) に変換する。"""
    n = len(scan.ranges)
    angles = scan.angle_min + np.arange(n) * scan.angle_increment
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    valid = (ranges >= scan.range_min) & (ranges <= scan.range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def _build_ndt_cells(
    src_pts: np.ndarray,
    cell_size: float,
) -> dict[tuple[int, int], tuple[np.ndarray, np.ndarray]]:
    """src_pts から NDT セル辞書を構築する。

    Returns:
        {(cell_x, cell_y): (mean (2,), sigma_inv (2, 2))}
        3点未満のセルは除外する。
    """
    cells_pts: dict[tuple[int, int], list] = {}
    inv = 1.0 / cell_size
    for pt in src_pts:
        key = (int(math.floor(pt[0] * inv)), int(math.floor(pt[1] * inv)))
        if key not in cells_pts:
            cells_pts[key] = []
        cells_pts[key].append(pt)

    cells = {}
    for key, pts in cells_pts.items():
        if len(pts) < 3:
            continue
        arr = np.array(pts)
        mean = arr.mean(axis=0)
        cov = np.cov(arr.T) + 1e-3 * np.eye(2)  # 退化防止の正則化
        try:
            sigma_inv = np.linalg.inv(cov)
        except np.linalg.LinAlgError:
            continue
        cells[key] = (mean, sigma_inv)

    return cells


class NDTMatcher(ScanMatcherBase):
    """2D Normal Distributions Transform (NDT) によるスキャンマッチング。

    空間をグリッドに分割し、各セルを2Dガウス分布でモデル化する。
    ICPと異なり対応点を必要としないため、スパースな屋外環境でロバスト。
    """

    def __init__(
        self,
        max_iterations: int = 30,
        tolerance: float = 1e-4,
        cell_size: float = 1.0,
    ) -> None:
        self._max_iterations = max_iterations
        self._tolerance = tolerance
        self._cell_size = cell_size

    def match(
        self,
        src_pts: np.ndarray,
        dst: ScanData,
        initial_guess: OdomData,
    ) -> MatchResult:
        """src_pts を基準として dst をマッチングし、補正後の相対変換を返す。

        Args:
            src_pts: 参照点群 (N, 2)。最後ノードのボディフレーム基準。
            dst: 現フレームのスキャン（変換対象スキャン）。
            initial_guess: odom から得た相対デルタ。x/y/yaw が prev フレーム基準の相対移動量。

        Returns:
            MatchResult: 補正後の相対変換と収束状態、情報行列。
        """
        dst_pts = _scan_to_points(dst)

        if len(src_pts) < _N_MIN_CORRESPONDENCES or len(dst_pts) < _N_MIN_CORRESPONDENCES:
            return MatchResult(
                dx=initial_guess.x, dy=initial_guess.y, dyaw=initial_guess.yaw,
                converged=False, information=np.zeros((3, 3)),
            )

        cells = _build_ndt_cells(src_pts, self._cell_size)
        if not cells:
            return MatchResult(
                dx=initial_guess.x, dy=initial_guess.y, dyaw=initial_guess.yaw,
                converged=False, information=np.zeros((3, 3)),
            )

        inv_cell = 1.0 / self._cell_size
        tx, ty, theta = initial_guess.x, initial_guess.y, initial_guess.yaw
        H_final = np.zeros((3, 3))
        n_valid_final = 0
        converged = False

        for _ in range(self._max_iterations):
            c, s = math.cos(theta), math.sin(theta)
            R = np.array([[c, -s], [s, c]])
            p_trans = (R @ dst_pts.T).T + np.array([tx, ty])

            # g: -score の勾配 (3,)
            # H: -score のヘッセ行列近似 (3, 3)
            g = np.zeros(3)
            H = np.zeros((3, 3))
            n_valid = 0

            for i in range(len(p_trans)):
                cell_key = (
                    int(math.floor(p_trans[i, 0] * inv_cell)),
                    int(math.floor(p_trans[i, 1] * inv_cell)),
                )
                if cell_key not in cells:
                    continue
                mean, sigma_inv = cells[cell_key]

                d = p_trans[i] - mean
                exponent = -0.5 * float(d @ sigma_inv @ d)
                if exponent < _EXPONENT_CUTOFF:
                    continue

                exp_val = math.exp(exponent)
                n_valid += 1

                # ヤコビアン J (2, 3): p_trans[i] の (tx, ty, θ) 微分
                p_orig = dst_pts[i]
                dp_dtheta = np.array([
                    -s * p_orig[0] - c * p_orig[1],
                    c * p_orig[0] - s * p_orig[1],
                ])
                J = np.array([[1.0, 0.0, dp_dtheta[0]],
                              [0.0, 1.0, dp_dtheta[1]]])

                # -score の勾配・ヘッセ行列に加算
                sigma_d = sigma_inv @ d
                g += exp_val * (J.T @ sigma_d)
                H += exp_val * (J.T @ sigma_inv @ J)

            if n_valid < _N_MIN_CORRESPONDENCES:
                return MatchResult(
                    dx=tx, dy=ty, dyaw=theta,
                    converged=False, information=np.zeros((3, 3)),
                )

            H_final = H
            n_valid_final = n_valid

            # Newton ステップ: H * delta = -g を解く
            try:
                delta = np.linalg.solve(H + 1e-6 * np.eye(3), -g)
            except np.linalg.LinAlgError:
                return MatchResult(
                    dx=tx, dy=ty, dyaw=theta,
                    converged=False, information=np.zeros((3, 3)),
                )

            tx += delta[0]
            ty += delta[1]
            theta += delta[2]

            if np.linalg.norm(delta) < self._tolerance:
                converged = True
                break

        information = H_final / \
            n_valid_final if n_valid_final > 0 else np.zeros((3, 3))
        return MatchResult(dx=tx, dy=ty, dyaw=theta, converged=converged, information=information)
