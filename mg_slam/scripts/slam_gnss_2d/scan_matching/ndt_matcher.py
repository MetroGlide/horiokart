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
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """src_pts から NDT セルを構築する。

    Returns:
        cell_keys  : (C, 2) int32 — 有効セルのグリッドインデックス
        means      : (C, 2) float64 — 各セルの重心
        sigma_invs : (C, 2, 2) float64 — 各セルの精度行列
    """
    inv = 1.0 / cell_size
    cell_indices = np.floor(src_pts * inv).astype(np.int32)  # (N, 2)

    unique_keys, inverse = np.unique(cell_indices, axis=0, return_inverse=True)
    C = len(unique_keys)

    counts = np.bincount(inverse, minlength=C)
    valid_mask = counts >= 3
    if not np.any(valid_mask):
        return np.empty((0, 2), dtype=np.int32), np.empty((0, 2)), np.empty((0, 2, 2))

    # 重心を一括計算
    sums = np.zeros((C, 2), dtype=np.float64)
    np.add.at(sums, inverse, src_pts)
    means = sums / np.maximum(counts[:, None], 1)  # (C, 2)

    # 共分散行列要素を一括蓄積
    centered = src_pts - means[inverse]  # (N, 2)
    cov_xx = np.zeros(C, dtype=np.float64)
    cov_xy = np.zeros(C, dtype=np.float64)
    cov_yy = np.zeros(C, dtype=np.float64)
    np.add.at(cov_xx, inverse, centered[:, 0] ** 2)
    np.add.at(cov_xy, inverse, centered[:, 0] * centered[:, 1])
    np.add.at(cov_yy, inverse, centered[:, 1] ** 2)
    denom = np.maximum(counts - 1, 1).astype(np.float64)
    cov_xx /= denom
    cov_xy /= denom
    cov_yy /= denom

    # (C, 2, 2) 共分散行列組み立て + 正則化
    cov_batch = np.empty((C, 2, 2), dtype=np.float64)
    cov_batch[:, 0, 0] = cov_xx + 1e-3
    cov_batch[:, 0, 1] = cov_xy
    cov_batch[:, 1, 0] = cov_xy
    cov_batch[:, 1, 1] = cov_yy + 1e-3

    valid_cov = cov_batch[valid_mask]
    try:
        sigma_invs = np.linalg.inv(valid_cov)
    except np.linalg.LinAlgError:
        return np.empty((0, 2), dtype=np.int32), np.empty((0, 2)), np.empty((0, 2, 2))

    return (
        unique_keys[valid_mask].astype(np.int32),
        means[valid_mask],
        sigma_invs,
    )


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

        cell_keys, means, sigma_invs = _build_ndt_cells(
            src_pts, self._cell_size)
        if len(cell_keys) == 0:
            return MatchResult(
                dx=initial_guess.x, dy=initial_guess.y, dyaw=initial_guess.yaw,
                converged=False, information=np.zeros((3, 3)),
            )

        # セルキーを行優先でソートして searchsorted によるルックアップを可能にする
        sort_order = np.lexsort((cell_keys[:, 1], cell_keys[:, 0]))
        cell_keys_sorted = cell_keys[sort_order]          # (C, 2)
        means_sorted = means[sort_order]                  # (C, 2)
        sigma_invs_sorted = sigma_invs[sort_order]        # (C, 2, 2)

        inv_cell = 1.0 / self._cell_size
        tx, ty, theta = initial_guess.x, initial_guess.y, initial_guess.yaw
        H_final = np.zeros((3, 3))
        n_valid_final = 0
        converged = False

        for _ in range(self._max_iterations):
            c, s = math.cos(theta), math.sin(theta)
            R = np.array([[c, -s], [s, c]])
            p_trans = (R @ dst_pts.T).T + np.array([tx, ty])  # (N, 2)

            # 全点のセルキーを一括計算
            query_keys = np.floor(
                p_trans * inv_cell).astype(np.int32)  # (N, 2)

            # lexsort 済み cell_keys_sorted に対して searchsorted でルックアップ
            # 各点が cell_keys_sorted の何行目に対応するかを求める
            encoded_query = query_keys[:, 0].astype(
                np.int64) * (2 ** 32) + query_keys[:, 1]
            encoded_cells = cell_keys_sorted[:, 0].astype(
                np.int64) * (2 ** 32) + cell_keys_sorted[:, 1]
            hit_pos = np.searchsorted(encoded_cells, encoded_query)
            in_range = hit_pos < len(encoded_cells)
            exact_match = np.zeros(len(p_trans), dtype=bool)
            exact_match[in_range] = (
                encoded_cells[hit_pos[in_range]] == encoded_query[in_range])

            if exact_match.sum() < _N_MIN_CORRESPONDENCES:
                return MatchResult(
                    dx=tx, dy=ty, dyaw=theta,
                    converged=False, information=np.zeros((3, 3)),
                )

            # ヒットした点のみ抽出
            pt_idx = np.where(exact_match)[0]          # (M,)
            cell_idx = hit_pos[pt_idx]                 # (M,) → cell 配列インデックス
            p_hit = p_trans[pt_idx]                    # (M, 2)
            d_hit = p_hit - means_sorted[cell_idx]     # (M, 2)
            si_hit = sigma_invs_sorted[cell_idx]       # (M, 2, 2)

            # exponent = -0.5 * d^T Σ^-1 d  (M,)
            exponents = -0.5 * np.einsum('ni,nij,nj->n', d_hit, si_hit, d_hit)
            exp_mask = exponents >= _EXPONENT_CUTOFF
            if exp_mask.sum() < _N_MIN_CORRESPONDENCES:
                return MatchResult(
                    dx=tx, dy=ty, dyaw=theta,
                    converged=False, information=np.zeros((3, 3)),
                )

            pt_idx_f = pt_idx[exp_mask]
            cell_idx_f = cell_idx[exp_mask]
            d_f = d_hit[exp_mask]                      # (M2, 2)
            si_f = si_hit[exp_mask]                    # (M2, 2, 2)
            exp_vals = np.exp(exponents[exp_mask])     # (M2,)
            n_valid = len(exp_vals)

            # ヤコビアン J: (M2, 2, 3)
            p_orig_f = dst_pts[pt_idx_f]               # (M2, 2)
            dp_dtheta = np.column_stack([
                -s * p_orig_f[:, 0] - c * p_orig_f[:, 1],
                c * p_orig_f[:, 0] - s * p_orig_f[:, 1],
            ])  # (M2, 2)
            # J[k] = [[1, 0, dp_dtheta[k,0]], [0, 1, dp_dtheta[k,1]]]
            J = np.zeros((n_valid, 2, 3))
            J[:, 0, 0] = 1.0
            J[:, 1, 1] = 1.0
            J[:, :, 2] = dp_dtheta  # (M2, 2)

            # g = sum_k exp_k * J_k^T @ (Σ^-1 d)_k  → (3,)
            sigma_d = np.einsum('nij,nj->ni', si_f, d_f)   # (M2, 2)
            # exp_vals[:,None] * J^T (3,2) @ sigma_d (2,) → weighted sum
            g = np.einsum('n,nki,ni->k', exp_vals,
                          J.transpose(0, 2, 1), sigma_d)

            # H = sum_k exp_k * J_k^T @ Σ^-1 @ J_k  → (3, 3)
            H = np.einsum('n,nki,nij,nlj->kl', exp_vals,
                          J.transpose(0, 2, 1), si_f, J.transpose(0, 2, 1))

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

        information = H_final / n_valid_final + 1e-6 * \
            np.eye(3) if n_valid_final > 0 else np.zeros((3, 3))
        return MatchResult(dx=tx, dy=ty, dyaw=theta, converged=converged, information=information)
