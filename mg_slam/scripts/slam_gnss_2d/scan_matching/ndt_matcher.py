from __future__ import annotations

import math
from typing import Optional

import numpy as np

from slam_gnss_2d.scan_matching.base import ScanMatcherBase
from slam_gnss_2d.core.data_types import MatchResult, OdomData, ScanData

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
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """src_pts から NDT セルを構築する。

    Returns:
        cell_keys_sorted  : (C, 2) int32 — ソート済みセルインデックス
        means_sorted      : (C, 2) float64 — ソート済み重心
        sigma_invs_sorted : (C, 2, 2) float64 — ソート済み精度行列
        encoded_cells     : (C,) int64 — searchsorted用のエンコード済みキー
    """
    inv = 1.0 / cell_size
    cell_indices = np.floor(src_pts * inv).astype(np.int32)

    unique_keys, inverse = np.unique(cell_indices, axis=0, return_inverse=True)
    C = len(unique_keys)

    counts = np.bincount(inverse, minlength=C)
    valid_mask = counts >= 3
    if not np.any(valid_mask):
        empty_keys = np.empty((0, 2), dtype=np.int32)
        empty_means = np.empty((0, 2))
        empty_si = np.empty((0, 2, 2))
        empty_enc = np.empty(0, dtype=np.int64)
        return empty_keys, empty_means, empty_si, empty_enc

    sums = np.zeros((C, 2), dtype=np.float64)
    np.add.at(sums, inverse, src_pts)
    means = sums / np.maximum(counts[:, None], 1)

    centered = src_pts - means[inverse]
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

    cov_batch = np.empty((C, 2, 2), dtype=np.float64)
    cov_batch[:, 0, 0] = cov_xx + 1e-3
    cov_batch[:, 0, 1] = cov_xy
    cov_batch[:, 1, 0] = cov_xy
    cov_batch[:, 1, 1] = cov_yy + 1e-3

    valid_cov = cov_batch[valid_mask]
    try:
        sigma_invs = np.linalg.inv(valid_cov)
    except np.linalg.LinAlgError:
        empty_keys = np.empty((0, 2), dtype=np.int32)
        empty_means = np.empty((0, 2))
        empty_si = np.empty((0, 2, 2))
        empty_enc = np.empty(0, dtype=np.int64)
        return empty_keys, empty_means, empty_si, empty_enc

    cell_keys = unique_keys[valid_mask].astype(np.int32)
    means_v = means[valid_mask]
    
    sort_order = np.lexsort((cell_keys[:, 1], cell_keys[:, 0]))
    cell_keys_sorted = cell_keys[sort_order]
    means_sorted = means_v[sort_order]
    sigma_invs_sorted = sigma_invs[sort_order]
    encoded_cells = (cell_keys_sorted[:, 0].astype(np.int64) * (2 ** 32) + cell_keys_sorted[:, 1])
    
    return cell_keys_sorted, means_sorted, sigma_invs_sorted, encoded_cells


class NDTMatcher(ScanMatcherBase):
    """2D Normal Distributions Transform (NDT) によるスキャンマッチング。
    マルチ解像度およびキャッシュ化に対応。
    """

    def __init__(
        self,
        max_iterations: int = 30,
        tolerance: float = 1e-4,
        cell_sizes: list[float] | None = None,
        use_bilinear: bool = False,
        yaw_information_multiplier: float = 1.0,
    ) -> None:
        self._max_iterations = max_iterations
        self._tolerance = tolerance
        
        if cell_sizes is None:
            self._cell_sizes = [1.0]
        else:
            self._cell_sizes = sorted(cell_sizes, reverse=True)  # 粗い順(大きい順)にソート
            
        self._use_bilinear = use_bilinear
        self._yaw_information_multiplier = yaw_information_multiplier
        
        # 参照点群のキャッシュ
        self._src_pts: np.ndarray | None = None
        self._ndt_pyramids: dict[float, tuple] = {}

    def set_target_cloud(self, src_pts: np.ndarray) -> None:
        """参照点群をセットし、各解像度のNDTセルを事前計算する。"""
        self._src_pts = src_pts
        self._ndt_pyramids.clear()
        
        for cs in self._cell_sizes:
            self._ndt_pyramids[cs] = _build_ndt_cells(src_pts, cs)

    def _match_single_resolution(
        self,
        dst_pts: np.ndarray,
        tx: float,
        ty: float,
        theta: float,
        cell_size: float,
        cell_data: tuple
    ) -> tuple[float, float, float, bool, np.ndarray, float]:
        """1つの解像度で最適化を行う"""
        cell_keys_sorted, means_sorted, sigma_invs_sorted, encoded_cells = cell_data
        
        if len(cell_keys_sorted) == 0:
            return tx, ty, theta, False, np.zeros((3, 3)), 0.0

        inv_cell = 1.0 / cell_size
        H_final = np.zeros((3, 3))
        n_valid_final = 0
        converged = False

        for _ in range(self._max_iterations):
            c, s = math.cos(theta), math.sin(theta)
            R = np.array([[c, -s], [s, c]])
            p_trans = (R @ dst_pts.T).T + np.array([tx, ty])

            query_keys = np.floor(p_trans * inv_cell).astype(np.int32)
            encoded_query = query_keys[:, 0].astype(np.int64) * (2 ** 32) + query_keys[:, 1]
            
            # NOTE: ここにBilinear補間を入れる場合、近傍セルのキーも生成して検索し、確率分布を重み付け合算する。
            # 現在はシンプル版（Nearest Cell）として実装。
            
            hit_pos = np.searchsorted(encoded_cells, encoded_query)
            in_range = hit_pos < len(encoded_cells)
            exact_match = np.zeros(len(p_trans), dtype=bool)
            exact_match[in_range] = (encoded_cells[hit_pos[in_range]] == encoded_query[in_range])

            if exact_match.sum() < _N_MIN_CORRESPONDENCES:
                break

            pt_idx = np.where(exact_match)[0]
            cell_idx = hit_pos[pt_idx]
            p_hit = p_trans[pt_idx]
            d_hit = p_hit - means_sorted[cell_idx]
            si_hit = sigma_invs_sorted[cell_idx]

            exponents = -0.5 * np.einsum('ni,nij,nj->n', d_hit, si_hit, d_hit)
            exp_mask = exponents >= _EXPONENT_CUTOFF
            if exp_mask.sum() < _N_MIN_CORRESPONDENCES:
                break

            pt_idx_f = pt_idx[exp_mask]
            cell_idx_f = cell_idx[exp_mask]
            d_f = d_hit[exp_mask]
            si_f = si_hit[exp_mask]
            exp_vals = np.exp(exponents[exp_mask])
            n_valid = len(exp_vals)

            p_orig_f = dst_pts[pt_idx_f]
            dp_dtheta = np.column_stack([
                -s * p_orig_f[:, 0] - c * p_orig_f[:, 1],
                c * p_orig_f[:, 0] - s * p_orig_f[:, 1],
            ])
            
            J = np.zeros((n_valid, 2, 3))
            J[:, 0, 0] = 1.0
            J[:, 1, 1] = 1.0
            J[:, :, 2] = dp_dtheta

            sigma_d = np.einsum('nij,nj->ni', si_f, d_f)
            g = np.einsum('n,nki,ni->k', exp_vals, J.transpose(0, 2, 1), sigma_d)
            H = np.einsum('n,nki,nij,nlj->kl', exp_vals, J.transpose(0, 2, 1), si_f, J.transpose(0, 2, 1))

            H_final = H
            n_valid_final = n_valid

            try:
                delta = np.linalg.solve(H + 1e-6 * np.eye(3), -g)
            except np.linalg.LinAlgError:
                break

            tx += delta[0]
            ty += delta[1]
            theta += delta[2]

            if np.linalg.norm(delta) < self._tolerance:
                converged = True
                break

        information = H_final / n_valid_final + 1e-6 * np.eye(3) if n_valid_final > 0 else np.zeros((3, 3))
        if n_valid_final > 0:
            information[2, 2] *= self._yaw_information_multiplier

        score = 0.0
        if converged:
            c_f, s_f = math.cos(theta), math.sin(theta)
            p_final = (np.array([[c_f, -s_f], [s_f, c_f]]) @ dst_pts.T).T + np.array([tx, ty])
            query_keys_f = np.floor(p_final * inv_cell).astype(np.int32)
            encoded_query_f = query_keys_f[:, 0].astype(np.int64) * (2 ** 32) + query_keys_f[:, 1]
            hit_pos_f = np.searchsorted(encoded_cells, encoded_query_f)
            in_range_f = hit_pos_f < len(encoded_cells)
            exact_match_f = np.zeros(len(p_final), dtype=bool)
            exact_match_f[in_range_f] = (encoded_cells[hit_pos_f[in_range_f]] == encoded_query_f[in_range_f])
            
            pt_idx_f2 = np.where(exact_match_f)[0]
            if len(pt_idx_f2) >= _N_MIN_CORRESPONDENCES:
                cell_idx_f2 = hit_pos_f[pt_idx_f2]
                d_f2 = p_final[pt_idx_f2] - means_sorted[cell_idx_f2]
                si_f2 = sigma_invs_sorted[cell_idx_f2]
                exp_f2 = -0.5 * np.einsum('ni,nij,nj->n', d_f2, si_f2, d_f2)
                mask_f2 = exp_f2 >= _EXPONENT_CUTOFF
                if mask_f2.sum() >= _N_MIN_CORRESPONDENCES:
                    score = float(np.mean(-exp_f2[mask_f2]))

        return tx, ty, theta, converged, information, score

    def match(
        self,
        dst: ScanData,
        initial_guess: OdomData,
    ) -> MatchResult:
        if self._src_pts is None or not self._ndt_pyramids:
            return MatchResult(dx=initial_guess.x, dy=initial_guess.y, dyaw=initial_guess.yaw, converged=False, information=np.zeros((3, 3)), score=0.0)

        dst_pts = _scan_to_points(dst)
        if len(dst_pts) < _N_MIN_CORRESPONDENCES:
            return MatchResult(dx=initial_guess.x, dy=initial_guess.y, dyaw=initial_guess.yaw, converged=False, information=np.zeros((3, 3)), score=0.0)

        tx, ty, theta = initial_guess.x, initial_guess.y, initial_guess.yaw
        
        final_converged = False
        final_info = np.zeros((3, 3))
        final_score = 0.0

        # マルチ解像度で最適化（粗い方から細かい方へ）
        for cs in self._cell_sizes:
            cell_data = self._ndt_pyramids[cs]
            tx, ty, theta, conv, info, score = self._match_single_resolution(
                dst_pts, tx, ty, theta, cs, cell_data
            )
            final_converged = conv
            final_info = info
            final_score = score
            
            # 最低解像度で大きく失敗した場合は早期リターン
            if not conv and cs == self._cell_sizes[0]:
                break

        return MatchResult(
            dx=tx, dy=ty, dyaw=theta,
            converged=final_converged,
            information=final_info,
            score=final_score
        )
