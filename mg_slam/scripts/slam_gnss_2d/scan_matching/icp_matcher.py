from __future__ import annotations

import math

import numpy as np
from scipy.spatial import KDTree

from .base import ScanMatcherBase
from ..data_types import MatchResult, OdomData, ScanData

_N_MIN_CORRESPONDENCES = 10
_N_NORMAL_NEIGHBORS = 5


def _scan_to_points(scan: ScanData) -> np.ndarray:
    """有効レンジのみを2D点群 (N, 2) に変換する。"""
    n = len(scan.ranges)
    angles = scan.angle_min + np.arange(n) * scan.angle_increment
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    valid = (ranges >= scan.range_min) & (ranges <= scan.range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def _apply_transform(pts: np.ndarray, tx: float, ty: float, theta: float) -> np.ndarray:
    """2D剛体変換 T = (tx, ty, theta) を点群に適用する。"""
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return (R @ pts.T).T + np.array([tx, ty])


def _estimate_normals(
    query_pts: np.ndarray,
    src_pts: np.ndarray,
    tree: KDTree,
    k: int = _N_NORMAL_NEIGHBORS,
) -> np.ndarray:
    """query_pts の各点について src_pts の隣接 k 点で PCA を行い法線ベクトルを返す。"""
    _, idx = tree.query(query_pts, k=k)
    # src_pts[idx]: (N, k, 2) — 全点の近傍点群を一括取得
    neighbors_all = src_pts[idx]
    centered = neighbors_all - \
        neighbors_all.mean(axis=1, keepdims=True)  # (N, k, 2)
    # バッチ 2×2 共分散行列: (N, 2, 2)
    cov_batch = np.einsum('nki,nkj->nij', centered, centered) / max(k - 1, 1)
    # np.linalg.eigh は (N, 2, 2) を一括処理できる
    _, vecs = np.linalg.eigh(cov_batch)
    # 最小固有値の固有ベクトル = vecs[:, :, 0]
    return vecs[:, :, 0]


class ICPMatcher(ScanMatcherBase):
    """NumPy/SciPy による Point-to-Line ICP (Gauss-Newton)。"""

    def __init__(
        self,
        max_iterations: int = 30,
        tolerance: float = 1e-4,
        max_correspondence_dist: float = 0.5,
    ) -> None:
        self._max_iterations = max_iterations
        self._tolerance = tolerance
        self._max_correspondence_dist = max_correspondence_dist

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

        src_tree = KDTree(src_pts)
        tx, ty, theta = initial_guess.x, initial_guess.y, initial_guess.yaw

        H = np.zeros((3, 3))
        converged = False

        for _ in range(self._max_iterations):
            p_trans = _apply_transform(dst_pts, tx, ty, theta)

            dists, nn_idx = src_tree.query(p_trans)
            valid_mask = dists < self._max_correspondence_dist
            p_trans_v = p_trans[valid_mask]
            q_v = src_pts[nn_idx[valid_mask]]

            if len(p_trans_v) < _N_MIN_CORRESPONDENCES:
                return MatchResult(
                    dx=tx, dy=ty, dyaw=theta,
                    converged=False, information=np.zeros((3, 3)),
                )

            normals = _estimate_normals(q_v, src_pts, src_tree)

            # Jacobian 行列と残差ベクトルを構築
            # ∂r/∂θ = n^T * (dR/dθ * p) = -n_x*(Rp)_y + n_y*(Rp)_x
            # Rp = p_trans - t（並進を除いた回転後の点）
            rp = p_trans_v - np.array([tx, ty])
            J = np.column_stack([
                normals[:, 0],
                normals[:, 1],
                -normals[:, 0] * rp[:, 1] + normals[:, 1] * rp[:, 0],
            ])
            r = np.sum(normals * (p_trans_v - q_v), axis=1)

            H = J.T @ J
            b = J.T @ r

            try:
                delta = np.linalg.solve(H, -b)
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

        n_valid = len(p_trans_v)
        information = H / n_valid + 1e-6 * \
            np.eye(3) if n_valid > 0 else np.zeros((3, 3))
        return MatchResult(dx=tx, dy=ty, dyaw=theta, converged=converged, information=information)
