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
    normals = np.zeros((len(query_pts), 2))
    for i, neighbors in enumerate(idx):
        neighbors_pts = src_pts[neighbors]  # src_pts のインデックスで参照
        cov = np.cov(neighbors_pts.T)
        _, vecs = np.linalg.eigh(cov)
        normals[i] = vecs[:, 0]  # 最小固有値の固有ベクトル = 法線方向
    return normals


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
        src: ScanData,
        dst: ScanData,
        initial_guess: OdomData,
    ) -> MatchResult:
        """src を基準として dst をマッチングし、補正後の相対変換を返す。

        Args:
            src: 前フレームのスキャン（参照スキャン）
            dst: 現フレームのスキャン（変換対象スキャン）
            initial_guess: odom から得た相対デルタ。x/y/yaw が prev フレーム基準の相対移動量。

        Returns:
            MatchResult: 補正後の相対変換と収束状態、情報行列。
        """
        src_pts = _scan_to_points(src)
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
        information = H / n_valid if n_valid > 0 else np.zeros((3, 3))
        return MatchResult(dx=tx, dy=ty, dyaw=theta, converged=converged, information=information)
