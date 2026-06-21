from __future__ import annotations

import math

import numpy as np
from scipy.spatial import KDTree

from slam_gnss_2d.scan_matching.base import ScanMatcherBase
from slam_gnss_2d.core.data_types import MatchResult, OdomData, ScanData

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


def _estimate_all_normals(pts: np.ndarray, tree: KDTree, k: int = _N_NORMAL_NEIGHBORS) -> np.ndarray:
    """pts の全点について KDTree で k 個の近傍点から法線を事前計算する。"""
    _, idx = tree.query(pts, k=k)
    neighbors_all = pts[idx]
    centered = neighbors_all - neighbors_all.mean(axis=1, keepdims=True)
    cov_batch = np.einsum('nki,nkj->nij', centered, centered) / max(k - 1, 1)
    _, vecs = np.linalg.eigh(cov_batch)
    return vecs[:, :, 0]


class ICPMatcher(ScanMatcherBase):
    """NumPy/SciPy による Point-to-Line ICP (Gauss-Newton)。
    ロバストカーネルおよび法線事前計算（キャッシュ）をサポート。
    """

    def __init__(
        self,
        max_iterations: int = 30,
        tolerance: float = 1e-4,
        max_correspondence_dist: float = 0.5,
        robust_kernel: str = 'huber',  # 'none', 'huber', 'cauchy'
        robust_kernel_scale: float = 0.1,
        yaw_information_multiplier: float = 1.0,
    ) -> None:
        self._max_iterations = max_iterations
        self._tolerance = tolerance
        self._max_correspondence_dist = max_correspondence_dist
        self._robust_kernel = robust_kernel.lower()
        self._robust_kernel_scale = robust_kernel_scale
        self._yaw_information_multiplier = yaw_information_multiplier
        
        self._src_pts: np.ndarray | None = None
        self._src_tree: KDTree | None = None
        self._src_normals: np.ndarray | None = None

    def set_target_cloud(self, src_pts: np.ndarray) -> None:
        """参照点群をセットし、KDTreeと法線を事前計算する。"""
        self._src_pts = src_pts
        if len(src_pts) >= _N_NORMAL_NEIGHBORS:
            self._src_tree = KDTree(src_pts)
            self._src_normals = _estimate_all_normals(src_pts, self._src_tree)
        else:
            self._src_tree = None
            self._src_normals = None

    def match(
        self,
        dst: ScanData,
        initial_guess: OdomData,
    ) -> MatchResult:
        """セットされたターゲットを基準として dst をマッチングし、補正後の相対変換を返す。"""
        if self._src_pts is None or self._src_tree is None or self._src_normals is None:
            return MatchResult(
                dx=initial_guess.x, dy=initial_guess.y, dyaw=initial_guess.yaw,
                converged=False, information=np.zeros((3, 3)), score=0.0
            )

        dst_pts = _scan_to_points(dst)
        if len(dst_pts) < _N_MIN_CORRESPONDENCES:
            return MatchResult(
                dx=initial_guess.x, dy=initial_guess.y, dyaw=initial_guess.yaw,
                converged=False, information=np.zeros((3, 3)), score=0.0
            )

        tx, ty, theta = initial_guess.x, initial_guess.y, initial_guess.yaw
        H = np.zeros((3, 3))
        converged = False

        for _ in range(self._max_iterations):
            p_trans = _apply_transform(dst_pts, tx, ty, theta)

            dists, nn_idx = self._src_tree.query(p_trans)
            valid_mask = dists < self._max_correspondence_dist
            p_trans_v = p_trans[valid_mask]
            
            if len(p_trans_v) < _N_MIN_CORRESPONDENCES:
                return MatchResult(
                    dx=tx, dy=ty, dyaw=theta,
                    converged=False, information=np.zeros((3, 3)), score=0.0
                )

            q_v = self._src_pts[nn_idx[valid_mask]]
            normals_v = self._src_normals[nn_idx[valid_mask]]

            # Jacobian 行列と残差ベクトルを構築
            rp = p_trans_v - np.array([tx, ty])
            J = np.column_stack([
                normals_v[:, 0],
                normals_v[:, 1],
                -normals_v[:, 0] * rp[:, 1] + normals_v[:, 1] * rp[:, 0],
            ])
            r = np.sum(normals_v * (p_trans_v - q_v), axis=1)

            # ロバストカーネルによる重み付け (IRLS)
            w = np.ones_like(r)
            if self._robust_kernel == 'huber':
                k = self._robust_kernel_scale
                abs_r = np.abs(r)
                mask = abs_r > k
                w[mask] = k / abs_r[mask]
            elif self._robust_kernel == 'cauchy':
                k = self._robust_kernel_scale
                w = 1.0 / (1.0 + (r / k)**2)

            J_w = J * w[:, np.newaxis]
            r_w = r * w

            H = J_w.T @ J
            b = J_w.T @ r

            # 直線廊下など一方向にしか拘束がない環境での特異行列（計算不能）エラーを防ぐため、
            # 微小な正則化項（Tikhonov regularization / LM damping）を追加
            H_reg = H + np.eye(3) * 1e-4

            try:
                delta = np.linalg.solve(H_reg, -b)
            except np.linalg.LinAlgError:
                return MatchResult(
                    dx=tx, dy=ty, dyaw=theta,
                    converged=False, information=np.zeros((3, 3)), score=0.0
                )

            tx += delta[0]
            ty += delta[1]
            theta += delta[2]

            if np.linalg.norm(delta) < self._tolerance:
                converged = True
                break

        n_valid = len(p_trans_v) if 'p_trans_v' in locals() else 0
        information = H / n_valid + 1e-6 * np.eye(3) if n_valid > 0 else np.zeros((3, 3))
        
        # 直進性（Yaw）を保持するため、Yawの確信度を意図的に高く（yaw_information_multiplier倍）設定
        # これにより、GTSAMがGNSSのズレを吸収する際に「横滑り」は許容しても「曲がる」ことは許さなくなる
        if n_valid > 0:
            information[2, 2] *= self._yaw_information_multiplier

        # スコア計算
        score = 0.0
        if converged:
            p_final = _apply_transform(dst_pts, tx, ty, theta)
            dists_f, nn_f = self._src_tree.query(p_final)
            valid_f = dists_f < self._max_correspondence_dist
            p_f = p_final[valid_f]
            if len(p_f) >= _N_MIN_CORRESPONDENCES:
                q_f = self._src_pts[nn_f[valid_f]]
                normals_f = self._src_normals[nn_f[valid_f]]
                r_f = np.sum(normals_f * (p_f - q_f), axis=1)
                
                if self._robust_kernel == 'huber':
                    k = self._robust_kernel_scale
                    abs_r = np.abs(r_f)
                    mask = abs_r <= k
                    cost = np.zeros_like(r_f)
                    cost[mask] = 0.5 * r_f[mask]**2
                    cost[~mask] = k * (abs_r[~mask] - 0.5 * k)
                    score = float(np.mean(cost))
                elif self._robust_kernel == 'cauchy':
                    k = self._robust_kernel_scale
                    cost = 0.5 * k**2 * np.log1p((r_f / k)**2)
                    score = float(np.mean(cost))
                else:
                    score = float(np.mean(np.abs(r_f)))

        return MatchResult(dx=tx, dy=ty, dyaw=theta, converged=converged, information=information, score=score)
