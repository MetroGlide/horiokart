from __future__ import annotations

import math
import numpy as np

from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter

from .base import ScanMatcherBase
from ..data_types import MatchResult, OdomData, ScanData

_N_MIN_CORRESPONDENCES = 10

def _scan_to_points(scan: ScanData) -> np.ndarray:
    n = len(scan.ranges)
    angles = scan.angle_min + np.arange(n) * scan.angle_increment
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    valid = (ranges >= scan.range_min) & (ranges <= scan.range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))

class CSMMatcher(ScanMatcherBase):
    """Correlative Scan Matching (CSM) のシンプルなグリッド探索実装。
    初期値ズレが非常に大きい場合でも大域的に探索してロバストにマッチングを行う。
    """
    
    def __init__(
        self,
        linear_search_window: float = 1.0,    # 探索幅 (±m)
        angular_search_window: float = 0.5,   # 探索幅 (±rad)
        linear_step: float = 0.05,            # 探索ステップ (m)
        angular_step: float = 0.02,           # 探索ステップ (rad)
    ) -> None:
        self._linear_window = linear_search_window
        self._angular_window = angular_search_window
        self._linear_step = linear_step
        self._angular_step = angular_step
        
        self._src_pts: np.ndarray | None = None
        self._tree: cKDTree | None = None
        
    def set_target_cloud(self, src_pts: np.ndarray) -> None:
        """参照点群をセットし、探索用の構造を準備する。
        今回はシンプルな KDTree ベースの擬似尤度評価とする。
        （より高速化する場合はここで 2D Likelihood Grid を事前構築する）
        """
        self._src_pts = src_pts
        if len(src_pts) >= _N_MIN_CORRESPONDENCES:
            # cKDTree は SciPy の C実装版。クエリが高速。
            self._tree = cKDTree(src_pts)
        else:
            self._tree = None

    def match(
        self,
        dst: ScanData,
        initial_guess: OdomData,
    ) -> MatchResult:
        if self._src_pts is None or self._tree is None:
            return MatchResult(dx=initial_guess.x, dy=initial_guess.y, dyaw=initial_guess.yaw, converged=False, information=np.zeros((3, 3)), score=0.0)

        dst_pts = _scan_to_points(dst)
        if len(dst_pts) < _N_MIN_CORRESPONDENCES:
            return MatchResult(dx=initial_guess.x, dy=initial_guess.y, dyaw=initial_guess.yaw, converged=False, information=np.zeros((3, 3)), score=0.0)

        best_score = float('-inf')
        best_pose = (initial_guess.x, initial_guess.y, initial_guess.yaw)
        
        # 探索グリッドの生成
        x_steps = int(self._linear_window / self._linear_step)
        y_steps = int(self._linear_window / self._linear_step)
        yaw_steps = int(self._angular_window / self._angular_step)
        
        x_offsets = np.linspace(-self._linear_window, self._linear_window, 2 * x_steps + 1)
        y_offsets = np.linspace(-self._linear_window, self._linear_window, 2 * y_steps + 1)
        yaw_offsets = np.linspace(-self._angular_window, self._angular_window, 2 * yaw_steps + 1)
        
        base_x, base_y, base_yaw = initial_guess.x, initial_guess.y, initial_guess.yaw
        
        sigma_sq = (self._linear_step * 2.0) ** 2
        
        for dyaw in yaw_offsets:
            theta = base_yaw + dyaw
            c, s = math.cos(theta), math.sin(theta)
            R = np.array([[c, -s], [s, c]])
            rotated_dst = (R @ dst_pts.T).T
            
            for dx in x_offsets:
                for dy in y_offsets:
                    tx = base_x + dx
                    ty = base_y + dy
                    
                    p_trans = rotated_dst + np.array([tx, ty])
                    
                    # 近傍探索による擬似尤度評価
                    dists, _ = self._tree.query(p_trans, k=1, distance_upper_bound=self._linear_step * 3)
                    # distance_upper_bound を超えた場合は inf になるため、有効な距離のみ尤度に変換
                    valid_mask = dists != float('inf')
                    
                    if np.sum(valid_mask) < _N_MIN_CORRESPONDENCES:
                        continue
                        
                    # 距離を正規分布ベースの尤度スコアに変換して加算
                    score = float(np.sum(np.exp(-0.5 * (dists[valid_mask] ** 2) / sigma_sq)))
                    
                    if score > best_score:
                        best_score = score
                        best_pose = (tx, ty, theta)
                        
        converged = best_score > 0.0
        # CSM自体の情報行列は形状から推定可能だが、簡易的に固定値とする
        information = np.eye(3) * (best_score / len(dst_pts)) * 100.0 if converged else np.zeros((3,3))
        
        return MatchResult(
            dx=best_pose[0],
            dy=best_pose[1],
            dyaw=best_pose[2],
            converged=converged,
            information=information,
            score=best_score
        )
