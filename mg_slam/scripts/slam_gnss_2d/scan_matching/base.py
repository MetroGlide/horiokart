from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from slam_gnss_2d.core.data_types import MatchResult, OdomData, ScanData


class ScanMatcherBase(ABC):
    """2D スキャンマッチングの抽象基底クラス。

    ICPMatcher・NDTMatcher 等に差し替え可能。
    参照点群の生成元（1枚スキャン/ローカルマップ）は ReferenceProviderBase が担うため、
    このクラスは参照の由来を意識しない。
    """

    @abstractmethod
    def set_target_cloud(self, src_pts: np.ndarray) -> None:
        """参照点群（ターゲット）をセットし、必要な事前計算（KDTree構築、法線計算、セル構築など）を行う。

        Args:
            src_pts: 参照点群 (N, 2)。
        """
        raise NotImplementedError

    @abstractmethod
    def match(
        self,
        dst: ScanData,
        initial_guess: OdomData,
    ) -> MatchResult:
        """セットされたターゲットを基準として dst をマッチングし、補正後の相対変換を返す。

        Args:
            dst: 現フレームのスキャン（変換対象スキャン）。
            initial_guess: オドメトリから得られる初期推定値。
                x/y/yaw は src_pts フレームを基準とした相対デルタ。

        Returns:
            MatchResult: 補正後の相対変換と収束状態、情報行列。
                converged=False の場合、呼び出し元はオドメトリ値で代替すること。
        """
        raise NotImplementedError
