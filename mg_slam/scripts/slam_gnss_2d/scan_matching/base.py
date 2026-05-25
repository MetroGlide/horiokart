from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from ..data_types import MatchResult, OdomData, ScanData


class ScanMatcherBase(ABC):
    """2D スキャンマッチングの抽象基底クラス。

    ICPMatcher・NDTMatcher 等に差し替え可能。
    参照点群の生成元（1枚スキャン/ローカルマップ）は ReferenceProviderBase が担うため、
    このクラスは参照の由来を意識しない。
    """

    @abstractmethod
    def match(
        self,
        src_pts: np.ndarray,
        dst: ScanData,
        initial_guess: OdomData,
    ) -> MatchResult:
        """src_pts を基準として dst をマッチングし、補正後の相対変換を返す。

        Args:
            src_pts: 参照点群 (N, 2)。最後ノードのボディフレーム基準。
                     1枚スキャン分またはローカルマップ集約分のいずれかが渡される。
            dst: 現フレームのスキャン（変換対象スキャン）。
            initial_guess: オドメトリから得られる初期推定値。
                x/y/yaw は src_pts フレームを基準とした相対デルタ。

        Returns:
            MatchResult: 補正後の相対変換と収束状態、情報行列。
                converged=False の場合、呼び出し元はオドメトリ値で代替すること。
        """
        raise NotImplementedError
