from __future__ import annotations

from abc import ABC, abstractmethod

from ..data_types import MatchResult, OdomData, ScanData


class ScanMatcherBase(ABC):
    """2D スキャンマッチングの抽象基底クラス。

    NumPy ICP・KISS-ICP 等に差し替え可能。
    """

    @abstractmethod
    def match(
        self,
        src: ScanData,
        dst: ScanData,
        initial_guess: OdomData,
    ) -> MatchResult:
        """src を基準として dst をマッチングし、補正後の相対変換を返す。

        Args:
            src: 前フレームのスキャン（基準スキャン）
            dst: 現フレームのスキャン（変換対象スキャン）
            initial_guess: オドメトリから得られる初期推定値。
                x/y/yaw は src フレームを基準とした相対デルタ。

        Returns:
            MatchResult: 補正後の相対変換と収束状態、情報行列。
                converged=False の場合、呼び出し元はオドメトリ値で代替すること。
        """
        raise NotImplementedError
