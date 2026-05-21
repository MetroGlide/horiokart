from __future__ import annotations

from abc import ABC, abstractmethod

from ..data_types import OdomData, ScanData


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
    ) -> tuple[float, float, float]:
        """src を dst に合わせ込み、補正後の相対変換 (dx, dy, dyaw) を返す。

        Args:
            src: 前フレームのスキャン（基準）
            dst: 現フレームのスキャン（変換対象）
            initial_guess: オドメトリから得られる初期推定値

        Returns:
            (dx, dy, dyaw): スキャンマッチングによる補正後の相対移動量
        """
        raise NotImplementedError
