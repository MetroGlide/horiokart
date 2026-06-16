from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from ...data_types import PoseNode


class ReferenceProviderBase(ABC):
    """スキャンマッチングの参照点群供給源を表す抽象基底クラス。

    ScanMatchingBuilder が呼び出し、ScanMatcherBase に渡す src_pts を生成する。
    マッチャー側は参照が1枚スキャン由来かローカルマップ由来かを意識しない。
    """

    @abstractmethod
    def update(self, node: PoseNode) -> None:
        """新しいノードが確定したときに呼ぶ。"""

    @abstractmethod
    def get_reference_pts(self) -> Optional[np.ndarray]:
        """最後に確定したノードのボディフレームで参照点群 (N, 2) を返す。

        まだ参照点群がない場合は None を返す。
        """

    def invalidate_cache(self) -> None:
        """グラフ最適化等でノード位置が更新された後、参照点群キャッシュを無効化する。

        ローカルマップなどノード位置履歴を保持する実装はオーバーライドしてキャッシュをクリアすること。
        デフォルト実装は何もしない。
        """
        pass
