from __future__ import annotations

from .base import ScanMatcherBase
from ..data_types import OdomData, ScanData


class ICPMatcher(ScanMatcherBase):
    """NumPy/SciPy による Point-to-Line ICP の実装。Phase 2 で実装予定。"""

    def match(
        self,
        src: ScanData,
        dst: ScanData,
        initial_guess: OdomData,
    ) -> tuple[float, float, float]:
        raise NotImplementedError("ICPMatcher は Phase 2 で実装予定です")
