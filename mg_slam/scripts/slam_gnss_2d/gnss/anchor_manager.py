from __future__ import annotations

from ..data_types import GnssData


class GnssAnchorManager:
    """GNSSアンカー点を管理し、UTM座標をローカル座標へ変換する。"""

    def __init__(self) -> None:
        self._anchor_x: float | None = None
        self._anchor_y: float | None = None

    @property
    def is_initialized(self) -> bool:
        return self._anchor_x is not None and self._anchor_y is not None

    @property
    def anchor_utm(self) -> tuple[float, float] | None:
        if not self.is_initialized:
            return None
        return (self._anchor_x, self._anchor_y)

    def try_set_anchor(self, gnss: GnssData, min_fix_status: int) -> bool:
        if self.is_initialized:
            return False
        if gnss.fix_status < min_fix_status:
            return False
        self._anchor_x = gnss.x
        self._anchor_y = gnss.y
        return True

    def to_local(self, gnss: GnssData) -> tuple[float, float]:
        if not self.is_initialized:
            raise RuntimeError('GNSS anchor is not initialized')
        return (gnss.x - self._anchor_x, gnss.y - self._anchor_y)
