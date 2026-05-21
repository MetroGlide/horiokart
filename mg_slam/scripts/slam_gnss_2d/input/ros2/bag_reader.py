from __future__ import annotations

from typing import Callable, Optional

from ..base import GnssSourceBase, OdomSourceBase, ScanSourceBase
from ...data_types import GnssData, OdomData, ScanData


class BagScanSource(ScanSourceBase):
    """rosbag2 から ScanData をリプレイするソース。Phase 4 で実装予定。"""

    def set_scan_callback(self, callback: Callable[[ScanData], None]) -> None:
        raise NotImplementedError("BagScanSource は Phase 4 で実装予定です")

    def start(self) -> None:
        raise NotImplementedError("BagScanSource は Phase 4 で実装予定です")

    def stop(self) -> None:
        raise NotImplementedError("BagScanSource は Phase 4 で実装予定です")


class BagOdomSource(OdomSourceBase):
    """rosbag2 から OdomData を提供するソース。Phase 4 で実装予定。"""

    def get_odom_at(self, timestamp: float) -> Optional[OdomData]:
        raise NotImplementedError("BagOdomSource は Phase 4 で実装予定です")

    def start(self) -> None:
        raise NotImplementedError("BagOdomSource は Phase 4 で実装予定です")

    def stop(self) -> None:
        raise NotImplementedError("BagOdomSource は Phase 4 で実装予定です")


class BagGnssSource(GnssSourceBase):
    """rosbag2 から GnssData を提供するソース。Phase 4 で実装予定。"""

    def get_gnss_at(self, timestamp: float) -> Optional[GnssData]:
        raise NotImplementedError("BagGnssSource は Phase 4 で実装予定です")

    def get_all_gnss(self) -> list[GnssData]:
        raise NotImplementedError("BagGnssSource は Phase 4 で実装予定です")

    def start(self) -> None:
        raise NotImplementedError("BagGnssSource は Phase 4 で実装予定です")

    def stop(self) -> None:
        raise NotImplementedError("BagGnssSource は Phase 4 で実装予定です")
