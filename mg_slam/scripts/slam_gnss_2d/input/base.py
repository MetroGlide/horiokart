from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable, Optional

from slam_gnss_2d.core.data_types import GnssData, OdomData, ScanData


class ScanSourceBase(ABC):
    """スキャンデータを供給するソースの抽象基底クラス。

    ROS2ノード・rosbag2・シミュレータ等、任意の実装に差し替え可能。
    """

    @abstractmethod
    def set_scan_callback(self, callback: Callable[[ScanData], None]) -> None:
        """スキャンが届いた際に呼ばれるコールバックを登録する。"""
        raise NotImplementedError

    @abstractmethod
    def start(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError


class OdomSourceBase(ABC):
    """オドメトリデータを供給するソースの抽象基底クラス。

    /odom・/odom/gnss・Bag等、任意の実装に差し替え可能。
    タイムスタンプ指定で最近傍値を返すインターフェース。
    """

    @abstractmethod
    def get_odom_at(self, timestamp: float) -> Optional[OdomData]:
        """指定タイムスタンプに最も近いオドメトリを返す。バッファが空の場合は None。"""
        raise NotImplementedError

    @abstractmethod
    def start(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError


class GnssSourceBase(ABC):
    """GNSSデータを供給するソースの抽象基底クラス。Phase 4 で使用する。"""

    @abstractmethod
    def get_gnss_at(self, timestamp: float) -> Optional[GnssData]:
        """指定タイムスタンプに最も近い GnssData を返す。"""
        raise NotImplementedError

    @abstractmethod
    def get_all_gnss(self) -> list[GnssData]:
        """バッファ内の全 GnssData を返す（オフラインバッチ最適化用）。"""
        raise NotImplementedError

    @abstractmethod
    def start(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError
