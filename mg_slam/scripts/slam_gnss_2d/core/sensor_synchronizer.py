from typing import Callable, Optional

from slam_gnss_2d.core.data_types import SensorFrame, ScanData
from slam_gnss_2d.input.base import ScanSourceBase, OdomSourceBase, GnssSourceBase


class SensorSynchronizer:
    """Scanをトリガーとして、対応する時刻のOdomとGNSSを取得し、同期されたSensorFrameを生成する。"""

    def __init__(
        self,
        scan_source: ScanSourceBase,
        odom_source: OdomSourceBase,
        gnss_source: Optional[GnssSourceBase] = None,
        logger=None
    ) -> None:
        self._scan_source = scan_source
        self._odom_source = odom_source
        self._gnss_source = gnss_source
        self._logger = logger
        self._frame_callback: Optional[Callable[[SensorFrame], None]] = None

        self._scan_recv_count = 0
        self._odom_miss_count = 0

        self._scan_source.set_scan_callback(self._on_scan)

    def set_frame_callback(self, callback: Callable[[SensorFrame], None]) -> None:
        """同期完了時に呼ばれるコールバックを登録する。"""
        self._frame_callback = callback

    def start(self) -> None:
        """全センサソースの受信を開始する。"""
        self._odom_source.start()
        if self._gnss_source is not None:
            self._gnss_source.start()
        self._scan_source.start()

    def stop(self) -> None:
        """全センサソースの受信を停止する。"""
        if self._gnss_source is not None:
            self._gnss_source.stop()
        self._scan_source.stop()
        self._odom_source.stop()

    def get_stats(self) -> dict:
        """現在の統計情報を返す。"""
        return {
            'scans': self._scan_recv_count,
            'odom_miss': self._odom_miss_count,
        }

    def _on_scan(self, scan: ScanData) -> None:
        self._scan_recv_count += 1
        odom = self._odom_source.get_odom_at(scan.timestamp)
        if odom is None:
            self._odom_miss_count += 1
            if self._logger:
                self._logger.warn(
                    f'No odom for scan ts={scan.timestamp:.3f} (miss #{self._odom_miss_count})'
                )
            return

        gnss = None
        if self._gnss_source is not None:
            gnss = self._gnss_source.get_gnss_at(scan.timestamp)

        frame = SensorFrame(scan=scan, odom=odom, gnss=gnss)
        if self._frame_callback is not None:
            self._frame_callback(frame)
