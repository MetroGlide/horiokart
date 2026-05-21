from __future__ import annotations

from typing import Optional

from .base import PoseGraphBuilderBase
from ..data_types import OdomData, PoseNode, ScanData
from ..scan_matching.base import ScanMatcherBase


class ScanMatchingBuilder(PoseGraphBuilderBase):
    """スキャンマッチング補正を加えたポーズグラフ構築実装。Phase 2 で実装予定。

    オドメトリを初期値として ScanMatcherBase 実装（ICP 等）で補正し、
    より高精度な相対移動量をノードに記録する。
    """

    def __init__(self, matcher: ScanMatcherBase) -> None:
        self._matcher = matcher

    def add_scan(self, scan: ScanData, odom: OdomData) -> Optional[PoseNode]:
        raise NotImplementedError("ScanMatchingBuilder は Phase 2 で実装予定です")

    def get_nodes(self) -> list[PoseNode]:
        raise NotImplementedError("ScanMatchingBuilder は Phase 2 で実装予定です")

    def reset(self) -> None:
        raise NotImplementedError("ScanMatchingBuilder は Phase 2 で実装予定です")
