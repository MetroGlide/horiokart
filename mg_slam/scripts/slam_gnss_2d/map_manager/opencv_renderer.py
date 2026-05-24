from __future__ import annotations

import logging
import math

import numpy as np
import cv2

from .base import MapRendererBase
from ..data_types import PoseNode

_logger = logging.getLogger(__name__)


class OpenCVRenderer(MapRendererBase):
    """OpenCV cv2.line を使った Ray-Casting による占有格子マップ実装。

    内部マップは uint8 で管理する:
        128 = unknown
        255 = free（cv2.line で空き領域を描画）
          0 = occupied（ヒット点を上書き）

    to_occupancy_array() で ROS2 OccupancyGrid 形式（-1 / 0 / 100）に変換して返す。
    マップの境界は rerender_all() が呼ばれるたびに全ノードから動的に計算される。
    """

    def __init__(
        self,
        resolution: float = 0.05,
        expansion_margin: float = 100.0,
    ) -> None:
        """
        Args:
            resolution: マップの解像度 [m/pixel]
            expansion_margin: 境界計算時に全ノード位置に追加するマージン [m]
        """
        self._resolution = resolution
        self._expansion_margin = expansion_margin
        self._map_size = 1
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._map = np.full((1, 1), 128, dtype=np.uint8)
        self._render_count = 0

    def add_node(self, node: PoseNode) -> bool:
        if node.scan is None:
            return True
        robot_px, robot_py = self._world_to_pixel(node.x, node.y)
        if not self._in_bounds(robot_px, robot_py):
            return False
        self._render_node(node)
        return True

    def rerender_all(self, nodes: list[PoseNode]) -> None:
        if not nodes:
            return
        all_x = [n.x for n in nodes]
        all_y = [n.y for n in nodes]
        new_origin_x = min(all_x) - self._expansion_margin
        new_origin_y = min(all_y) - self._expansion_margin
        new_max_x = max(all_x) + self._expansion_margin
        new_max_y = max(all_y) + self._expansion_margin
        new_size = max(
            math.ceil((new_max_x - new_origin_x) / self._resolution),
            math.ceil((new_max_y - new_origin_y) / self._resolution),
        )
        _logger.info(
            f'Map recomputed: size={new_size}px '
            f'({new_size * self._resolution:.0f}m), '
            f'origin=({new_origin_x:.1f}, {new_origin_y:.1f})'
        )
        self._origin_x = new_origin_x
        self._origin_y = new_origin_y
        self._map_size = new_size
        self._map = np.full((new_size, new_size), 128, dtype=np.uint8)
        self._render_count = 0
        for node in nodes:
            if node.scan is not None:
                self._render_node(node)

    def to_occupancy_array(self) -> tuple[np.ndarray, float, float, float]:
        data = np.where(
            self._map == 128, -1,
            np.where(self._map == 255, 0, 100)
        ).astype(np.int8)
        return data, self._origin_x, self._origin_y, self._resolution

    def _world_to_pixel(self, wx: float, wy: float) -> tuple[int, int]:
        px = int((wx - self._origin_x) / self._resolution)
        py = int((wy - self._origin_y) / self._resolution)
        return px, py

    def _in_bounds(self, px: int, py: int) -> bool:
        return 0 <= px < self._map_size and 0 <= py < self._map_size

    def _render_node(self, node: PoseNode) -> None:
        scan = node.scan
        angles = scan.angle_min + \
            np.arange(len(scan.ranges)) * scan.angle_increment
        valid_mask = (scan.ranges > scan.range_min) & (
            scan.ranges < scan.range_max)

        cos_yaw = np.cos(node.yaw)
        sin_yaw = np.sin(node.yaw)

        robot_px, robot_py = self._world_to_pixel(node.x, node.y)
        if not self._in_bounds(robot_px, robot_py):
            return

        self._render_count += 1
        valid_indices = np.where(valid_mask)[0]
        hit_count = 0
        oob_hits = 0
        for i in valid_indices:
            r = float(scan.ranges[i])
            a = float(angles[i])
            lx = r * np.cos(a)
            ly = r * np.sin(a)
            wx = node.x + cos_yaw * lx - sin_yaw * ly
            wy = node.y + sin_yaw * lx + cos_yaw * ly

            hit_px, hit_py = self._world_to_pixel(wx, wy)
            if not self._in_bounds(hit_px, hit_py):
                oob_hits += 1
                continue

            hit_count += 1
            cv2.line(self._map, (robot_px, robot_py), (hit_px, hit_py), 255, 1)
            self._map[hit_py, hit_px] = 0

        if self._render_count == 1:
            _logger.info(
                f'First render: robot=({node.x:.2f}, {node.y:.2f}), '
                f'hits={hit_count}, oob_hits={oob_hits}'
            )
        elif self._render_count % 10 == 0:
            _logger.info(
                f'Render #{self._render_count}: '
                f'robot=({node.x:.2f}, {node.y:.2f}), '
                f'hits={hit_count}, oob_hits={oob_hits}'
            )
