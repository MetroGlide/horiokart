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
        xs = np.fromiter((n.x for n in nodes),
                         dtype=np.float64, count=len(nodes))
        ys = np.fromiter((n.y for n in nodes),
                         dtype=np.float64, count=len(nodes))
        new_origin_x = float(xs.min()) - self._expansion_margin
        new_origin_y = float(ys.min()) - self._expansion_margin
        new_max_x = float(xs.max()) + self._expansion_margin
        new_max_y = float(ys.max()) + self._expansion_margin
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
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid_mask = (ranges > scan.range_min) & (ranges < scan.range_max)

        cos_yaw = np.cos(node.yaw)
        sin_yaw = np.sin(node.yaw)

        robot_px, robot_py = self._world_to_pixel(node.x, node.y)
        if not self._in_bounds(robot_px, robot_py):
            return

        self._render_count += 1

        # 有効点の座標変換を一括計算
        r_v = ranges[valid_mask]
        a_v = angles[valid_mask]
        lx = r_v * np.cos(a_v)
        ly = r_v * np.sin(a_v)
        wx = node.x + cos_yaw * lx - sin_yaw * ly
        wy = node.y + sin_yaw * lx + cos_yaw * ly

        hit_px = ((wx - self._origin_x) / self._resolution).astype(np.int32)
        hit_py = ((wy - self._origin_y) / self._resolution).astype(np.int32)
        in_bounds = (
            (hit_px >= 0) & (hit_px < self._map_size) &
            (hit_py >= 0) & (hit_py < self._map_size)
        )
        hit_px_valid = hit_px[in_bounds]
        hit_py_valid = hit_py[in_bounds]

        hit_px_valid = hit_px[in_bounds]
        hit_py_valid = hit_py[in_bounds]
        n_hits = len(hit_px_valid)

        if n_hits > 0:
            # free ライン描画: N 本の 2 点ラインを cv2.polylines で C++ 側に一括委譲
            # shape (N, 2, 1, 2): pts[i] は (start, end) の 2 点ポリライン
            pts = np.empty((n_hits, 2, 1, 2), dtype=np.int32)
            pts[:, 0, 0, 0] = robot_px
            pts[:, 0, 0, 1] = robot_py
            pts[:, 1, 0, 0] = hit_px_valid
            pts[:, 1, 0, 1] = hit_py_valid
            cv2.polylines(self._map, pts, False, 255, 1)

            # occupied 点を一括で書き込む
            self._map[hit_py_valid, hit_px_valid] = 0

        if self._render_count == 1:
            _logger.info(
                f'First render: robot=({node.x:.2f}, {node.y:.2f}), '
                f'hits={n_hits}, oob_hits={int((~in_bounds).sum())}'
            )
        elif self._render_count % 10 == 0:
            _logger.info(
                f'Render #{self._render_count}: '
                f'robot=({node.x:.2f}, {node.y:.2f}), '
                f'hits={n_hits}, oob_hits={int((~in_bounds).sum())}'
            )
