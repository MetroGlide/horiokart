from __future__ import annotations

import numpy as np
import cv2

from .base import MapRendererBase
from ..data_types import PoseNode


class OpenCVRenderer(MapRendererBase):
    """OpenCV cv2.line を使った Ray-Casting による占有格子マップ実装。

    内部マップは uint8 で管理する:
        128 = unknown
        255 = free（cv2.line で空き領域を描画）
          0 = occupied（ヒット点を上書き）

    to_occupancy_array() で ROS2 OccupancyGrid 形式（-1 / 0 / 100）に変換して返す。
    """

    def __init__(
        self,
        resolution: float = 0.05,
        map_size: int = 2000,
        origin_x: float = -50.0,
        origin_y: float = -50.0,
    ) -> None:
        """
        Args:
            resolution: マップの解像度 [m/pixel]
            map_size: マップの一辺のピクセル数（正方形）
            origin_x: マップ左下隅のワールド座標 X [m]
            origin_y: マップ左下隅のワールド座標 Y [m]
        """
        self._resolution = resolution
        self._map_size = map_size
        self._origin_x = origin_x
        self._origin_y = origin_y
        self._map = np.full((map_size, map_size), 128, dtype=np.uint8)

    def add_node(self, node: PoseNode) -> None:
        if node.scan is None:
            return
        self._render_node(node)

    def rerender_all(self, nodes: list[PoseNode]) -> None:
        self._map.fill(128)
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

        valid_indices = np.where(valid_mask)[0]
        for i in valid_indices:
            r = float(scan.ranges[i])
            a = float(angles[i])
            lx = r * np.cos(a)
            ly = r * np.sin(a)
            wx = node.x + cos_yaw * lx - sin_yaw * ly
            wy = node.y + sin_yaw * lx + cos_yaw * ly

            hit_px, hit_py = self._world_to_pixel(wx, wy)
            if not self._in_bounds(hit_px, hit_py):
                continue

            cv2.line(self._map, (robot_px, robot_py), (hit_px, hit_py), 255, 1)
            self._map[hit_py, hit_px] = 0
