#!/usr/bin/env python3
"""slam_gnss_2d ROS2 エントリポイント。

どのコンポーネント実装を組み合わせるかをここで決定する。
コアロジック（pose_graph / map_manager）は ROS に非依存。
"""
from __future__ import annotations

import logging

import rclpy

from slam_gnss_2d.core.config import SlamConfig
from slam_gnss_2d.input.base import GnssSourceBase, OdomSourceBase, ScanSourceBase
from slam_gnss_2d.input.ros2.ros_adapter import (
    ROS2GnssUtmSource,
    ROS2NavpvtSource,
    ROS2OdomSource,
    ROS2ScanSource,
)
from slam_gnss_2d.core.slam_node_base import SlamNodeBase


class SlamGnss2DNode(SlamNodeBase):
    def __init__(self) -> None:
        super().__init__('slam_gnss_2d_node')

    def _setup_io(self, cfg: SlamConfig):
        from slam_gnss_2d.input.ros2.ros_adapter import ROS2OdomSource, ROS2ScanSource
        return ROS2ScanSource(self, cfg.topics.scan), ROS2OdomSource(self, cfg.topics.odom)

    def _setup_gnss_source(self, cfg: SlamConfig):
        from slam_gnss_2d.input.ros2.ros_adapter import ROS2GnssUtmSource, ROS2NavpvtSource
        if cfg.gnss.source == 'navpvt':
            return ROS2NavpvtSource(
                self,
                topic=cfg.gnss.topics.navpvt,
                hacc_scale=cfg.gnss.navpvt_hacc_scale
            )
        else:
            return ROS2GnssUtmSource(self, cfg.gnss.topics.fix)


def main(args=None):
    logging.basicConfig(level=logging.INFO,
                        format='%(name)s %(levelname)s: %(message)s')
    rclpy.init(args=args)
    node = SlamGnss2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finalize()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
