#!/usr/bin/env python3
"""slam_gnss_2d ROS2 エントリポイント。

どのコンポーネント実装を組み合わせるかをここで決定する。
コアロジック（pose_graph / map_manager）は ROS に非依存。
"""
from __future__ import annotations

import logging
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from slam_gnss_2d.component_factory import build_pose_graph_builder
from slam_gnss_2d.data_types import ScanData
from slam_gnss_2d.input.ros2.ros_adapter import ROS2OdomSource, ROS2ScanSource
from slam_gnss_2d.map_manager.opencv_renderer import OpenCVRenderer


class SlamGnss2DNode(Node):
    def __init__(self) -> None:
        super().__init__('slam_gnss_2d_node')
        self._declare_params()

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        resolution = self.get_parameter('map_resolution').value
        expansion_margin = self.get_parameter('map_expansion_margin').value
        min_trans = self.get_parameter('min_translation').value
        min_rot = self.get_parameter('min_rotation').value
        map_publish_hz = self.get_parameter('map_publish_hz').value
        builder_type = self.get_parameter('pose_graph_builder').value
        icp_max_iter = self.get_parameter('icp_max_iterations').value
        icp_tol = self.get_parameter('icp_tolerance').value
        icp_max_dist = self.get_parameter('icp_max_correspondence_dist').value

        self._scan_source = ROS2ScanSource(self, scan_topic)
        self._odom_source = ROS2OdomSource(self, odom_topic)
        self._pose_graph = build_pose_graph_builder(
            builder_type=builder_type,
            min_translation=min_trans,
            min_rotation=min_rot,
            icp_max_iterations=icp_max_iter,
            icp_tolerance=icp_tol,
            icp_max_correspondence_dist=icp_max_dist,
        )
        self._renderer = OpenCVRenderer(
            resolution=resolution,
            expansion_margin=expansion_margin,
        )

        self._map_pub = self.create_publisher(
            OccupancyGrid, 'slam_gnss_2d/map', 1)
        self._path_pub = self.create_publisher(Path, 'slam_gnss_2d/path', 1)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._path_msg = Path()
        self._path_msg.header.frame_id = 'map'
        self._map_dirty = False
        self._map_to_odom_x = 0.0
        self._map_to_odom_y = 0.0
        self._map_to_odom_yaw = 0.0

        self._scan_source.set_scan_callback(self._on_scan)
        self._odom_source.start()
        self._scan_source.start()

        self.create_timer(1.0 / map_publish_hz, self._publish_map_timer)
        self.create_timer(0.1, self._publish_tf)

        self._scan_recv_count = 0
        self._odom_miss_count = 0
        self._node_count = 0
        self._last_stat_time = time.monotonic()

        self.get_logger().info(
            f'slam_gnss_2d_node started (builder={builder_type})\n'
            f'  scan: {scan_topic}, odom: {odom_topic}\n'
            f'  map: dynamic @ {resolution}m/px, margin={expansion_margin}m'
        )

    def _declare_params(self) -> None:
        self.declare_parameter('scan_topic', '/scan_top_lidar')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_expansion_margin', 100.0)
        self.declare_parameter('min_translation', 0.3)
        self.declare_parameter('min_rotation', 0.1)
        self.declare_parameter('map_publish_hz', 1.0)
        self.declare_parameter('pose_graph_builder', 'scan_matching')
        self.declare_parameter('icp_max_iterations', 30)
        self.declare_parameter('icp_tolerance', 1e-4)
        self.declare_parameter('icp_max_correspondence_dist', 0.5)

    def _on_scan(self, scan: ScanData) -> None:
        self._scan_recv_count += 1
        odom = self._odom_source.get_odom_at(scan.timestamp)
        if odom is None:
            self._odom_miss_count += 1
            self.get_logger().warn(
                f'No odom for scan ts={scan.timestamp:.3f} (miss #{self._odom_miss_count})'
            )
            return

        node = self._pose_graph.add_scan(scan, odom)
        if node is None:
            self.get_logger().debug(
                f'Scan #{self._scan_recv_count} rejected: '
                f'below threshold at ({odom.x:.2f}, {odom.y:.2f})'
            )
            return

        self._update_map_to_odom(node, odom)
        self._node_count += 1
        if self._node_count == 1 or self._node_count % 10 == 0:
            self.get_logger().info(
                f'Node #{node.index}: x={node.x:.2f} y={node.y:.2f} '
                f'yaw={math.degrees(node.yaw):.1f}deg'
            )

        if not self._renderer.add_node(node):
            self._renderer.rerender_all(self._pose_graph.get_nodes())
        self._map_dirty = True
        self._publish_path(node)

    def _publish_map_timer(self) -> None:
        now = time.monotonic()
        if now - self._last_stat_time >= 30.0:
            self.get_logger().info(
                f'[stat] nodes={self._node_count}, '
                f'scans={self._scan_recv_count}, '
                f'odom_miss={self._odom_miss_count}'
            )
            self._last_stat_time = now

        if not self._map_dirty:
            return
        self._map_dirty = False

        data, origin_x, origin_y, resolution = self._renderer.to_occupancy_array()
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = resolution
        msg.info.width = int(data.shape[1])
        msg.info.height = int(data.shape[0])
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.data = data.flatten().tolist()
        self._map_pub.publish(msg)
        self.get_logger().debug(
            f'Map published: occupied={int((data == 100).sum())}, '
            f'free={int((data == 0).sum())} px'
        )

    def _update_map_to_odom(self, node, odom) -> None:
        """最新キーフレームから map->odom 変換を更新する。

        map_T_odom = map_T_base * inv(odom_T_base)
        """
        c_o = math.cos(odom.yaw)
        s_o = math.sin(odom.yaw)
        inv_x = -(c_o * odom.x + s_o * odom.y)
        inv_y = -(-s_o * odom.x + c_o * odom.y)
        c_m = math.cos(node.yaw)
        s_m = math.sin(node.yaw)
        self._map_to_odom_x = node.x + c_m * inv_x - s_m * inv_y
        self._map_to_odom_y = node.y + s_m * inv_x + c_m * inv_y
        self._map_to_odom_yaw = node.yaw - odom.yaw

    def _publish_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self._map_to_odom_x
        t.transform.translation.y = self._map_to_odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = math.cos(self._map_to_odom_yaw / 2.0)
        t.transform.rotation.z = math.sin(self._map_to_odom_yaw / 2.0)
        self._tf_broadcaster.sendTransform(t)

    def _publish_path(self, node) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = node.x
        pose.pose.position.y = node.y
        pose.pose.orientation.w = math.cos(node.yaw / 2.0)
        pose.pose.orientation.z = math.sin(node.yaw / 2.0)
        self._path_msg.header.stamp = pose.header.stamp
        self._path_msg.poses.append(pose)
        self._path_pub.publish(self._path_msg)

    def destroy_node(self) -> None:
        self._scan_source.stop()
        self._odom_source.stop()
        super().destroy_node()


def main(args=None):
    logging.basicConfig(level=logging.INFO,
                        format='%(name)s %(levelname)s: %(message)s')
    rclpy.init(args=args)
    node = SlamGnss2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
