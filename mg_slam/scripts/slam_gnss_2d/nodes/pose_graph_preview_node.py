#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import os
import yaml
import cv2
import numpy as np

from mg_msgs.msg import PoseGraphDiff
from mg_msgs.srv import GetPoseGraph
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class PoseGraphPreviewNode(Node):
    def __init__(self):
        super().__init__('pose_graph_preview_node')
        self.declare_parameter('pose_graph_file', '')
        
        self.pose_graph_file = self.get_parameter('pose_graph_file').value
        
        self._srv = self.create_service(
            GetPoseGraph, 'slam_gnss_2d/get_pose_graph', self._handle_get_pose_graph
        )
        self._diff_pub = self.create_publisher(PoseGraphDiff, 'slam_gnss_2d/pose_graph_diff', 1)
        
        qos_latch = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', qos_latch)
        
        self._graph_diff = PoseGraphDiff()
        self._graph_diff.full_refresh_needed = True
        self._map_msg = None
        
        if self.pose_graph_file and os.path.exists(self.pose_graph_file):
            self.get_logger().info(f"Loading pose graph from {self.pose_graph_file}")
            self.load_pose_graph()
            self.load_map()
            
            # 定期的にPublishするか、一度だけTimerでPublishするか
            self.timer = self.create_timer(2.0, self._publish_initial_data)
        else:
            self.get_logger().warn(f"pose_graph_file not found or empty: {self.pose_graph_file}")

    def load_pose_graph(self):
        try:
            with open(self.pose_graph_file, 'r') as f:
                data = json.load(f)
            
            nodes = data.get('nodes', [])
            seq_edges = data.get('sequential_edges', [])
            loop_edges = data.get('loop_edges', [])
            
            for n in nodes:
                self._graph_diff.new_node_indices.append(n.get('index', 0))
                self._graph_diff.new_node_x.append(float(n.get('x', 0.0)))
                self._graph_diff.new_node_y.append(float(n.get('y', 0.0)))
                self._graph_diff.new_node_yaw.append(float(n.get('yaw', 0.0)))
                self._graph_diff.new_node_timestamps.append(float(n.get('timestamp', 0.0)))
                
            for e in seq_edges:
                self._graph_diff.seq_edge_from.append(e.get('from', 0))
                self._graph_diff.seq_edge_to.append(e.get('to', 0))
                self._graph_diff.seq_edge_type.append(1 if e.get('is_odom_fallback', False) else 0)
                self._graph_diff.seq_edge_score.append(float(e.get('score', 0.0)))
                self._graph_diff.seq_edge_info_diag.extend([0.0, 0.0, 0.0])

            for e in loop_edges:
                self._graph_diff.loop_edge_from.append(e.get('from', 0))
                self._graph_diff.loop_edge_to.append(e.get('to', 0))
                self._graph_diff.loop_edge_score.append(float(e.get('score', 0.0)))
                self._graph_diff.loop_edge_info_diag.extend([0.0, 0.0, 0.0])
                
            self.get_logger().info(f"Loaded {len(nodes)} nodes, {len(seq_edges)} seq edges, {len(loop_edges)} loop edges.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load pose_graph.json: {e}")

    def load_map(self):
        try:
            map_dir = os.path.dirname(self.pose_graph_file)
            map_yaml_path = os.path.join(map_dir, 'map.yaml')
            
            if not os.path.exists(map_yaml_path):
                self.get_logger().warn(f"map.yaml not found at {map_yaml_path}")
                return
                
            with open(map_yaml_path, 'r') as f:
                map_info = yaml.safe_load(f)
                
            image_path = os.path.join(map_dir, map_info['image'])
            if not os.path.exists(image_path):
                self.get_logger().warn(f"Map image not found at {image_path}")
                return
                
            img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                self.get_logger().error(f"Failed to read image {image_path}")
                return
                
            # OpenCVは左上が原点、ROSのOccupancyGridは左下が原点なので上下反転
            img_flipped = np.flipud(img)
            
            occ_th = map_info.get('occupied_thresh', 0.65)
            free_th = map_info.get('free_thresh', 0.196)
            negate = map_info.get('negate', 0)
            
            # 画像のピクセル値(0-255)を確率(0.0-1.0)に変換
            # 黒(0) -> val=0.0 -> occ=1.0 (Occupied)
            # 白(255) -> val=1.0 -> occ=0.0 (Free)
            val = img_flipped.astype(np.float32) / 255.0
            if negate:
                val = 1.0 - val
            occ = 1.0 - val
            
            grid_data = np.full(img_flipped.shape, -1, dtype=np.int8)
            grid_data[occ > occ_th] = 100
            grid_data[occ < free_th] = 0
            
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.info.resolution = float(map_info['resolution'])
            msg.info.width = img_flipped.shape[1]
            msg.info.height = img_flipped.shape[0]
            msg.info.origin.position.x = float(map_info['origin'][0])
            msg.info.origin.position.y = float(map_info['origin'][1])
            msg.info.origin.position.z = float(map_info['origin'][2])
            msg.data = grid_data.flatten().tolist()
            
            self._map_msg = msg
            self.get_logger().info(f"Loaded map {image_path} ({msg.info.width}x{msg.info.height})")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")

    def _handle_get_pose_graph(self, request, response):
        response.graph = self._graph_diff
        response.total_nodes = len(self._graph_diff.new_node_indices)
        response.total_seq_edges = len(self._graph_diff.seq_edge_from)
        response.total_loop_edges = len(self._graph_diff.loop_edge_from)
        response.icp_attempt_count = 0
        response.icp_success_count = 0
        response.odom_fallback_count = sum(1 for t in self._graph_diff.seq_edge_type if t == 1)
        response.loop_attempt_count = 0
        response.loop_success_count = 0
        return response

    def _publish_initial_data(self):
        self._diff_pub.publish(self._graph_diff)
        if self._map_msg:
            self._map_msg.header.stamp = self.get_clock().now().to_msg()
            self._map_pub.publish(self._map_msg)
            self.get_logger().info("Published initial pose graph and map.")
        else:
            self.get_logger().info("Published initial pose graph (no map).")
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphPreviewNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
