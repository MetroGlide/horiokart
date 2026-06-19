#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import os
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point as RosPoint
from std_msgs.msg import ColorRGBA

class PoseGraphPreviewNode(Node):
    def __init__(self):
        super().__init__('pose_graph_preview_node')
        self.declare_parameter('pose_graph_file', '')
        
        self.pose_graph_file = self.get_parameter('pose_graph_file').value
        self.publisher = self.create_publisher(MarkerArray, 'slam_gnss_2d/pose_graph', 1)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.marker_array = None

        if self.pose_graph_file and os.path.exists(self.pose_graph_file):
            self.get_logger().info(f"Loading pose graph from {self.pose_graph_file}")
            self.load_pose_graph()
        else:
            self.get_logger().warn(f"pose_graph_file not found or empty: {self.pose_graph_file}")

    def load_pose_graph(self):
        try:
            with open(self.pose_graph_file, 'r') as f:
                data = json.load(f)
            
            nodes = data.get('nodes', [])
            seq_edges = data.get('sequential_edges', [])
            loop_edges = data.get('loop_edges', [])
            
            if not nodes:
                return

            node_by_idx = {n['index']: n for n in nodes}
            
            array = MarkerArray()
            
            # Nodes Marker
            node_m = Marker()
            node_m.header.frame_id = 'map'
            node_m.ns = 'nodes'
            node_m.id = 0
            node_m.type = Marker.SPHERE_LIST
            node_m.action = Marker.ADD
            node_m.scale.x = node_m.scale.y = node_m.scale.z = 0.2
            
            latest_idx = nodes[-1]['index'] if nodes else -1
            
            for n in nodes:
                pt = RosPoint()
                pt.x, pt.y, pt.z = float(n['x']), float(n['y']), 0.0
                node_m.points.append(pt)
                
                c = ColorRGBA()
                c.a = 1.0
                if n['index'] == latest_idx:
                    c.r, c.g, c.b = 1.0, 0.0, 0.0  # Red
                else:
                    c.r, c.g, c.b = 0.0, 0.0, 1.0  # Blue
                node_m.colors.append(c)
                
            array.markers.append(node_m)

            # Edges Marker
            edge_m = Marker()
            edge_m.header.frame_id = 'map'
            edge_m.ns = 'edges'
            edge_m.id = 1
            edge_m.type = Marker.LINE_LIST
            edge_m.action = Marker.ADD
            edge_m.scale.x = 0.05
            
            all_edges = seq_edges + loop_edges
            loop_edge_set = {(e['from'], e['to']) for e in loop_edges}
            
            for e in all_edges:
                fi, ti = e['from'], e['to']
                if fi not in node_by_idx or ti not in node_by_idx:
                    continue
                
                fn, tn = node_by_idx[fi], node_by_idx[ti]
                pt1 = RosPoint()
                pt1.x, pt1.y, pt1.z = float(fn['x']), float(fn['y']), 0.0
                pt2 = RosPoint()
                pt2.x, pt2.y, pt2.z = float(tn['x']), float(tn['y']), 0.0
                
                edge_m.points.extend([pt1, pt2])
                
                c = ColorRGBA()
                c.a = 1.0
                if (fi, ti) in loop_edge_set or (ti, fi) in loop_edge_set:
                    c.r, c.g, c.b = 1.0, 0.5, 0.0  # Orange
                else:
                    c.r, c.g, c.b = 0.0, 1.0, 0.0  # Green
                edge_m.colors.extend([c, c])
                
            array.markers.append(edge_m)
            self.marker_array = array
            self.get_logger().info(f"Loaded {len(nodes)} nodes, {len(seq_edges)} seq edges, {len(loop_edges)} loop edges.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load pose_graph.json: {e}")

    def timer_callback(self):
        if self.marker_array is not None:
            now = self.get_clock().now().to_msg()
            for m in self.marker_array.markers:
                m.header.stamp = now
            self.publisher.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PoseGraphPreviewNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
