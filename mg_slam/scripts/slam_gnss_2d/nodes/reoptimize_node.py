#!/usr/bin/env python3
import os
import sys
import json
import math
import array
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point as RosPoint
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger

# scripts/slam_gnss_2d 階層を python パスに通す
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from slam_gnss_2d.core.config import SlamConfig
from slam_gnss_2d.core.data_types import PoseNode, PoseEdge, GnssPrior, ScanData, OdomData, GnssData
from slam_gnss_2d.core.component_factory import build_gnss_source, _build_matcher, _build_loop_matcher
from slam_gnss_2d.optimizer.gtsam_optimizer import GTSAMOptimizer
from slam_gnss_2d.map_manager import OverwriteRenderer, CountingRenderer
from slam_gnss_2d.core.slam_data_saver import SlamDataSaver
from slam_gnss_2d.input.ros2.bag_reader import BagScanSource
from slam_gnss_2d.core.pose_graph_reoptimizer import PoseGraphReoptimizer


class ReoptimizeNode(Node):
    def __init__(self) -> None:
        super().__init__('reoptimize_node')
        self.get_logger().info("Initializing reoptimize_node...")

        # Parameters
        self.declare_parameter('input_dir', '')
        self.declare_parameter('bag_path', '')
        self.declare_parameter('save_dir', '')
        self.declare_parameter('enable_re_scan_matching', False)
        self.declare_parameter('params_file', '')

        # Setup publishers
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, 'map', map_qos)
        self._path_pub = self.create_publisher(Path, 'slam_gnss_2d/path', 1)
        self._pg_marker_pub = self.create_publisher(MarkerArray, 'slam_gnss_2d/pose_graph', 1)
        self._path_before_pub = self.create_publisher(Path, 'slam_gnss_2d/path_before_optimize', 1)
        self._anchor_pub = self.create_publisher(NavSatFix, 'slam_gnss_2d/anchor', map_qos)

        # Service
        self._save_srv = self.create_service(
            Trigger, 'slam_gnss_2d/save_slam_map', self._handle_save_slam_map)

        # Member variables for keeping state
        self._optimized_nodes = []
        self._new_edges = []
        self._gnss_transform_data = {}
        self._bag_path = ""
        self._renderer = None
        self._config = None

        # Start optimization via a timer to run outside __init__
        self._timer = self.create_timer(0.5, self._run_optimization)

    def _run_optimization(self) -> None:
        self._timer.cancel()
        
        input_dir = self.get_parameter('input_dir').value
        if not input_dir:
            self.get_logger().error("Parameter 'input_dir' is required but not specified!")
            return

        self.get_logger().info(f"Starting reoptimization. Input directory: {input_dir}")
        pose_graph_path = os.path.join(input_dir, "pose_graph.json")
        gnss_transform_path = os.path.join(input_dir, "gnss_transform.yaml")

        if not os.path.exists(pose_graph_path):
            self.get_logger().error(f"pose_graph.json not found in {input_dir}!")
            return
        if not os.path.exists(gnss_transform_path):
            self.get_logger().error(f"gnss_transform.yaml not found in {input_dir}!")
            return

        try:
            with open(pose_graph_path, 'r') as f:
                pose_graph_data = json.load(f)
            with open(gnss_transform_path, 'r') as f:
                self._gnss_transform_data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load input files: {e}")
            return

        # Resolve Bag Path
        bag_path = self.get_parameter('bag_path').value
        if not bag_path:
            bag_path = pose_graph_data.get("metadata", {}).get("bag_path", "")

        if not bag_path or not os.path.exists(bag_path):
            self.get_logger().error(f"ROS Bag path '{bag_path}' is invalid or file does not exist!")
            return
        
        self._bag_path = bag_path
        self.get_logger().info(f"Using ROS Bag: {bag_path}")

        # Resolve Config File
        config_file = self.get_parameter('params_file').value
        if not config_file:
            # Fallback to default
            # SCRIPT_DIR is mg_slam/scripts/slam_gnss_2d/nodes
            default_yaml = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR))), "params", "slam_gnss_2d.yaml")
            if os.path.exists(default_yaml):
                config_file = default_yaml
                self.get_logger().info(f"Using default config file: {config_file}")

        # Load config
        if config_file and os.path.exists(config_file):
            from slam_gnss_2d.tools.reoptimize_pose_graph import load_config_from_yaml
            config = load_config_from_yaml(config_file)
        else:
            config = SlamConfig()
            self.get_logger().info("Using built-in default config parameters.")
        self._config = config

        # Run Optimization using the core reoptimizer
        reoptimizer = PoseGraphReoptimizer(config, self.get_logger())
        
        def on_scans_extracted(old_nodes):
            self._publish_path_before(old_nodes)
            
        enable_re_scan_matching = self.get_parameter('enable_re_scan_matching').value
        self.get_logger().info(f"enable_re_scan_matching: {enable_re_scan_matching}")
            
        self._optimized_nodes, self._new_edges, self._renderer = reoptimizer.reoptimize(
            pose_graph_data, self._gnss_transform_data, bag_path,
            on_scans_extracted=on_scans_extracted,
            enable_re_scan_matching=enable_re_scan_matching
        )

        # Publish results
        self._publish_map()
        self._publish_path(self._optimized_nodes)
        loop_edges_data = pose_graph_data.get("loop_edges", [])
        self._publish_pose_graph_markers(self._optimized_nodes, self._new_edges, loop_edges_data)
        self._publish_anchor()

        self.get_logger().info("Re-optimization complete. Waiting for save service call...")

    def _publish_path(self, nodes):
        now = self.get_clock().now().to_msg()
        msg = Path()
        msg.header.stamp = now
        msg.header.frame_id = 'map'
        for n in nodes:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'map'
            pose.pose.position.x = n.x
            pose.pose.position.y = n.y
            pose.pose.orientation.w = math.cos(n.yaw / 2.0)
            pose.pose.orientation.z = math.sin(n.yaw / 2.0)
            msg.poses.append(pose)
        self._path_pub.publish(msg)

    def _publish_path_before(self, nodes):
        now = self.get_clock().now().to_msg()
        msg = Path()
        msg.header.stamp = now
        msg.header.frame_id = 'map'
        for n in nodes:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'map'
            pose.pose.position.x = n.x
            pose.pose.position.y = n.y
            pose.pose.orientation.w = math.cos(n.yaw / 2.0)
            pose.pose.orientation.z = math.sin(n.yaw / 2.0)
            msg.poses.append(pose)
        self._path_before_pub.publish(msg)

    def _publish_pose_graph_markers(self, nodes, edges, loop_edges_data):
        if not nodes:
            return
        loop_edge_set = {(e["from"], e["to"]) for e in loop_edges_data}
        node_by_idx = {n.index: n for n in nodes}
        now = self.get_clock().now().to_msg()
        array_msg = MarkerArray()

        node_m = Marker()
        node_m.header.stamp = now
        node_m.header.frame_id = 'map'
        node_m.ns = 'nodes'
        node_m.id = 0
        node_m.type = Marker.SPHERE_LIST
        node_m.action = Marker.ADD
        node_m.scale.x = node_m.scale.y = node_m.scale.z = 0.2
        node_m.color.r = node_m.color.g = node_m.color.b = node_m.color.a = 1.0
        latest_idx = nodes[-1].index
        for n in nodes:
            pt = RosPoint()
            pt.x, pt.y, pt.z = n.x, n.y, 0.0
            node_m.points.append(pt)
            c = ColorRGBA()
            if n.index == latest_idx:
                c.r, c.g, c.b, c.a = 0.0, 1.0, 1.0, 1.0
            else:
                c.r, c.g, c.b, c.a = 1.0, 1.0, 1.0, 0.8
            node_m.colors.append(c)
        array_msg.markers.append(node_m)

        seq_m = Marker()
        seq_m.header.stamp = now
        seq_m.header.frame_id = 'map'
        seq_m.ns = 'seq_edges'
        seq_m.id = 1
        seq_m.type = Marker.LINE_LIST
        seq_m.action = Marker.ADD
        seq_m.scale.x = 0.05
        seq_m.color.r, seq_m.color.g = 0.2, 0.5
        seq_m.color.b, seq_m.color.a = 1.0, 0.9

        loop_m = Marker()
        loop_m.header.stamp = now
        loop_m.header.frame_id = 'map'
        loop_m.ns = 'loop_edges'
        loop_m.id = 2
        loop_m.type = Marker.LINE_LIST
        loop_m.action = Marker.ADD
        loop_m.scale.x = 0.08
        loop_m.color.r, loop_m.color.g = 0.0, 1.0
        loop_m.color.b, loop_m.color.a = 0.4, 1.0

        for edge in edges:
            p0 = node_by_idx.get(edge.from_index)
            p1 = node_by_idx.get(edge.to_index)
            if p0 is None or p1 is None:
                continue
            is_loop = (edge.from_index, edge.to_index) in loop_edge_set
            target = loop_m if is_loop else seq_m
            pt_a = RosPoint()
            pt_a.x, pt_a.y, pt_a.z = p0.x, p0.y, 0.0
            pt_b = RosPoint()
            pt_b.x, pt_b.y, pt_b.z = p1.x, p1.y, 0.0
            target.points.append(pt_a)
            target.points.append(pt_b)
        
        array_msg.markers.append(seq_m)
        array_msg.markers.append(loop_m)
        self._pg_marker_pub.publish(array_msg)

    def _publish_map(self):
        if self._renderer is None:
            return
        data, origin_x, origin_y, resolution = self._renderer.to_occupancy_array()
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = resolution
        msg.info.width = int(data.shape[1])
        msg.info.height = int(data.shape[0])
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.data = array.array('b', data.ravel().tobytes())
        self._map_pub.publish(msg)

    def _publish_anchor(self):
        if 'anchor' not in self._gnss_transform_data:
            return
        lat = self._gnss_transform_data['anchor'].get('latitude', 0.0)
        lon = self._gnss_transform_data['anchor'].get('longitude', 0.0)
        
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        self._anchor_pub.publish(msg)
        self.get_logger().info(f"Published anchor: lat={lat}, lon={lon}")

    def _handle_save_slam_map(self, request, response):
        output_dir = self.get_parameter('save_dir').value
        if not output_dir:
            import datetime
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = f"/root/ros2_data/slam_maps/{timestamp}_opt"

        self.get_logger().info(f"Saving optimized results to directory: {output_dir}")
        try:
            os.makedirs(output_dir, exist_ok=True)
            
            # Save pose graph
            SlamDataSaver.save_pose_graph(output_dir, self._optimized_nodes, self._new_edges, bag_path=self._bag_path)
            
            # Save GNSS transform
            if self._gnss_transform_data:
                lat = self._gnss_transform_data.get('anchor', {}).get('latitude', 0.0)
                lon = self._gnss_transform_data.get('anchor', {}).get('longitude', 0.0)
                easting = self._gnss_transform_data.get('anchor_utm', {}).get('easting', 0.0)
                northing = self._gnss_transform_data.get('anchor_utm', {}).get('northing', 0.0)
                zone = self._gnss_transform_data.get('anchor_utm', {}).get('zone', 54)
                hemisphere = self._gnss_transform_data.get('anchor_utm', {}).get('hemisphere', 'north')
                rotation_rad = self._gnss_transform_data.get('rotation_rad', 0.0)
                
                SlamDataSaver.save_gnss_transform(
                    output_dir=output_dir,
                    anchor_lat=lat,
                    anchor_lon=lon,
                    anchor_utm_easting=easting,
                    anchor_utm_northing=northing,
                    utm_zone=zone,
                    utm_hemisphere=hemisphere,
                    rotation_rad=rotation_rad,
                    backend_name="gtsam_batch"
                )

            # Save OccupancyGrid image
            from slam_gnss_2d.tools.reoptimize_pose_graph import save_map_pgm_and_yaml
            save_map_pgm_and_yaml(output_dir, self._renderer)
            
            response.success = True
            response.message = f"Optimized map saved to {output_dir}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save optimized map: {e}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    import logging
    logging.basicConfig(level=logging.INFO, format='%(name)s %(levelname)s: %(message)s')
    rclpy.init(args=args)
    node = ReoptimizeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
