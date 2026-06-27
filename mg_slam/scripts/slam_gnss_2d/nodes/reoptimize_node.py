#!/usr/bin/env python3
from slam_gnss_2d.ros.slam_visualizer import SlamVisualizer
from slam_gnss_2d.input.ros2.bag_reader import BagScanSource
from slam_gnss_2d.core.slam_data_saver import SlamDataSaver
from slam_gnss_2d.optimizer.gtsam_optimizer import GTSAMOptimizer
from slam_gnss_2d.scan_matching.base import ScanMatcherBase
from slam_gnss_2d.core.data_types import PoseNode, PoseEdge, GnssPrior, ScanData, OdomData, GnssData, MatchResult
from slam_gnss_2d.core.geometry import angle_diff, scan_to_points, world_delta_to_local
from slam_gnss_2d.core.component_factory import build_gnss_source, _build_matcher, _build_loop_matcher, build_renderer
from slam_gnss_2d.core.config_loader import ConfigLoader
from slam_gnss_2d.map_manager.trajectory_noise_filter import TrajectoryNoiseFilter
from slam_gnss_2d.tools.reoptimize_geometry import (
    build_submap_points,
    find_nearest_scan,
    sigma_from_covariance_or_status,
)
import os
import sys
import math
import datetime
import logging
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from slam_gnss_2d.tools.reoptimize_io import (
    load_pose_graph_and_transform,
    resolve_bag_path,
)
from slam_gnss_2d.tools.reoptimize_loop_search import search_new_loop_edges

# scripts/slam_gnss_2d 階層を python パスに通す
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))


class ReadOnlyPoseGraph:
    """最適化済みノード・エッジを PoseGraphBuilderBase 互換インターフェースで提供するアダプタ。
    ReoptimizeNode が SlamVisualizer を変更なしに再利用するために使用する。
    """

    def __init__(
        self,
        nodes: list[PoseNode],
        seq_edges: list[PoseEdge],
        loop_edges: list[PoseEdge],
    ) -> None:
        self._nodes = nodes
        self._seq_edges = seq_edges
        self._loop_edges = loop_edges

    def get_nodes(self) -> list[PoseNode]:
        return self._nodes

    def get_edges(self) -> list[PoseEdge]:
        return self._seq_edges + self._loop_edges

    def get_loop_edges(self) -> list[PoseEdge]:
        return self._loop_edges


class ReoptimizeNode(Node):
    def __init__(self) -> None:
        super().__init__('reoptimize_node')
        self.get_logger().info("Initializing reoptimize_node...")

        # Parameters
        self.declare_parameter('input_dir', '')
        self.declare_parameter('bag_path', '')
        self.declare_parameter('enable_re_scan_matching', False)
        self.declare_parameter('enable_new_loop_search', False)
        self.declare_parameter('multi_guess_shift_m', 0.2)
        self.declare_parameter('multi_guess_shift_deg', 5.0)

        # 既存の ConfigLoader を利用して params を宣言・ロード
        ConfigLoader.declare_params(self)
        self._config = ConfigLoader.build_config(self)

        self._visualizer = SlamVisualizer(self, use_gnss=True)

        # Service
        self._save_srv = self.create_service(
            Trigger, 'slam_gnss_2d/save_slam_map', self._handle_save_slam_map)

        # Member variables for keeping state
        self._optimized_nodes = []
        self._new_edges = []
        self._gnss_transform_data = {}
        self._bag_path = ""
        self._renderer = None
        self._orchestrator = None

        # Start optimization via a timer to run outside __init__
        self._timer = self.create_timer(0.5, self._run_optimization)

    def _match_with_multi_guess(
        self,
        matcher: ScanMatcherBase,
        src_pts: np.ndarray,
        dst_scan: ScanData,
        base_guess: OdomData,
        gnss_guess: OdomData | None,
        shift_m: float,
        shift_rad: float,
    ) -> MatchResult | None:
        """8候補の初期値でスキャンマッチングし、スコア最小の収束結果を返す。"""
        candidates = [
            base_guess,
            OdomData(base_guess.timestamp, base_guess.x + shift_m,
                     base_guess.y,            base_guess.yaw),
            OdomData(base_guess.timestamp, base_guess.x - shift_m,
                     base_guess.y,            base_guess.yaw),
            OdomData(base_guess.timestamp, base_guess.x,
                     base_guess.y + shift_m, base_guess.yaw),
            OdomData(base_guess.timestamp, base_guess.x,
                     base_guess.y - shift_m, base_guess.yaw),
            OdomData(base_guess.timestamp, base_guess.x,
                     base_guess.y,            base_guess.yaw + shift_rad),
            OdomData(base_guess.timestamp, base_guess.x,
                     base_guess.y,            base_guess.yaw - shift_rad),
        ]
        if gnss_guess is not None:
            candidates.append(gnss_guess)
            candidates.append(OdomData(
                gnss_guess.timestamp, gnss_guess.x, gnss_guess.y, gnss_guess.yaw + shift_rad))

        best_result = None
        best_score = float('inf')
        for i, guess in enumerate(candidates):
            result = matcher.match(dst=dst_scan, initial_guess=guess)
            if not result.converged:
                continue
            if result.score < best_score or (result.score == best_score and i == 0):
                best_score = result.score
                best_result = result
        return best_result

    def _search_new_loop_edges(
        self,
        optimized_nodes: list[PoseNode],
        node_scans: dict[int, ScanData],
        existing_loop_edges: list[PoseEdge],
        loop_matcher: ScanMatcherBase,
        config,
    ) -> list[PoseEdge]:
        return search_new_loop_edges(
            optimized_nodes=optimized_nodes,
            node_scans=node_scans,
            existing_loop_edges=existing_loop_edges,
            loop_matcher=loop_matcher,
            search_radius=config.loop_closure.search_radius,
            min_node_gap=config.loop_closure.min_node_gap,
            submap_radius=config.loop_closure.submap_radius,
            max_score=config.loop_closure.max_score,
            max_dyaw_deg=config.loop_closure.max_dyaw_deg,
            crossing_reject_deg=config.loop_closure.crossing_reject_deg,
            logger_info=self.get_logger().info,
        )

    def _run_optimization(self) -> None:
        self._timer.cancel()

        input_dir = self.get_parameter('input_dir').value
        if not input_dir:
            self.get_logger().error("Parameter 'input_dir' is required but not specified!")
            return

        self.get_logger().info(
            f"Starting reoptimization. Input directory: {input_dir}")
        try:
            pose_graph_data, self._gnss_transform_data = load_pose_graph_and_transform(
                input_dir)
        except Exception as e:
            self.get_logger().error(f"Failed to load input files: {e}")
            return

        try:
            bag_path = resolve_bag_path(
                self.get_parameter('bag_path').value,
                pose_graph_data,
            )
        except Exception as e:
            self.get_logger().error(str(e))
            return

        self._bag_path = bag_path
        self.get_logger().info(f"Using ROS Bag: {bag_path}")

        config = self._config
        self._renderer = build_renderer(config)

        # 1. Extract scans
        self.get_logger().info("Extracting LiDAR scans from ROS Bag...")
        scan_source = BagScanSource(bag_path, config.topics.scan)
        scan_source.start()

        all_scans = []

        def scan_callback(scan_data: ScanData):
            all_scans.append(scan_data)

        scan_source.set_scan_callback(scan_callback)
        scan_count = 0
        while scan_source.step():
            scan_count += 1
            if scan_count % 1000 == 0:
                self.get_logger().info(f"Loaded {scan_count} scans...")
        scan_source.stop()
        self.get_logger().info(
            f"Loaded total {len(all_scans)} scans from Bag.")

        # 2. Reconstruct old nodes
        self.get_logger().info("Matching nodes to nearest LiDAR scans...")
        old_nodes = []
        node_scans = {}
        for n_data in pose_graph_data["nodes"]:
            node_idx = n_data["index"]
            ts = n_data["timestamp"]
            scan = find_nearest_scan(all_scans, ts, max_diff=0.1)
            node = PoseNode(
                index=node_idx, timestamp=ts,
                x=n_data["x"], y=n_data["y"], yaw=n_data["yaw"],
                scan=scan
            )
            old_nodes.append(node)
            if scan is not None:
                node_scans[node_idx] = scan
            else:
                self.get_logger().warning(
                    f"No matching scan found for node {node_idx}")

        # Publish before optimize
        self._visualizer.rebuild_path(old_nodes)
        self._visualizer.publish_path_before_optimize()

        # 3. GNSS Priors
        gnss_priors = []
        anchor_utm = self._gnss_transform_data.get("anchor_utm", {})
        anchor_easting = anchor_utm.get("easting")
        anchor_northing = anchor_utm.get("northing")

        # Setup dummy orchestrator for visualizer
        anchor_data = self._gnss_transform_data.get("anchor", {})
        lat = anchor_data.get('latitude', None)
        lon = anchor_data.get('longitude', None)
        rot = self._gnss_transform_data.get('rotation_rad', 0.0)

        class DummyOrchestrator:
            def __init__(self, lat, lon, rot):
                self.anchor_latlon = (
                    lat, lon) if lat is not None and lon is not None else None
                self.init_rotation = rot
                self.anchor = (
                    anchor_easting, anchor_northing) if anchor_easting is not None and anchor_northing is not None else None
        self._orchestrator = DummyOrchestrator(lat, lon, rot)

        if config.gnss.enabled and anchor_easting is not None and anchor_northing is not None:
            self.get_logger().info("Loading GNSS data from Bag...")
            gnss_source = build_gnss_source(config, bag_path)
            gnss_source.start()

            for node in old_nodes:
                gnss = gnss_source.get_gnss_at(node.timestamp)
                if gnss is None:
                    continue
                sigma_xy = sigma_from_covariance_or_status(gnss, config)
                if sigma_xy <= 0.0 or sigma_xy > config.gnss.validation.max_sigma_m:
                    continue

                gx = gnss.x - anchor_easting
                gy = gnss.y - anchor_northing

                pos_var = sigma_xy * sigma_xy
                info_2x2 = np.diag([1.0 / pos_var, 1.0 / pos_var])
                gnss_priors.append(GnssPrior(
                    node_index=node.index, x=gx, y=gy, information=info_2x2
                ))
            gnss_source.stop()
            self.get_logger().info(
                f"Built {len(gnss_priors)} GNSS Prior constraints.")

        # 4. Edges Re-matching
        new_edges = []
        seq_edges_list = []
        loop_edges_list = []
        seq_edges_data = pose_graph_data.get("sequential_edges", [])
        loop_edges_data = pose_graph_data.get("loop_edges", [])

        enable_re_scan_matching = self.get_parameter(
            'enable_re_scan_matching').value
        self.get_logger().info(
            f"enable_re_scan_matching: {enable_re_scan_matching}")

        if not enable_re_scan_matching:
            self.get_logger().info(
                "Skipping re-scan matching. Using existing edges from pose_graph.json...")
            for e_data in seq_edges_data:
                edge = PoseEdge(
                    from_index=e_data["from"], to_index=e_data["to"],
                    dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"],
                    information=np.array(e_data["information"]).reshape(3, 3)
                )
                seq_edges_list.append(edge)
                new_edges.append(edge)
            for e_data in loop_edges_data:
                edge = PoseEdge(
                    from_index=e_data["from"], to_index=e_data["to"],
                    dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"],
                    information=np.array(e_data["information"]).reshape(3, 3)
                )
                loop_edges_list.append(edge)
                new_edges.append(edge)
        else:
            self.get_logger().info("Re-running Scan Matching for edges...")
            matcher = _build_matcher(config)
            loop_matcher = _build_loop_matcher(config)

            # Sequential Edges
            self.get_logger().info(
                f"Re-matching {len(seq_edges_data)} sequential edges...")
            shift_m = self.get_parameter('multi_guess_shift_m').value
            shift_rad = math.radians(
                self.get_parameter('multi_guess_shift_deg').value)

            for e_data in seq_edges_data:
                f_idx = e_data["from"]
                t_idx = e_data["to"]
                node_f, node_t = old_nodes[f_idx], old_nodes[t_idx]

                if f_idx not in node_scans or t_idx not in node_scans:
                    edge = PoseEdge(from_index=f_idx, to_index=t_idx, dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"], information=np.array(
                        e_data["information"]).reshape(3, 3))
                    seq_edges_list.append(edge)
                    new_edges.append(edge)
                    continue

                dx_local = e_data["dx"]
                dy_local = e_data["dy"]
                dyaw_local = e_data["dyaw"]
                initial_guess = OdomData(
                    timestamp=node_t.timestamp, x=dx_local, y=dy_local, yaw=dyaw_local)

                if config.scan_matching.reference == 'scan_to_local_map':
                    src_pts = build_submap_points(
                        f_idx, old_nodes, node_scans, config.scan_matching.local_map.radius)
                    if src_pts is None:
                        src_pts = scan_to_points(node_scans[f_idx])
                else:
                    src_pts = scan_to_points(node_scans[f_idx])

                gnss_guess = None
                if f_idx < len(old_nodes) and t_idx < len(old_nodes):
                    nf = old_nodes[f_idx]
                    nt = old_nodes[t_idx]
                    dx_w = nt.x - nf.x
                    dy_w = nt.y - nf.y
                    dx_local, dy_local = world_delta_to_local(
                        dx_w, dy_w, nf.yaw)
                    gnss_guess = OdomData(
                        timestamp=node_t.timestamp,
                        x=dx_local,
                        y=dy_local,
                        yaw=angle_diff(nt.yaw, nf.yaw),
                    )

                matcher.set_target_cloud(src_pts)
                result = self._match_with_multi_guess(
                    matcher=matcher,
                    src_pts=src_pts,
                    dst_scan=node_scans[t_idx],
                    base_guess=initial_guess,
                    gnss_guess=gnss_guess,
                    shift_m=shift_m,
                    shift_rad=shift_rad,
                )

                if result is not None:
                    edge = PoseEdge(from_index=f_idx, to_index=t_idx, dx=result.dx,
                                    dy=result.dy, dyaw=result.dyaw, information=result.information)
                else:
                    self.get_logger().warning(
                        f"Sequential edge matching failed {f_idx} -> {t_idx} (all guesses failed)")
                    edge = PoseEdge(from_index=f_idx, to_index=t_idx, dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"], information=np.array(
                        e_data["information"]).reshape(3, 3))
                seq_edges_list.append(edge)
                new_edges.append(edge)

            # Loop Edges
            self.get_logger().info(
                f"Re-matching {len(loop_edges_data)} loop edges...")
            for e_data in loop_edges_data:
                f_idx = e_data["from"]
                t_idx = e_data["to"]
                node_f, node_t = old_nodes[f_idx], old_nodes[t_idx]

                if f_idx not in node_scans or t_idx not in node_scans:
                    edge = PoseEdge(from_index=f_idx, to_index=t_idx, dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"], information=np.array(
                        e_data["information"]).reshape(3, 3))
                    loop_edges_list.append(edge)
                    new_edges.append(edge)
                    continue

                dx_local = e_data["dx"]
                dy_local = e_data["dy"]
                dyaw_local = e_data["dyaw"]
                initial_guess = OdomData(
                    timestamp=node_t.timestamp, x=dx_local, y=dy_local, yaw=dyaw_local)

                submap_radius = config.loop_closure.submap_radius
                if submap_radius > 0:
                    src_pts = build_submap_points(
                        f_idx, old_nodes, node_scans, submap_radius)
                else:
                    src_pts = scan_to_points(node_scans[f_idx])

                if src_pts is None:
                    edge = PoseEdge(from_index=f_idx, to_index=t_idx, dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"], information=np.array(
                        e_data["information"]).reshape(3, 3))
                    loop_edges_list.append(edge)
                    new_edges.append(edge)
                    continue

                loop_matcher.set_target_cloud(src_pts)
                result = loop_matcher.match(
                    dst=node_scans[t_idx], initial_guess=initial_guess)

                score_ok = True
                if config.loop_closure.max_score > 0.0 and result.score > config.loop_closure.max_score:
                    score_ok = False
                    self.get_logger().warning(
                        f"Loop edge {f_idx} -> {t_idx} rejected: score {result.score:.3f}")

                if result.converged and score_ok:
                    edge = PoseEdge(from_index=f_idx, to_index=t_idx, dx=result.dx,
                                    dy=result.dy, dyaw=result.dyaw, information=result.information)
                else:
                    self.get_logger().warning(
                        f"Loop edge matching failed {f_idx} -> {t_idx}")
                    edge = PoseEdge(from_index=f_idx, to_index=t_idx, dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"], information=np.array(
                        e_data["information"]).reshape(3, 3))
                loop_edges_list.append(edge)
                new_edges.append(edge)

        # 5. GTSAM Batch Optimization (Pass 1)
        self.get_logger().info("Executing GTSAM Batch optimization (pass 1)...")
        optimizer = GTSAMOptimizer()
        optimized_nodes_pass1 = optimizer.optimize(
            old_nodes, new_edges, gnss_priors)

        # 6. 新規ループエッジ探索
        enable_new_loop_search = self.get_parameter(
            'enable_new_loop_search').value
        if enable_new_loop_search and enable_re_scan_matching:
            self.get_logger().info("Searching for new loop edges (offline full search)...")
            loop_matcher = _build_loop_matcher(config)
            additional_loop_edges = self._search_new_loop_edges(
                optimized_nodes=optimized_nodes_pass1,
                node_scans=node_scans,
                existing_loop_edges=loop_edges_list,
                loop_matcher=loop_matcher,
                config=config,
            )
            self.get_logger().info(
                f"Found {len(additional_loop_edges)} new loop edges.")
            new_edges.extend(additional_loop_edges)
            loop_edges_list.extend(additional_loop_edges)

            # 7. GTSAM Batch Optimization (Pass 2)
            self.get_logger().info("Executing GTSAM Batch optimization (pass 2)...")
            self._optimized_nodes = optimizer.optimize(
                old_nodes, new_edges, gnss_priors)
        else:
            self._optimized_nodes = optimized_nodes_pass1

        self._new_edges = new_edges

        # 6. Render map
        self.get_logger().info("Re-rendering OccupancyGrid map...")
        self._renderer.rerender_all(self._optimized_nodes)

        if config.trajectory_noise_filter.enabled:
            noise_filter = TrajectoryNoiseFilter(
                config.trajectory_noise_filter)
            noise_filter.apply(self._renderer, self._optimized_nodes)

        # Publish results
        self._visualizer.publish_map(self._renderer)
        self._visualizer.rebuild_path(self._optimized_nodes)

        self._pose_graph_view = ReadOnlyPoseGraph(
            self._optimized_nodes, seq_edges_list, loop_edges_list)
        self._visualizer.publish_pose_graph_markers(self._pose_graph_view)
        self._visualizer.publish_anchor(self._orchestrator)

        self.get_logger().info("Re-optimization complete. Waiting for save service call...")

    def _handle_save_slam_map(self, request, response):
        output_dir = self.get_parameter('save_dir').value
        if not output_dir:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = f"/root/ros2_data/slam_maps/{timestamp}_opt"

        self.get_logger().info(
            f"Saving optimized results to directory: {output_dir}")
        try:
            os.makedirs(output_dir, exist_ok=True)

            # Save pose graph
            SlamDataSaver.save_pose_graph(
                output_dir, self._optimized_nodes, self._new_edges, bag_path=self._bag_path)

            # Save GNSS transform
            if self._gnss_transform_data:
                lat = self._gnss_transform_data.get(
                    'anchor', {}).get('latitude', 0.0)
                lon = self._gnss_transform_data.get(
                    'anchor', {}).get('longitude', 0.0)
                easting = self._gnss_transform_data.get(
                    'anchor_utm', {}).get('easting', 0.0)
                northing = self._gnss_transform_data.get(
                    'anchor_utm', {}).get('northing', 0.0)
                zone = self._gnss_transform_data.get(
                    'anchor_utm', {}).get('zone', 54)
                hemisphere = self._gnss_transform_data.get(
                    'anchor_utm', {}).get('hemisphere', 'north')
                rotation_rad = self._gnss_transform_data.get(
                    'rotation_rad', 0.0)

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

            # NOTE: PGM 保存は system-manager が map_saver_cli を使って行うため、ここから PGM 保存機能を削除。

            response.success = True
            response.message = f"Optimized map saved to {output_dir}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save optimized map: {e}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    logging.basicConfig(level=logging.INFO,
                        format='%(name)s %(levelname)s: %(message)s')
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
