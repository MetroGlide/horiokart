import bisect
import math
import logging
import numpy as np
from typing import List, Dict, Optional, Callable, Any

from slam_gnss_2d.core.config import SlamConfig
from slam_gnss_2d.core.data_types import PoseNode, PoseEdge, GnssPrior, ScanData, OdomData, GnssData
from slam_gnss_2d.core.component_factory import build_gnss_source, _build_matcher, _build_loop_matcher
from slam_gnss_2d.optimizer.gtsam_optimizer import GTSAMOptimizer
from slam_gnss_2d.map_manager import OverwriteRenderer, CountingRenderer
from slam_gnss_2d.input.ros2.bag_reader import BagScanSource


def _scan_to_points(scan: ScanData) -> np.ndarray:
    """有効レンジのみを2D点群 (N, 2) に変換する。"""
    n = len(scan.ranges)
    angles = scan.angle_min + np.arange(n) * scan.angle_increment
    ranges = np.asarray(scan.ranges, dtype=np.float64)
    valid = (ranges >= scan.range_min) & (ranges <= scan.range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def build_submap_points(
    center_node_idx: int,
    nodes: List[PoseNode],
    node_scans: Dict[int, ScanData],
    radius: float
) -> Optional[np.ndarray]:
    """指定ノード周辺の点群を合成してサブマップを構築する"""
    center_node = nodes[center_node_idx]
    near_nodes = []
    for n in nodes:
        if n.index in node_scans:
            dist = math.hypot(n.x - center_node.x, n.y - center_node.y)
            if dist <= radius:
                near_nodes.append((n, node_scans[n.index]))
                
    if not near_nodes:
        return None
        
    world_pts_list = []
    for n, scan in near_nodes:
        local_pts = _scan_to_points(scan)
        c = math.cos(n.yaw)
        s = math.sin(n.yaw)
        wx = c * local_pts[:, 0] - s * local_pts[:, 1] + n.x
        wy = s * local_pts[:, 0] + c * local_pts[:, 1] + n.y
        world_pts_list.append(np.column_stack((wx, wy)))
        
    world_pts = np.concatenate(world_pts_list, axis=0)
    
    # center_node のボディフレームに逆投影
    c = math.cos(center_node.yaw)
    s = math.sin(center_node.yaw)
    R_inv = np.array([[c, s], [-s, c]])
    
    relative_pts = (R_inv @ (world_pts - np.array([center_node.x, center_node.y])).T).T
    return relative_pts


def find_nearest_scan(scans: List[ScanData], timestamp: float, max_diff: float = 0.1) -> Optional[ScanData]:
    """タイムスタンプが最も近いスキャンデータを返す"""
    if not scans:
        return None
    timestamps = [s.timestamp for s in scans]
    idx = bisect.bisect_left(timestamps, timestamp)
    if idx == 0:
        candidate = scans[0]
    elif idx >= len(scans):
        candidate = scans[-1]
    else:
        prev = scans[idx - 1]
        next_ = scans[idx]
        if abs(timestamp - prev.timestamp) <= abs(next_.timestamp - timestamp):
            candidate = prev
        else:
            candidate = next_
            
    if abs(candidate.timestamp - timestamp) <= max_diff:
        return candidate
    return None


def sigma_from_covariance_or_status(gnss: GnssData, config: SlamConfig) -> float:
    """GNSS データから位置の標準偏差を導出する"""
    cov_xx = float(gnss.covariance[0, 0]) if gnss.covariance is not None else 0.0
    if cov_xx > 0.0:
        return math.sqrt(cov_xx)
    
    status = gnss.fix_status
    if status >= 2:
        return config.gnss.sigma.fix_m
    if status >= 0:
        return config.gnss.sigma.float_m
    return -1.0



class PoseGraphReoptimizer:
    def __init__(self, config: SlamConfig, logger=None):
        self.config = config
        self.logger = logger or logging.getLogger("reoptimizer")
        
    def reoptimize(
        self,
        pose_graph_data: dict,
        gnss_transform_data: dict,
        bag_path: str,
        on_scans_extracted: Callable[[List[PoseNode]], None] = None,
        enable_re_scan_matching: bool = False
    ):
        """
        Runs the full reoptimization pipeline.
        Returns: optimized_nodes, new_edges, renderer
        """
        # 1. Extract scans
        self.logger.info("Extracting LiDAR scans from ROS Bag...")
        scan_source = BagScanSource(bag_path, self.config.topics.scan)
        scan_source.start()
        
        all_scans = []
        def scan_callback(scan_data: ScanData):
            all_scans.append(scan_data)
            
        scan_source.set_scan_callback(scan_callback)
        scan_count = 0
        while scan_source.step():
            scan_count += 1
            if scan_count % 1000 == 0:
                self.logger.info(f"Loaded {scan_count} scans...")
        scan_source.stop()
        self.logger.info(f"Loaded total {len(all_scans)} scans from Bag.")

        # 2. Reconstruct old nodes
        self.logger.info("Matching nodes to nearest LiDAR scans...")
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
                self.logger.warning(f"No matching scan found for node {node_idx}")
                
        if on_scans_extracted:
            on_scans_extracted(old_nodes)

        # 3. GNSS Priors
        gnss_priors = []
        anchor_utm = gnss_transform_data.get("anchor_utm", {})
        anchor_easting = anchor_utm.get("easting")
        anchor_northing = anchor_utm.get("northing")

        if self.config.gnss.enabled and anchor_easting is not None and anchor_northing is not None:
            self.logger.info("Loading GNSS data from Bag...")
            gnss_source = build_gnss_source(self.config, bag_path)
            gnss_source.start()

            for node in old_nodes:
                gnss = gnss_source.get_gnss_at(node.timestamp)
                if gnss is None: continue
                sigma_xy = sigma_from_covariance_or_status(gnss, self.config)
                if sigma_xy <= 0.0 or sigma_xy > self.config.gnss.validation.max_sigma_m:
                    continue
                    
                gx = gnss.x - anchor_easting
                gy = gnss.y - anchor_northing

                pos_var = sigma_xy * sigma_xy
                info_2x2 = np.diag([1.0 / pos_var, 1.0 / pos_var])
                gnss_priors.append(GnssPrior(
                    node_index=node.index, x=gx, y=gy, information=info_2x2
                ))
            gnss_source.stop()
            self.logger.info(f"Built {len(gnss_priors)} GNSS Prior constraints.")

        # 4. Edges Re-matching
        new_edges = []
        seq_edges_data = pose_graph_data.get("sequential_edges", [])
        loop_edges_data = pose_graph_data.get("loop_edges", [])

        if not enable_re_scan_matching:
            self.logger.info("Skipping re-scan matching. Using existing edges from pose_graph.json...")
            for e_data in seq_edges_data + loop_edges_data:
                new_edges.append(PoseEdge(
                    from_index=e_data["from"], to_index=e_data["to"],
                    dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"],
                    information=np.array(e_data["information"]).reshape(3, 3)
                ))
        else:
            self.logger.info("Re-running Scan Matching for edges...")
            matcher = _build_matcher(self.config)
            loop_matcher = _build_loop_matcher(self.config)

            # Sequential Edges
            self.logger.info(f"Re-matching {len(seq_edges_data)} sequential edges...")
            for e_data in seq_edges_data:
                f_idx = e_data["from"]
                t_idx = e_data["to"]
                node_f, node_t = old_nodes[f_idx], old_nodes[t_idx]

                if f_idx not in node_scans or t_idx not in node_scans:
                    new_edges.append(PoseEdge(
                        from_index=f_idx, to_index=t_idx,
                        dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"],
                        information=np.array(e_data["information"]).reshape(3, 3)
                    ))
                    continue

                dx_local = e_data["dx"]
                dy_local = e_data["dy"]
                dyaw_local = e_data["dyaw"]
                initial_guess = OdomData(timestamp=node_t.timestamp, x=dx_local, y=dy_local, yaw=dyaw_local)

                src_pts = _scan_to_points(node_scans[f_idx])
                matcher.set_target_cloud(src_pts)
                result = matcher.match(dst=node_scans[t_idx], initial_guess=initial_guess)

                if result.converged:
                    new_edges.append(PoseEdge(from_index=f_idx, to_index=t_idx, dx=result.dx, dy=result.dy, dyaw=result.dyaw, information=result.information))
                else:
                    self.logger.warning(f"Sequential edge matching failed {f_idx} -> {t_idx}")
                    new_edges.append(PoseEdge(from_index=f_idx, to_index=t_idx, dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"], information=np.array(e_data["information"]).reshape(3, 3)))

            # Loop Edges
            self.logger.info(f"Re-matching {len(loop_edges_data)} loop edges...")
            for e_data in loop_edges_data:
                f_idx = e_data["from"]
                t_idx = e_data["to"]
                node_f, node_t = old_nodes[f_idx], old_nodes[t_idx]

                if f_idx not in node_scans or t_idx not in node_scans:
                    new_edges.append(PoseEdge(from_index=f_idx, to_index=t_idx, dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"], information=np.array(e_data["information"]).reshape(3, 3)))
                    continue

                dx_local = e_data["dx"]
                dy_local = e_data["dy"]
                dyaw_local = e_data["dyaw"]
                initial_guess = OdomData(timestamp=node_t.timestamp, x=dx_local, y=dy_local, yaw=dyaw_local)

                submap_radius = self.config.loop_closure.submap_radius
                if submap_radius > 0:
                    src_pts = build_submap_points(f_idx, old_nodes, node_scans, submap_radius)
                else:
                    src_pts = _scan_to_points(node_scans[f_idx])

                if src_pts is None:
                    new_edges.append(PoseEdge(from_index=f_idx, to_index=t_idx, dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"], information=np.array(e_data["information"]).reshape(3, 3)))
                    continue

                loop_matcher.set_target_cloud(src_pts)
                result = loop_matcher.match(dst=node_scans[t_idx], initial_guess=initial_guess)

                score_ok = True
                if self.config.loop_closure.max_score > 0.0 and result.score > self.config.loop_closure.max_score:
                    score_ok = False
                    self.logger.warning(f"Loop edge {f_idx} -> {t_idx} rejected: score {result.score:.3f}")

                if result.converged and score_ok:
                    new_edges.append(PoseEdge(from_index=f_idx, to_index=t_idx, dx=result.dx, dy=result.dy, dyaw=result.dyaw, information=result.information))
                else:
                    self.logger.warning(f"Loop edge matching failed {f_idx} -> {t_idx}")
                    new_edges.append(PoseEdge(from_index=f_idx, to_index=t_idx, dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"], information=np.array(e_data["information"]).reshape(3, 3)))

        # 5. GTSAM Batch Optimization
        self.logger.info("Executing GTSAM Batch optimization (Levenberg-Marquardt)...")
        optimizer = GTSAMOptimizer()
        optimized_nodes = optimizer.optimize(old_nodes, new_edges, gnss_priors)

        # 6. Render map
        self.logger.info("Re-rendering OccupancyGrid map...")
        if self.config.map.renderer == 'counting':
            renderer = CountingRenderer(
                resolution=self.config.map.resolution,
                expansion_margin=self.config.map.expansion_margin,
                hit_threshold=self.config.map.hit_threshold,
            )
        else:
            renderer = OverwriteRenderer(
                resolution=self.config.map.resolution,
                expansion_margin=self.config.map.expansion_margin,
            )
        renderer.rerender_all(optimized_nodes)

        return optimized_nodes, new_edges, renderer

