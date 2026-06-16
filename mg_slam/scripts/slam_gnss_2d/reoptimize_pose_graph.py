#!/usr/bin/env python3
import os
import sys
import argparse
import json
import math
import logging
import bisect
from typing import List, Dict, Optional, Tuple

import numpy as np
import yaml
import cv2

# scripts/slam_gnss_2d 階層を python パスに通す
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from slam_gnss_2d.config import (
    SlamConfig, TopicsConfig, MapConfig, KeyframeConfig,
    IcpConfig, NdtConfig, LocalMapConfig, CsmConfig,
    ScanMatchingConfig, LoopClosureConfig, GnssTopicsConfig,
    GnssValidationConfig, GnssAnchorConfig, GnssSigmaConfig,
    GnssConfig, OptimizationConfig
)
from slam_gnss_2d.data_types import PoseNode, PoseEdge, GnssPrior, ScanData, OdomData, GnssData
from slam_gnss_2d.component_factory import build_gnss_source, _build_matcher, _build_loop_matcher
from slam_gnss_2d.optimizer.gtsam_optimizer import GTSAMOptimizer
from slam_gnss_2d.map_manager import OverwriteRenderer, CountingRenderer
from slam_gnss_2d.slam_data_saver import SlamDataSaver
from slam_gnss_2d.input.ros2.bag_reader import BagScanSource

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(name)s: %(message)s')
logger = logging.getLogger("reoptimize")

def load_config_from_yaml(yaml_path: str) -> SlamConfig:
    """YAML ファイルからパラメータをロードして SlamConfig に反映する"""
    if not os.path.exists(yaml_path):
        logger.warning(f"Config file {yaml_path} not found. Using default parameters.")
        return SlamConfig()
        
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
        
    if not isinstance(data, dict):
        return SlamConfig()
        
    # ROS2 の yaml 構造に対応
    params = {}
    for node_key in data.keys():
        if isinstance(data[node_key], dict) and 'ros__parameters' in data[node_key]:
            params = data[node_key]['ros__parameters']
            break
    else:
        params = data
        
    # Topics
    topics = TopicsConfig(
        scan=params.get('topics.scan', '/scan_top_lidar'),
        odom=params.get('topics.odom', '/odom')
    )
    # Map
    map_cfg = MapConfig(
        resolution=float(params.get('map.resolution', 0.05)),
        expansion_margin=float(params.get('map.expansion_margin', 100.0)),
        publish_hz=float(params.get('map.publish_hz', 1.0)),
        renderer=params.get('map.renderer', 'overwrite'),
        hit_threshold=float(params.get('map.hit_threshold', 0.3))
    )
    # Keyframe
    keyframe = KeyframeConfig(
        min_translation=float(params.get('keyframe.min_translation', 1.0)),
        min_rotation=float(params.get('keyframe.min_rotation', 0.1))
    )
    # Icp
    icp = IcpConfig(
        max_iterations=int(params.get('scan_matching.icp.max_iterations', 100)),
        tolerance=float(params.get('scan_matching.icp.tolerance', 1e-5)),
        max_correspondence_dist=float(params.get('scan_matching.icp.max_correspondence_dist', 1.0)),
        robust_kernel=params.get('scan_matching.icp.robust_kernel', 'huber'),
        robust_kernel_scale=float(params.get('scan_matching.icp.robust_kernel_scale', 0.1))
    )
    # Ndt
    ndt = NdtConfig(
        cell_size=float(params.get('scan_matching.ndt.cell_size', 1.0)),
        cell_sizes=tuple(float(x) for x in params.get('scan_matching.ndt.cell_sizes', [1.0])),
        use_bilinear=bool(params.get('scan_matching.ndt.use_bilinear', False))
    )
    # LocalMap
    local_map = LocalMapConfig(
        window=int(params.get('scan_matching.local_map.window', 30)),
        radius=float(params.get('scan_matching.local_map.radius', 30.0))
    )
    # Csm
    csm = CsmConfig(
        linear_search_window=float(params.get('scan_matching.csm.linear_search_window', 1.0)),
        angular_search_window=float(params.get('scan_matching.csm.angular_search_window', 0.5)),
        linear_step=float(params.get('scan_matching.csm.linear_step', 0.05)),
        angular_step=float(params.get('scan_matching.csm.angular_step', 0.02))
    )
    # ScanMatching
    scan_matching = ScanMatchingConfig(
        enabled=bool(params.get('scan_matching.enabled', True)),
        type=params.get('scan_matching.type', 'ndt'),
        reference=params.get('scan_matching.reference', 'scan_to_local_map'),
        max_failure_streak=int(params.get('scan_matching.max_failure_streak', 5)),
        icp=icp,
        ndt=ndt,
        csm=csm,
        local_map=local_map
    )
    
    # LoopClosure
    loop_icp = IcpConfig(
        max_iterations=int(params.get('loop_closure.icp.max_iterations', 100)),
        tolerance=float(params.get('loop_closure.icp.tolerance', 1e-5)),
        max_correspondence_dist=float(params.get('loop_closure.icp.max_correspondence_dist', 1.0)),
        robust_kernel=params.get('loop_closure.icp.robust_kernel', 'huber'),
        robust_kernel_scale=float(params.get('loop_closure.icp.robust_kernel_scale', 0.1))
    )
    loop_ndt = NdtConfig(
        cell_size=float(params.get('loop_closure.ndt.cell_size', 1.0)),
        cell_sizes=tuple(float(x) for x in params.get('loop_closure.ndt.cell_sizes', [1.0])),
        use_bilinear=bool(params.get('loop_closure.ndt.use_bilinear', False))
    )
    loop_csm = CsmConfig(
        linear_search_window=float(params.get('loop_closure.csm.linear_search_window', 1.0)),
        angular_search_window=float(params.get('loop_closure.csm.angular_search_window', 0.5)),
        linear_step=float(params.get('loop_closure.csm.linear_step', 0.05)),
        angular_step=float(params.get('loop_closure.csm.angular_step', 0.02))
    )
    loop_closure = LoopClosureConfig(
        enabled=bool(params.get('loop_closure.enabled', True)),
        search_radius=float(params.get('loop_closure.search_radius', 2.0)),
        min_node_gap=int(params.get('loop_closure.min_node_gap', 50)),
        max_failure_streak=int(params.get('loop_closure.max_failure_streak', 3)),
        matcher_type=params.get('loop_closure.matcher_type', 'icp'),
        icp=loop_icp,
        ndt=loop_ndt,
        csm=loop_csm,
        max_dyaw_deg=float(params.get('loop_closure.max_dyaw_deg', 145.0)),
        crossing_reject_deg=float(params.get('loop_closure.crossing_reject_deg', 45.0)),
        submap_radius=float(params.get('loop_closure.submap_radius', 5.0)),
        max_score=float(params.get('loop_closure.max_score', 0.0))
    )
    
    # GNSS
    gnss_topics = GnssTopicsConfig(
        fix=params.get('gnss.topics.fix', '/gps/fix'),
        navpvt=params.get('gnss.topics.navpvt', '/navpvt')
    )
    gnss_val = GnssValidationConfig(
        max_sigma_m=float(params.get('gnss.validation.max_sigma_m', 5.0)),
        missing_grace_frames=int(params.get('gnss.validation.missing_grace_frames', 30))
    )
    gnss_anchor = GnssAnchorConfig(
        min_fix_status=int(params.get('gnss.anchor.min_fix_status', 0)),
        sigma_m=float(params.get('gnss.anchor.sigma_m', 0.05)),
        init_yaw_sigma_rad=float(params.get('gnss.anchor.init_yaw_sigma_rad', 10.0)),
        init_distance_m=float(params.get('gnss.anchor.init_distance_m', 2.0))
    )
    gnss_sigma = GnssSigmaConfig(
        fix_m=float(params.get('gnss.sigma.fix_m', 0.02)),
        float_m=float(params.get('gnss.sigma.float_m', 0.5)),
        factor_yaw_variance=float(params.get('gnss.sigma.factor_yaw_variance', 1e8))
    )
    gnss = GnssConfig(
        enabled=bool(params.get('gnss.enabled', True)),
        source=params.get('gnss.source', 'navpvt'),
        topics=gnss_topics,
        navpvt_hacc_scale=float(params.get('gnss.navpvt_hacc_scale', 1.0)),
        validation=gnss_val,
        anchor=gnss_anchor,
        sigma=gnss_sigma
    )
    
    # Optimization
    opt = OptimizationConfig(
        backend=params.get('optimization.backend', 'gtsam'),
        incremental=bool(params.get('optimization.incremental', True)),
        optimize_every_n_loops=int(params.get('optimization.optimize_every_n_loops', 3)),
        rerender_threshold_m=float(params.get('optimization.rerender_threshold_m', 0.1))
    )
    
    return SlamConfig(
        topics=topics,
        map=map_cfg,
        keyframe=keyframe,
        scan_matching=scan_matching,
        loop_closure=loop_closure,
        gnss=gnss,
        optimization=opt
    )


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


def save_map_pgm_and_yaml(output_dir: str, renderer):
    """OccupancyGrid マップを PGM/YAML として保存する"""
    data, origin_x, origin_y, resolution = renderer.to_occupancy_array()
    
    # trinary 画像への変換
    # occupied (100) -> 0
    # free (0) -> 255
    # unknown (-1) -> 205
    img = np.full(data.shape, 205, dtype=np.uint8)
    img[data == 100] = 0
    img[data == 0] = 255
    
    # ROS 2 座標系 (Y-up) から画像座標系 (Y-down) への垂直フリップ
    img_flipped = np.flipud(img)
    
    os.makedirs(output_dir, exist_ok=True)
    pgm_path = os.path.join(output_dir, "map_optimized.pgm")
    cv2.imwrite(pgm_path, img_flipped)
    
    # YAML の保存
    yaml_path = os.path.join(output_dir, "map_optimized.yaml")
    yaml_data = {
        "image": "map_optimized.pgm",
        "resolution": float(resolution),
        "origin": [float(origin_x), float(origin_y), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25
    }
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
        
    logger.info(f"Saved optimized map to: {pgm_path} and {yaml_path}")


def main():
    parser = argparse.ArgumentParser(description="GTSAMを用いた slam_gnss_2d ポーズグラフ再最適化ツール")
    parser.add_argument("--input_dir", required=True, help="pose_graph.json や gnss_transform.yaml が含まれるディレクトリ")
    parser.add_argument("--output_dir", help="最適化結果を保存するディレクトリ (デフォルトは input_dir/reoptimized)")
    parser.add_argument("--config_file", help="パラメータが記載された yaml ファイル (指定がなければデフォルト値を使用)")
    parser.add_argument("--bag_path", help="元の ROS Bag パス (指定があれば json 内のパスを上書き)")
    args = parser.parse_args()

    # パス解決
    input_dir = os.path.abspath(args.input_dir)
    output_dir = os.path.abspath(args.output_dir) if args.output_dir else os.path.join(input_dir, "reoptimized")
    
    pose_graph_path = os.path.join(input_dir, "pose_graph.json")
    gnss_transform_path = os.path.join(input_dir, "gnss_transform.yaml")
    
    if not os.path.exists(pose_graph_path):
        logger.error(f"pose_graph.json が見つかりません: {pose_graph_path}")
        sys.exit(1)
        
    if not os.path.exists(gnss_transform_path):
        logger.error(f"gnss_transform.yaml が見つかりません: {gnss_transform_path}")
        sys.exit(1)

    # 1. データのロード
    logger.info("Loading existing pose graph and transform parameters...")
    with open(pose_graph_path, 'r') as f:
        pose_graph_data = json.load(f)
        
    with open(gnss_transform_path, 'r') as f:
        gnss_transform_data = yaml.safe_load(f)

    # ROS Bag パス決定
    bag_path = args.bag_path
    if not bag_path:
        bag_path = pose_graph_data.get("metadata", {}).get("bag_path")
        
    if not bag_path:
        logger.error("ROS Bag パスが指定されていません。--bag_path オプションを使用するか、メタデータに bag_path を保存した pose_graph.json を使用してください。")
        sys.exit(1)
        
    logger.info(f"Using ROS Bag: {bag_path}")

    # Config ロード
    config_file = args.config_file
    if not config_file:
        # デフォルトの params/slam_gnss_2d.yaml を探す
        default_yaml = os.path.join(os.path.dirname(os.path.dirname(SCRIPT_DIR)), "params", "slam_gnss_2d.yaml")
        if os.path.exists(default_yaml):
            config_file = default_yaml
            logger.info(f"Using default config file: {config_file}")
            
    if config_file:
        config = load_config_from_yaml(config_file)
    else:
        config = SlamConfig()
        logger.info("Using built-in default config parameters.")

    # 2. ROS Bag からスキャンデータの抽出
    logger.info("Extracting LiDAR scans from ROS Bag...")
    scan_source = BagScanSource(bag_path, config.topics.scan)
    scan_source.start()
    
    all_scans: List[ScanData] = []
    def scan_callback(scan_data: ScanData):
        all_scans.append(scan_data)
        
    scan_source.set_scan_callback(scan_callback)
    
    scan_count = 0
    while scan_source.step():
        scan_count += 1
        if scan_count % 1000 == 0:
            logger.info(f"Loaded {scan_count} scans...")
    scan_source.stop()
    logger.info(f"Loaded total {len(all_scans)} scans from Bag.")

    # 3. 既存のノードにスキャンデータを紐付ける
    logger.info("Matching nodes to nearest LiDAR scans...")
    old_nodes: List[PoseNode] = []
    node_scans: Dict[int, ScanData] = {}
    
    for n_data in pose_graph_data["nodes"]:
        node_idx = n_data["index"]
        ts = n_data["timestamp"]
        
        scan = find_nearest_scan(all_scans, ts, max_diff=0.05)
        node = PoseNode(
            index=node_idx,
            timestamp=ts,
            x=n_data["x"],
            y=n_data["y"],
            yaw=n_data["yaw"],
            scan=scan
        )
        old_nodes.append(node)
        if scan is not None:
            node_scans[node_idx] = scan
        else:
            logger.warning(f"No matching scan found for node {node_idx} (ts={ts:.3f})")

    # 4. GNSS データのロードと Prior 拘束構築
    gnss_priors: List[GnssPrior] = []
    anchor_utm = gnss_transform_data.get("anchor_utm", {})
    anchor_easting = anchor_utm.get("easting")
    anchor_northing = anchor_utm.get("northing")
    
    if config.gnss.enabled and anchor_easting is not None and anchor_northing is not None:
        logger.info("Loading GNSS data from Bag...")
        gnss_source = build_gnss_source(config, bag_path)
        gnss_source.start()
        
        for node in old_nodes:
            gnss = gnss_source.get_gnss_at(node.timestamp)
            if gnss is None:
                continue
                
            sigma_xy = sigma_from_covariance_or_status(gnss, config)
            if sigma_xy <= 0.0 or sigma_xy > config.gnss.validation.max_sigma_m:
                continue
                
            # アンカー基準のローカル平面座標に変換
            gx = gnss.x - anchor_easting
            gy = gnss.y - anchor_northing
            
            # 情報行列の構築 (位置のみ拘束、yawは極めて弱い情報にする)
            pos_var = sigma_xy * sigma_xy
            info_2x2 = np.diag([1.0 / pos_var, 1.0 / pos_var])
            
            gnss_priors.append(GnssPrior(
                node_index=node.index,
                x=gx,
                y=gy,
                information=info_2x2
            ))
        gnss_source.stop()
        logger.info(f"Built {len(gnss_priors)} GNSS Prior constraints.")

    # 5. エッジ (相対拘束) の再スキャンマッチング
    logger.info("Re-running Scan Matching for edges...")
    matcher = _build_matcher(config)
    loop_matcher = _build_loop_matcher(config)
    
    new_edges: List[PoseEdge] = []
    
    # (A) 順次エッジ (Sequential Edges)
    seq_edges_data = pose_graph_data.get("sequential_edges", [])
    logger.info(f"Re-matching {len(seq_edges_data)} sequential edges...")
    for idx, e_data in enumerate(seq_edges_data):
        f_idx = e_data["from"]
        t_idx = e_data["to"]
        
        node_f = old_nodes[f_idx]
        node_t = old_nodes[t_idx]
        
        if f_idx not in node_scans or t_idx not in node_scans:
            # スキャンがない場合は既存のエッジ情報をそのまま転写
            new_edges.append(PoseEdge(
                from_index=f_idx, to_index=t_idx,
                dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"],
                information=np.array(e_data["information"]).reshape(3, 3)
            ))
            continue
            
        # 大域的な姿勢からより正確な相対姿勢の初期推定値を計算
        # f_idx からの局所座標に t_idx のポーズを投影
        c = math.cos(node_f.yaw)
        s = math.sin(node_f.yaw)
        dx_w = node_t.x - node_f.x
        dy_w = node_t.y - node_f.y
        dx_local = c * dx_w + s * dy_w
        dy_local = -s * dx_w + c * dy_w
        dyaw_local = math.atan2(math.sin(node_t.yaw - node_f.yaw), math.cos(node_t.yaw - node_f.yaw))
        
        initial_guess = OdomData(
            timestamp=node_t.timestamp,
            x=dx_local, y=dy_local, yaw=dyaw_local
        )
        
        # ターゲット点群（前フレーム）の設定
        src_pts = _scan_to_points(node_scans[f_idx])
        matcher.set_target_cloud(src_pts)
        result = matcher.match(
            dst=node_scans[t_idx],
            initial_guess=initial_guess
        )
        
        if result.converged:
            new_edges.append(PoseEdge(
                from_index=f_idx, to_index=t_idx,
                dx=result.dx, dy=result.dy, dyaw=result.dyaw,
                information=result.information
            ))
        else:
            logger.warning(f"Sequential edge matching failed {f_idx} -> {t_idx}. Falling back to old edge.")
            new_edges.append(PoseEdge(
                from_index=f_idx, to_index=t_idx,
                dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"],
                information=np.array(e_data["information"]).reshape(3, 3)
            ))

    # (B) ループエッジ (Loop Edges)
    loop_edges_data = pose_graph_data.get("loop_edges", [])
    logger.info(f"Re-matching {len(loop_edges_data)} loop edges...")
    for idx, e_data in enumerate(loop_edges_data):
        f_idx = e_data["from"]
        t_idx = e_data["to"]
        
        node_f = old_nodes[f_idx]
        node_t = old_nodes[t_idx]
        
        if f_idx not in node_scans or t_idx not in node_scans:
            new_edges.append(PoseEdge(
                from_index=f_idx, to_index=t_idx,
                dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"],
                information=np.array(e_data["information"]).reshape(3, 3)
            ))
            continue
            
        c = math.cos(node_f.yaw)
        s = math.sin(node_f.yaw)
        dx_w = node_t.x - node_f.x
        dy_w = node_t.y - node_f.y
        dx_local = c * dx_w + s * dy_w
        dy_local = -s * dx_w + c * dy_w
        dyaw_local = math.atan2(math.sin(node_t.yaw - node_f.yaw), math.cos(node_t.yaw - node_f.yaw))
        
        initial_guess = OdomData(
            timestamp=node_t.timestamp,
            x=dx_local, y=dy_local, yaw=dyaw_local
        )
        
        # サブマップ合成
        submap_radius = config.loop_closure.submap_radius
        if submap_radius > 0:
            src_pts = build_submap_points(f_idx, old_nodes, node_scans, submap_radius)
        else:
            src_pts = _scan_to_points(node_scans[f_idx])
            
        if src_pts is None:
            new_edges.append(PoseEdge(
                from_index=f_idx, to_index=t_idx,
                dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"],
                information=np.array(e_data["information"]).reshape(3, 3)
            ))
            continue
            
        loop_matcher.set_target_cloud(src_pts)
        result = loop_matcher.match(
            dst=node_scans[t_idx],
            initial_guess=initial_guess
        )
        
        # ループクロージャ特有のスコア制限判定
        score_ok = True
        if config.loop_closure.max_score > 0.0 and result.score > config.loop_closure.max_score:
            score_ok = False
            logger.warning(f"Loop edge {f_idx} -> {t_idx} rejected: score {result.score:.3f} > max {config.loop_closure.max_score:.3f}")
            
        if result.converged and score_ok:
            new_edges.append(PoseEdge(
                from_index=f_idx, to_index=t_idx,
                dx=result.dx, dy=result.dy, dyaw=result.dyaw,
                information=result.information
            ))
        else:
            logger.warning(f"Loop edge matching failed {f_idx} -> {t_idx}. Falling back to old edge.")
            new_edges.append(PoseEdge(
                from_index=f_idx, to_index=t_idx,
                dx=e_data["dx"], dy=e_data["dy"], dyaw=e_data["dyaw"],
                information=np.array(e_data["information"]).reshape(3, 3)
            ))

    # 6. GTSAM フルバッチ一括最適化
    logger.info("Executing GTSAM Batch optimization (Levenberg-Marquardt)...")
    optimizer = GTSAMOptimizer()
    
    # 最初のノード姿勢をアライメントの基準（アンカー）として強く固定するために渡す
    optimized_nodes = optimizer.optimize(old_nodes, new_edges, gnss_priors)

    # 7. 最適化マップの再描画
    logger.info("Re-rendering OccupancyGrid map...")
    if config.map.renderer == 'counting':
        renderer = CountingRenderer(
            resolution=config.map.resolution,
            expansion_margin=config.map.expansion_margin,
            hit_threshold=config.map.hit_threshold,
        )
    else:
        renderer = OverwriteRenderer(
            resolution=config.map.resolution,
            expansion_margin=config.map.expansion_margin,
        )
        
    renderer.rerender_all(optimized_nodes)

    # 8. 保存処理
    logger.info(f"Saving optimized results to directory: {output_dir}")
    os.makedirs(output_dir, exist_ok=True)
    
    # ポーズグラフの保存
    SlamDataSaver.save_pose_graph(output_dir, optimized_nodes, new_edges, bag_path=bag_path)
    
    # GNSS 変換情報の保存
    # 最適化によって rotation_rad が微調整されている可能性を反映する
    # 通常 rotation_rad は init_rotation だが、一括最適化によって node[0] の yaw が theta0 から
    # わずかに変動する可能性があるため、それを反映して rotation_rad を算出する
    node0 = optimized_nodes[0]
    # gnss_transform 内の rotation_rad は SLAM系座標から絶対方位（UTM系）への回転角
    # anchor_latlon, anchor_utm はそのまま引き継ぐ
    lat = gnss_transform_data['anchor']['latitude']
    lon = gnss_transform_data['anchor']['longitude']
    easting = gnss_transform_data['anchor_utm']['easting']
    northing = gnss_transform_data['anchor_utm']['northing']
    zone = gnss_transform_data['anchor_utm']['zone']
    hemisphere = gnss_transform_data['anchor_utm']['hemisphere']
    
    # 元のオンライン Heading aligner の仕組みに合わせる
    # _initialize_graph 内で `node0.yaw + rot = theta0`
    # よって rot = theta0 - node0.yaw. 最適化後の node0.yaw から逆算
    # ここでは元の transform の rotation_rad をそのままコピーするか、
    # 最初のノードの yaw がほぼ変化しない（アンカーによって強く固定されている）ため、そのまま引き継いでもほぼ同等。
    rotation_rad = gnss_transform_data.get('rotation_rad', 0.0)
    
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
    
    # マップ画像保存 (map_optimized.pgm, map_optimized.yaml)
    save_map_pgm_and_yaml(output_dir, renderer)
    
    logger.info("Re-optimization complete successfully.")


if __name__ == "__main__":
    main()
