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

from slam_gnss_2d.core.config import (
    SlamConfig, TopicsConfig, MapConfig, KeyframeConfig,
    IcpConfig, NdtConfig, LocalMapConfig, CsmConfig,
    ScanMatchingConfig, LoopClosureConfig, GnssTopicsConfig,
    GnssValidationConfig, GnssAnchorConfig, GnssSigmaConfig,
    GnssValidationConfig, GnssAnchorConfig, GnssSigmaConfig,
    GnssConfig, OptimizationConfig, Isam2Config
)
from slam_gnss_2d.core.data_types import PoseNode, PoseEdge, GnssPrior, ScanData, OdomData, GnssData
from slam_gnss_2d.core.component_factory import build_gnss_source, _build_matcher, _build_loop_matcher
from slam_gnss_2d.optimizer.gtsam_optimizer import GTSAMOptimizer
from slam_gnss_2d.map_manager import OverwriteRenderer, CountingRenderer
from slam_gnss_2d.core.slam_data_saver import SlamDataSaver
from slam_gnss_2d.input.ros2.bag_reader import BagScanSource
from slam_gnss_2d.core.pose_graph_reoptimizer import PoseGraphReoptimizer

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
        max_sigma_m=float(params.get('gnss.validation.max_sigma_m', 5.0))
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
        backend=params.get('optimization.backend', 'isam2'),
        isam2=Isam2Config(
            relinearize_threshold=float(params.get('optimization.isam2.relinearize_threshold', 0.1))
        )
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
    parser.add_argument("--enable-re-scan-matching", action="store_true", help="エッジのスキャンマッチングを再実行するかどうか（デフォルトはオフで既存エッジを再利用）")
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

    reoptimizer = PoseGraphReoptimizer(config, logger)
    optimized_nodes, new_edges, renderer = reoptimizer.reoptimize(
        pose_graph_data, gnss_transform_data, bag_path,
        enable_re_scan_matching=args.enable_re_scan_matching
    )

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
