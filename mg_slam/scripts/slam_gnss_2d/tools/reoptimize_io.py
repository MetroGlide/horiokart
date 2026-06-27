from __future__ import annotations

import json
import os

import yaml


def load_pose_graph_and_transform(input_dir: str) -> tuple[dict, dict]:
    """入力ディレクトリから pose_graph.json と gnss_transform.yaml を読み込む。"""
    pose_graph_path = os.path.join(input_dir, 'pose_graph.json')
    gnss_transform_path = os.path.join(input_dir, 'gnss_transform.yaml')

    if not os.path.exists(pose_graph_path):
        raise FileNotFoundError(f'pose_graph.json not found in {input_dir}')
    if not os.path.exists(gnss_transform_path):
        raise FileNotFoundError(
            f'gnss_transform.yaml not found in {input_dir}')

    with open(pose_graph_path, 'r') as pose_graph_file:
        pose_graph_data = json.load(pose_graph_file)
    with open(gnss_transform_path, 'r') as gnss_transform_file:
        gnss_transform_data = yaml.safe_load(gnss_transform_file)

    return pose_graph_data, gnss_transform_data


def resolve_bag_path(configured_bag_path: str, pose_graph_data: dict) -> str:
    """優先度: 明示指定 bag_path > pose_graph metadata の bag_path。"""
    bag_path = configured_bag_path
    if not bag_path:
        bag_path = pose_graph_data.get('metadata', {}).get('bag_path', '')

    if not bag_path or not os.path.exists(bag_path):
        raise FileNotFoundError(
            f"ROS Bag path '{bag_path}' is invalid or file does not exist"
        )

    return bag_path
