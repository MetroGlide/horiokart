import json
import yaml
import os
from datetime import datetime
from typing import Optional, List, Dict, Any
from .data_types import PoseNode, PoseEdge

class SlamDataSaver:
    @staticmethod
    def save_gnss_transform(
        output_dir: str,
        anchor_lat: float,
        anchor_lon: float,
        anchor_utm_easting: float,
        anchor_utm_northing: float,
        utm_zone: int,
        utm_hemisphere: str,
        rotation_rad: float,
        backend_name: str = "unknown"
    ) -> str:
        """
        GNSSとSLAM座標系の変換パラメータを gnss_transform.yaml として保存する
        """
        data = {
            "anchor": {
                "latitude": anchor_lat,
                "longitude": anchor_lon
            },
            "anchor_utm": {
                "easting": anchor_utm_easting,
                "northing": anchor_utm_northing,
                "zone": utm_zone,
                "hemisphere": utm_hemisphere
            },
            "rotation_rad": rotation_rad,
            "metadata": {
                "created_at": datetime.now().isoformat(),
                "slam_backend": backend_name
            }
        }

        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, "gnss_transform.yaml")
        with open(filepath, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        return filepath

    @staticmethod
    def save_pose_graph(
        output_dir: str,
        nodes: List[PoseNode],
        edges: List[PoseEdge],
        bag_path: Optional[str] = None
    ) -> str:
        """
        ポーズグラフを pose_graph.json として保存する
        """
        nodes_data = [
            {
                "index": n.index,
                "timestamp": n.timestamp,
                "x": float(n.x),
                "y": float(n.y),
                "yaw": float(n.yaw)
            } for n in nodes
        ]

        # 簡易的に全てのEdgeを sequential と loop で分ける
        sequential_edges = []
        loop_edges = []
        for e in edges:
            edge_data = {
                "from": e.from_index,
                "to": e.to_index,
                "dx": float(e.dx),
                "dy": float(e.dy),
                "dyaw": float(e.dyaw),
                "information": [float(x) for x in e.information.flatten().tolist()] if hasattr(e.information, 'flatten') else e.information
            }
            if abs(e.to_index - e.from_index) == 1:
                sequential_edges.append(edge_data)
            else:
                loop_edges.append(edge_data)

        data = {
            "metadata": {
                "created_at": datetime.now().isoformat(),
                "num_nodes": len(nodes),
                "num_sequential_edges": len(sequential_edges),
                "num_loop_edges": len(loop_edges),
                "bag_path": bag_path
            },
            "nodes": nodes_data,
            "sequential_edges": sequential_edges,
            "loop_edges": loop_edges,
            "gnss_priors": []  # 必要であれば追加
        }

        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, "pose_graph.json")
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        return filepath
