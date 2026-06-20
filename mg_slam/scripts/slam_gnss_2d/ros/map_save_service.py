import math
from std_srvs.srv import Trigger
from rclpy.node import Node
from slam_gnss_2d.core.slam_data_saver import SlamDataSaver

class MapSaveService:
    """SLAM結果（マップ、ポーズグラフ、GNSSアンカー）の保存処理を担当する"""

    def __init__(self, node: Node, pose_graph, orchestrator):
        self._node = node
        self._pose_graph = pose_graph
        self._orchestrator = orchestrator
        self._save_srv = node.create_service(
            Trigger, 'slam_gnss_2d/save_slam_map', self._handle_save_slam_map
        )

    def _handle_save_slam_map(self, request, response):
        output_dir = self._node.get_parameter('save_dir').value
        if not output_dir:
            output_dir = '/root/ros2_data/slam_maps/latest'
        
        try:
            nodes = self._pose_graph.get_nodes()
            edges = self._pose_graph.get_edges()

            bag_path = None
            if self._node.has_parameter('bag_path'):
                bag_path = self._node.get_parameter('bag_path').value
                if not bag_path:
                    bag_path = None
            pg_path = SlamDataSaver.save_pose_graph(output_dir, nodes, edges, bag_path=bag_path)
            msg_parts = [f"PoseGraph saved: {pg_path}"]

            if self._orchestrator.anchor_latlon is not None:
                anchor = self._orchestrator.anchor
                lat, lon = self._orchestrator.anchor_latlon
                zone = int(math.floor((lon + 180.0) / 6.0)) + 1
                hemisphere = "north" if lat >= 0 else "south"
                rotation_rad = self._orchestrator.init_rotation if self._orchestrator.init_rotation is not None else 0.0

                gnss_path = SlamDataSaver.save_gnss_transform(
                    output_dir=output_dir,
                    anchor_lat=lat,
                    anchor_lon=lon,
                    anchor_utm_easting=anchor[0] if anchor else 0.0,
                    anchor_utm_northing=anchor[1] if anchor else 0.0,
                    utm_zone=zone,
                    utm_hemisphere=hemisphere,
                    rotation_rad=rotation_rad,
                    backend_name=self._node.get_parameter('optimization.backend').value
                )
                msg_parts.append(f"GNSS transform saved: {gnss_path}")

            response.success = True
            response.message = "; ".join(msg_parts)
            self._node.get_logger().info(f"SLAM map saved successfully: {response.message}")

        except Exception as e:
            response.success = False
            response.message = f"Failed to save SLAM map: {e}"
            self._node.get_logger().error(response.message)

        return response
