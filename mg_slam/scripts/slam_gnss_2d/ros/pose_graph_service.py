from rclpy.node import Node
from mg_msgs.srv import GetPoseGraph
from slam_gnss_2d.pose_graph.base import PoseGraphBuilderBase
from slam_gnss_2d.core.graph_orchestrator import GraphOrchestrator
from slam_gnss_2d.ros.pose_graph_message_builder import (
    build_pose_graph_diff,
    split_edges,
)

class PoseGraphService:
    def __init__(self, node: Node, pose_graph: PoseGraphBuilderBase, orchestrator: GraphOrchestrator):
        self._node = node
        self._pose_graph = pose_graph
        self._orchestrator = orchestrator
        self._srv = node.create_service(GetPoseGraph, 'slam_gnss_2d/get_pose_graph', self._callback)

    def _callback(self, request, response):
        nodes = self._pose_graph.get_nodes()
        all_edges = self._pose_graph.get_edges()
        
        loop_edges = []
        if hasattr(self._pose_graph, 'get_loop_edges'):
            loop_edges = self._pose_graph.get_loop_edges()
        seq_edges, loop_edges = split_edges(all_edges, loop_edges)
        diff = build_pose_graph_diff(
            nodes=nodes,
            seq_edges=seq_edges,
            loop_edges=loop_edges,
            full_refresh_needed=True,
        )

        response.graph = diff
        response.total_nodes = len(nodes)
        response.total_seq_edges = len(seq_edges)
        response.total_loop_edges = len(loop_edges)
        response.icp_attempt_count = getattr(self._pose_graph, 'icp_attempt_count', 0)
        response.icp_success_count = getattr(self._pose_graph, 'icp_success_count', 0)
        response.odom_fallback_count = getattr(self._pose_graph, 'odom_fallback_count', 0)
        response.loop_attempt_count = getattr(self._pose_graph, 'loop_attempt_count', 0)
        response.loop_success_count = getattr(self._pose_graph, 'loop_success_count', 0)

        return response
