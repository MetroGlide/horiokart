import math
from rclpy.node import Node
from mg_msgs.srv import GetPoseGraph
from mg_msgs.msg import PoseGraphDiff
from slam_gnss_2d.pose_graph.base import PoseGraphBuilderBase
from slam_gnss_2d.core.graph_orchestrator import GraphOrchestrator

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
        loop_edge_set = {(e.from_index, e.to_index) for e in loop_edges}
        
        seq_edges = [e for e in all_edges if (e.from_index, e.to_index) not in loop_edge_set]

        diff = PoseGraphDiff()
        diff.full_refresh_needed = True

        for n in nodes:
            diff.new_node_indices.append(n.index)
            diff.new_node_x.append(float(n.x))
            diff.new_node_y.append(float(n.y))
            diff.new_node_yaw.append(float(n.yaw))
            diff.new_node_timestamps.append(float(n.timestamp))

        for e in seq_edges:
            diff.seq_edge_from.append(e.from_index)
            diff.seq_edge_to.append(e.to_index)
            diff.seq_edge_score.append(float(getattr(e, 'score', 0.0)))
            diff.seq_edge_type.append(1 if getattr(e, 'is_odom_fallback', False) else 0)
            diff.seq_edge_info_diag.extend([float(e.information[0,0]), float(e.information[1,1]), float(e.information[2,2])])

        for e in loop_edges:
            diff.loop_edge_from.append(e.from_index)
            diff.loop_edge_to.append(e.to_index)
            diff.loop_edge_score.append(float(getattr(e, 'score', 0.0)))
            diff.loop_edge_info_diag.extend([float(e.information[0,0]), float(e.information[1,1]), float(e.information[2,2])])

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
