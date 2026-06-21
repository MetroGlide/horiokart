import array
import math
from geometry_msgs.msg import Point as RosPoint, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

class SlamVisualizer:
    """RViz向け可視化とマップのパブリッシュを担当する"""

    def __init__(self, node: Node, use_gnss: bool):
        self._node = node
        self._use_gnss = use_gnss

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map_pub = node.create_publisher(OccupancyGrid, 'map', map_qos)
        self._path_pub = node.create_publisher(Path, 'slam_gnss_2d/path', 1)
        self._pg_marker_pub = node.create_publisher(MarkerArray, 'slam_gnss_2d/pose_graph', 1)
        self._path_before_pub = node.create_publisher(Path, 'slam_gnss_2d/path_before_optimize', 1)

        self._anchor_pub = node.create_publisher(NavSatFix, 'slam_gnss_2d/anchor', map_qos)
        self._anchor_published = False

        self._path_msg = Path()
        self._path_msg.header.frame_id = 'map'

    def publish_anchor(self, orchestrator) -> None:
        if self._use_gnss and not self._anchor_published:
            latlon = orchestrator.anchor_latlon
            if latlon is not None:
                lat, lon = latlon
                msg = NavSatFix()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                msg.latitude = lat
                msg.longitude = lon
                msg.status.status = 0  # STATUS_FIX
                self._anchor_pub.publish(msg)
                self._anchor_published = True
                self._node.get_logger().info(f'Anchor published: Lat={lat:.7f}, Lon={lon:.7f}')

    def publish_map(self, renderer) -> None:
        data, origin_x, origin_y, resolution = renderer.to_occupancy_array()
        msg = OccupancyGrid()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = resolution
        msg.info.width = int(data.shape[1])
        msg.info.height = int(data.shape[0])
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.data = array.array('b', data.ravel().tobytes())
        self._map_pub.publish(msg)
        self._node.get_logger().debug(
            f'Map published: occupied={int((data == 100).sum())}, '
            f'free={int((data == 0).sum())} px'
        )

    def publish_path_increment(self, node) -> None:
        pose = PoseStamped()
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = node.x
        pose.pose.position.y = node.y
        pose.pose.orientation.w = math.cos(node.yaw / 2.0)
        pose.pose.orientation.z = math.sin(node.yaw / 2.0)
        self._path_msg.header.stamp = pose.header.stamp
        self._path_msg.poses.append(pose)
        self._path_pub.publish(self._path_msg)

    def rebuild_path(self, nodes: list) -> None:
        now = self._node.get_clock().now().to_msg()
        self._path_msg.header.stamp = now
        self._path_msg.poses.clear()
        for n in nodes:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'map'
            pose.pose.position.x = n.x
            pose.pose.position.y = n.y
            pose.pose.orientation.w = math.cos(n.yaw / 2.0)
            pose.pose.orientation.z = math.sin(n.yaw / 2.0)
            self._path_msg.poses.append(pose)
        self._path_pub.publish(self._path_msg)

    def publish_path_before_optimize(self) -> None:
        self._path_before_pub.publish(self._path_msg)

    def publish_pose_graph_markers(self, pose_graph) -> None:
        nodes = pose_graph.get_nodes()
        if not nodes:
            return
        all_edges = pose_graph.get_edges()
        loop_edge_set: set[tuple[int, int]] = set()
        if hasattr(pose_graph, 'get_loop_edges'):
            loop_edge_set = {(e.from_index, e.to_index) for e in pose_graph.get_loop_edges()}
        node_by_idx = {n.index: n for n in nodes}
        now = self._node.get_clock().now().to_msg()
        array = MarkerArray()

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
        array.markers.append(node_m)

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

        for edge in all_edges:
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
        array.markers.append(seq_m)
        array.markers.append(loop_m)
        self._pg_marker_pub.publish(array)
