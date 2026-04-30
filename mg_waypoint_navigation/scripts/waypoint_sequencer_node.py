#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Int16
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from mg_msgs.msg import PauseRequest, SequencerStatus
from mg_msgs.msg import WaypointInfo
from mg_msgs.msg import WaypointList as WaypointListMsg
from mg_msgs.srv import StartSequence

from mg_waypoint_navigation.waypoint import WaypointList, WaypointsLoader
from mg_waypoint_navigation.waypoint_sequencer.fsm import WaypointSequencerFSM
from mg_waypoint_navigation.waypoint_sequencer.states import CommandResult, SequencerState


class WaypointSequencerNode(Node):
    def __init__(self):
        super().__init__("waypoint_sequencer_node")

        self._declare_parameters()

        self._fsm = WaypointSequencerFSM(self)
        self._fsm.set_on_state_changed(self._on_fsm_state_changed)

        self._init_ros_communications()
        self._load_waypoints()

    # ------------------------------------------------------------------
    # パラメータ
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter("load_path", "")
        self.declare_parameter("publish_waypoint_status", True)
        self.declare_parameter("waypoint_status_freq_hz", 10.0)
        self.declare_parameter("publish_waypoints_list", True)

        self._load_path: str = self.get_parameter("load_path").value
        self._publish_status: bool = self.get_parameter(
            "publish_waypoint_status").value
        self._status_freq: float = self.get_parameter(
            "waypoint_status_freq_hz").value
        self._publish_list: bool = self.get_parameter(
            "publish_waypoints_list").value

    # ------------------------------------------------------------------
    # ROS 通信初期化
    # ------------------------------------------------------------------

    def _init_ros_communications(self):
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._start_srv = self.create_service(
            StartSequence, "~/start", self._on_start_srv
        )
        self._stop_srv = self.create_service(
            Trigger, "~/stop", self._on_stop_srv)

        self._set_index_sub = self.create_subscription(
            Int16, "~/set_next_waypoint_index", self._on_set_index, 1
        )
        self._pause_request_sub = self.create_subscription(
            PauseRequest, "~/pause_request", self._on_pause_request, 10
        )

        if self._publish_status:
            self._status_pub = self.create_publisher(
                SequencerStatus, "~/status", 10
            )
            self._status_timer = self.create_timer(
                1.0 / self._status_freq, self._publish_status_cb
            )

        if self._publish_list:
            self._waypoints_list_pub = self.create_publisher(
                WaypointListMsg, "~/waypoints", latched_qos
            )

        self._markers_pub = self.create_publisher(
            MarkerArray, "~/waypoints_markers", latched_qos
        )

    # ------------------------------------------------------------------
    # ウェイポイントロード
    # ------------------------------------------------------------------

    def _load_waypoints(self):
        if not self._load_path:
            self.get_logger().warn("load_path not set. No waypoints loaded.")
            return

        try:
            loader = WaypointsLoader(self._load_path)
            waypoints = loader.load()
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return

        self._fsm.load_waypoints(waypoints)
        self.get_logger().info(
            f"Loaded {waypoints.get_size()} waypoints from {self._load_path}"
        )

        self._publish_waypoints_list(waypoints)
        self._publish_markers(waypoints)

    # ------------------------------------------------------------------
    # サービスコールバック
    # ------------------------------------------------------------------

    def _on_start_srv(
        self, request: StartSequence.Request, response: StartSequence.Response
    ):
        result = self._fsm.start(request.countdown_ms)
        response.success = result.success
        response.message = result.message
        return response

    def _on_stop_srv(self, request: Trigger.Request, response: Trigger.Response):
        result = self._fsm.stop()
        response.success = result.success
        response.message = result.message
        return response

    # ------------------------------------------------------------------
    # トピックコールバック
    # ------------------------------------------------------------------

    def _on_set_index(self, msg: Int16):
        ok = self._fsm.set_next_index(int(msg.data))
        if not ok:
            self.get_logger().warn(
                f"Cannot set index to {msg.data} in state {self._fsm.state.value}"
            )

    def _on_pause_request(self, msg: PauseRequest):
        self._fsm.pause_request(
            requester_id=msg.requester_id,
            active=msg.active,
            heartbeat_period_s=msg.heartbeat_period_s,
            reason=msg.reason,
        )

    # ------------------------------------------------------------------
    # FSM 状態変化コールバック
    # ------------------------------------------------------------------

    def _on_fsm_state_changed(self, new_state: SequencerState):
        pass

    # ------------------------------------------------------------------
    # パブリッシュ
    # ------------------------------------------------------------------

    def _publish_status_cb(self):
        msg = SequencerStatus()
        now = self.get_clock().now().to_msg()
        msg.stamp = now
        msg.state = self._fsm.state.value
        msg.current_index = self._fsm.current_index
        msg.total_waypoints = self._fsm.total_waypoints
        msg.countdown_ms_remaining = self._fsm.countdown_ms_remaining
        msg.is_paused = self._fsm.state == SequencerState.SUSPENDED
        msg.pause_requesters = self._fsm.pause_requesters
        msg.distance_remaining = self._fsm.distance_remaining
        self._status_pub.publish(msg)

    def _publish_waypoints_list(self, waypoints: WaypointList):
        msg = WaypointListMsg()
        msg.loaded_time = self.get_clock().now().to_msg()
        for wp in waypoints.get_all():
            info = WaypointInfo()
            info.index = wp.index
            info.pose = wp.pose
            info.reach_tolerance = wp.navigation.reach_tolerance
            info.is_through_point = wp.navigation.is_through_point
            info.on_reached_action_types = [
                a.type for a in wp.on_reached_actions
            ]
            msg.waypoints.append(info)
        self._waypoints_list_pub.publish(msg)

    def _publish_markers(self, waypoints: WaypointList):
        marker_array = MarkerArray()
        for wp in waypoints.get_all():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = wp.index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = wp.pose.pose
            marker.pose.position.z = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.25
            marker.scale.z = 0.25
            if wp.navigation.is_through_point:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            else:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker_array.markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "waypoints_text"
            text_marker.id = wp.index
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = wp.pose.pose
            text_marker.pose.position.z = 1.5
            text_marker.scale.z = 0.4
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = str(wp.index)
            marker_array.markers.append(text_marker)

        self._markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSequencerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
