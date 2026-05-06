from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Int16
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from mg_msgs.msg import SequencerStatus, PauseRequest

from .state import AppState

_BEST_EFFORT_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

_NODE_NS = {
    'WAYPOINT_SEQUENCER': 'waypoint_sequencer_node',
    'DIAGNOSTICS': '',
    'LOCALIZATION': '',
}


def _node_ns(ns: str, path: str) -> str:
    if ns:
        return f'/{ns}/{path.lstrip("/")}'
    return path


class Topics:
    WAYPOINT_STATUS = _node_ns(_NODE_NS['WAYPOINT_SEQUENCER'], '/status')
    WAYPOINT_PAUSE_REQUEST = _node_ns(
        _NODE_NS['WAYPOINT_SEQUENCER'], '/pause_request')
    WAYPOINT_SET_NEXT_INDEX = _node_ns(
        _NODE_NS['WAYPOINT_SEQUENCER'], '/set_next_waypoint_index')
    DIAGNOSTICS = _node_ns(_NODE_NS['DIAGNOSTICS'], '/diagnostics')
    AMCL_POSE = _node_ns(_NODE_NS['LOCALIZATION'], '/amcl_pose')


class Services:
    WAYPOINT_START = _node_ns(_NODE_NS['WAYPOINT_SEQUENCER'], '/start')
    WAYPOINT_STOP = _node_ns(_NODE_NS['WAYPOINT_SEQUENCER'], '/stop')


class RosBackend(Node):
    def __init__(self, state: AppState, refresh_cb):
        super().__init__('mg_tui_node')
        self._state = state
        self._refresh = refresh_cb

        self.create_subscription(
            SequencerStatus,
            Topics.WAYPOINT_STATUS,
            self._on_sequencer,
            _BEST_EFFORT_QOS,
        )
        self.create_subscription(
            DiagnosticArray,
            Topics.DIAGNOSTICS,
            self._on_diagnostics,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            Topics.AMCL_POSE,
            self._on_amcl,
            10,
        )
        self._pause_pub = self.create_publisher(
            PauseRequest, Topics.WAYPOINT_PAUSE_REQUEST, 10)
        self._jump_pub = self.create_publisher(
            Int16, Topics.WAYPOINT_SET_NEXT_INDEX, 10)

    def _on_sequencer(self, msg: SequencerStatus) -> None:
        self._state.seq_state = msg.state
        self._state.seq_index = msg.current_index
        self._state.seq_total = msg.total_waypoints
        self._state.seq_distance = msg.distance_remaining
        self._state.seq_countdown_ms = msg.countdown_ms_remaining
        self._state.is_paused = msg.is_paused
        self._state.pause_requesters = list(msg.pause_requesters)
        self._refresh()

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        self._state.diag_items = [
            (s.level, s.name, s.message) for s in msg.status
        ]
        self._refresh()

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        cov = msg.pose.covariance
        self._state.amcl_cov_xy = cov[0] + cov[7]
        self._refresh()

    def call_trigger(self, service: str) -> None:
        client = self.create_client(Trigger, service)
        if not client.wait_for_service(timeout_sec=2.0):
            self._state.last_service_msg = f'service not available: {service}'
            self._refresh()
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._on_srv_done(f, service))

    def _on_srv_done(self, future, service: str) -> None:
        try:
            result = future.result()
            self._state.last_service_msg = (
                f'{service}: {"OK" if result.success else "FAIL"} {result.message}'
            )
        except Exception as e:
            self._state.last_service_msg = f'{service}: exception {e}'
        self._refresh()

    def publish_pause(self, active: bool) -> None:
        msg = PauseRequest()
        msg.requester_id = 'tui'
        msg.active = active
        msg.heartbeat_period_s = 0.0
        msg.reason = 'manual' if active else ''
        self._pause_pub.publish(msg)

    def publish_jump(self, index: int) -> None:
        msg = Int16()
        msg.data = index
        self._jump_pub.publish(msg)
