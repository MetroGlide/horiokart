#!/usr/bin/env python3
"""MG-01 Terminal UI (Textual + rclpy)。

キーバインド:
  s  START waypoint navigation (3s countdown)
  x  STOP waypoint navigation
  p  PAUSE
  r  RESUME
  m  Save map (SLAM)
  q  Quit
"""
import json
import threading
import time
from dataclasses import dataclass, field

import requests

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Int16
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from mg_msgs.msg import SequencerStatus, PauseRequest

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Header, Footer, Static, Label
from textual.containers import Horizontal, Vertical


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
    """path has leading slash. Returns relative name if ns given, absolute if not."""
    if ns:
        return f'/{ns}/{path.lstrip("/")}'
    return path


class _Topics:
    WAYPOINT_STATUS = _node_ns(_NODE_NS['WAYPOINT_SEQUENCER'], '/status')
    WAYPOINT_PAUSE_REQUEST = _node_ns(
        _NODE_NS['WAYPOINT_SEQUENCER'], '/pause_request')
    WAYPOINT_SET_NEXT_INDEX = _node_ns(
        _NODE_NS['WAYPOINT_SEQUENCER'], '/set_next_waypoint_index')
    DIAGNOSTICS = _node_ns(_NODE_NS['DIAGNOSTICS'], '/diagnostics')
    AMCL_POSE = _node_ns(_NODE_NS['LOCALIZATION'], '/amcl_pose')


class _Services:
    WAYPOINT_START = _node_ns(_NODE_NS['WAYPOINT_SEQUENCER'], '/start')
    WAYPOINT_STOP = _node_ns(_NODE_NS['WAYPOINT_SEQUENCER'], '/stop')


_DIAG_LEVEL = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}
_STATE_STYLE = {
    'IDLE': 'dim',
    'NAVIGATING': 'bold cyan',
    'SUSPENDED': 'bold yellow',
    'ERROR': 'bold red',
    'GOAL_REACHED': 'bold green',
    'ON_STARTING': 'yellow',
    'ON_ARRIVING': 'cyan',
}


@dataclass
class AppState:
    seq_state: str = '—'
    seq_index: int = 0
    seq_total: int = 0
    seq_distance: float = 0.0
    seq_countdown_ms: int = 0
    is_paused: bool = False
    pause_requesters: list[str] = field(default_factory=list)
    diag_items: list[tuple[int, str, str]] = field(default_factory=list)
    containers: dict[str, str] = field(default_factory=dict)
    amcl_trace_xy: float = 0.0
    last_service_msg: str = ''


class RosBackend(Node):
    def __init__(self, state: AppState, refresh_cb):
        super().__init__('mg_tui_node')
        self._state = state
        self._refresh = refresh_cb

        self.create_subscription(
            SequencerStatus,
            _Topics.WAYPOINT_STATUS,
            self._on_sequencer,
            _BEST_EFFORT_QOS,
        )
        self.create_subscription(
            DiagnosticArray,
            _Topics.DIAGNOSTICS,
            self._on_diagnostics,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            _Topics.AMCL_POSE,
            self._on_amcl,
            10,
        )
        self._pause_pub = self.create_publisher(
            PauseRequest, _Topics.WAYPOINT_PAUSE_REQUEST, 10)
        self._jump_pub = self.create_publisher(
            Int16, _Topics.WAYPOINT_SET_NEXT_INDEX, 10)

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
        self._state.amcl_trace_xy = cov[0] + cov[7]
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
            self._state.last_service_msg = f'{service}: {"OK" if result.success else "FAIL"} {result.message}'
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


class MgTuiApp(App):
    CSS = """
    Screen { layout: vertical; }
    #top-row { height: 1fr; layout: horizontal; }
    #seq-panel { width: 1fr; border: solid $primary; padding: 1; }
    #system-panel { width: 1fr; border: solid $primary; padding: 1; }
    #diag-panel { height: 1fr; border: solid $warning; padding: 1; overflow-y: auto; }
    #status-bar { height: 1; background: $surface; }
    Label.section-title { color: $text-muted; text-style: bold; }
    """

    BINDINGS = [
        Binding('s', 'start_nav', 'Start'),
        Binding('x', 'stop_nav', 'Stop'),
        Binding('p', 'pause_nav', 'Pause'),
        Binding('r', 'resume_nav', 'Resume'),
        Binding('m', 'save_map', 'Save Map'),
        Binding('q', 'quit', 'Quit'),
    ]

    def __init__(self, backend: RosBackend, state: AppState):
        super().__init__()
        self._backend = backend
        self._state = state

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id='top-row'):
            with Vertical(id='seq-panel'):
                yield Label('Waypoint Navigation', classes='section-title')
                yield Static(id='seq-state')
                yield Static(id='seq-detail')
            with Vertical(id='system-panel'):
                yield Label('System', classes='section-title')
                yield Static(id='system-detail')
        with Vertical(id='diag-panel'):
            yield Label('Diagnostics', classes='section-title')
            yield Static(id='diag-detail')
        yield Static(id='status-bar')
        yield Footer()

    def on_mount(self) -> None:
        self.set_interval(0.5, self._update_ui)

    def _update_ui(self) -> None:
        s = self._state

        style = _STATE_STYLE.get(s.seq_state, '')
        state_text = f'[{style}]{s.seq_state}[/]' if style else s.seq_state
        self.query_one('#seq-state', Static).update(state_text)

        detail_lines = [
            f'Index: {s.seq_index + 1} / {s.seq_total}',
            f'Remaining: {s.seq_distance:.1f} m',
        ]
        if s.seq_countdown_ms > 0:
            detail_lines.append(
                f'[yellow]Countdown: {s.seq_countdown_ms / 1000:.1f} s[/]')
        if s.is_paused:
            detail_lines.append(
                f'[yellow]Paused by: {", ".join(s.pause_requesters)}[/]')
        self.query_one('#seq-detail', Static).update('\n'.join(detail_lines))

        slam_state = s.containers.get('slam', '—')
        nav_state = s.containers.get('navigation', '—')
        sys_lines = [
            f'SLAM: [{"green" if slam_state == "running" else "dim"}]{slam_state}[/]',
            f'NAV:  [{"green" if nav_state == "running" else "dim"}]{nav_state}[/]',
            f'AMCL cov: {s.amcl_trace_xy:.3f}',
        ]
        self.query_one('#system-detail', Static).update('\n'.join(sys_lines))

        diag_lines = []
        for level, name, message in s.diag_items:
            label = _DIAG_LEVEL.get(level, '?')
            color = 'green' if level == 0 else (
                'yellow' if level == 1 else 'red')
            diag_lines.append(f'[{color}]{label:5}[/] {name}: {message}')
        self.query_one(
            '#diag-detail', Static).update('\n'.join(diag_lines) or 'waiting…')

        self.query_one('#status-bar', Static).update(s.last_service_msg)

    def action_start_nav(self) -> None:
        self._backend.call_trigger(_Services.WAYPOINT_START)

    def action_stop_nav(self) -> None:
        self._backend.call_trigger(_Services.WAYPOINT_STOP)

    def action_pause_nav(self) -> None:
        self._backend.publish_pause(active=True)

    def action_resume_nav(self) -> None:
        self._backend.publish_pause(active=False)

    def action_save_map(self) -> None:
        def _do() -> None:
            try:
                r = requests.post('http://localhost:8001/map/save', timeout=35)
                data = r.json()
                self._state.last_service_msg = (
                    f'save_map: {"OK" if data["success"] else "FAIL"} {data["message"]}'
                )
            except Exception as e:
                self._state.last_service_msg = f'save_map: error {e}'
            self.call_from_thread(self._update_ui)
        threading.Thread(target=_do, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    state = AppState()

    app = MgTuiApp.__new__(MgTuiApp)

    backend = RosBackend(state, lambda: app.call_from_thread(app._update_ui))

    MgTuiApp.__init__(app, backend, state)

    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(backend),
        daemon=True,
    )
    ros_thread.start()

    def _poll_containers() -> None:
        while True:
            try:
                r = requests.get('http://localhost:8001/status', timeout=2)
                state.containers = r.json()
                app.call_from_thread(app._update_ui)
            except Exception:
                pass
            time.sleep(2.0)

    threading.Thread(target=_poll_containers, daemon=True).start()

    try:
        app.run()
    finally:
        backend.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
