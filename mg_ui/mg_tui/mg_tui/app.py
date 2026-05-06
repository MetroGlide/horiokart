from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.widgets import Header, Footer, Static, Label
from textual.containers import Horizontal, Vertical

from .state import AppState, DIAG_LEVEL, STATE_STYLE
from .ros_backend import RosBackend, Services
from .system_client import SystemClient


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

    def __init__(self, backend: RosBackend, sys_client: SystemClient, state: AppState):
        super().__init__()
        self._backend = backend
        self._sys_client = sys_client
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

        style = STATE_STYLE.get(s.seq_state, '')
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
            f'AMCL cov: {s.amcl_cov_xy:.3f}',
        ]
        self.query_one('#system-detail', Static).update('\n'.join(sys_lines))

        diag_lines = []
        for level, name, message in s.diag_items:
            label = DIAG_LEVEL.get(level, '?')
            color = 'green' if level == 0 else (
                'yellow' if level == 1 else 'red')
            diag_lines.append(f'[{color}]{label:5}[/] {name}: {message}')
        self.query_one('#diag-detail', Static).update(
            '\n'.join(diag_lines) or 'waiting…'
        )

        self.query_one('#status-bar', Static).update(s.last_service_msg)

    def action_start_nav(self) -> None:
        self._backend.call_trigger(Services.WAYPOINT_START)

    def action_stop_nav(self) -> None:
        self._backend.call_trigger(Services.WAYPOINT_STOP)

    def action_pause_nav(self) -> None:
        self._backend.publish_pause(active=True)

    def action_resume_nav(self) -> None:
        self._backend.publish_pause(active=False)

    def action_save_map(self) -> None:
        self._sys_client.save_map()
