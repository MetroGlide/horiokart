from textual.app import ComposeResult
from textual.message import Message
from textual.reactive import reactive
from textual.widget import Widget
from textual.widgets import Label

from ..state import AppState
from ..widgets import ServiceRow, DiagnosticsPanel

MANAGED_SERVICES = [
    ('navigation', 'Navigation'),
    ('slam', 'SLAM'),
    ('foxglove-bridge', 'Foxglove Bridge'),
    ('diagnostics', 'Diagnostics'),
    ('waypoint-editor', 'Waypoint Editor'),
    ('gazebo-simulation', 'Gazebo Simulation'),
    ('rviz2', 'RViz2'),
    ('rviz2-navigation', 'RViz2 Navigation'),
    ('rviz2-slam', 'RViz2 SLAM'),
]


class SystemPage(Widget):

    class ServiceStartRequested(Message):
        def __init__(self, service_key: str) -> None:
            super().__init__()
            self.service_key = service_key

    class ServiceStopRequested(Message):
        def __init__(self, service_key: str) -> None:
            super().__init__()
            self.service_key = service_key

    DEFAULT_CSS = """
    SystemPage { height: 1fr; overflow-y: auto; padding: 1; }
    SystemPage .section-title { color: $text-muted; text-style: bold; margin-bottom: 1; }
    SystemPage #diag-section { border: solid $primary; padding: 1; margin-top: 1; }
    SystemPage #service-hint { color: $text-muted; height: 1; margin-bottom: 1; }
    """

    cursor: reactive[int] = reactive(0)

    def compose(self) -> ComposeResult:
        yield Label('Services  [j/k] move  [s] Start  [x] Stop', id='service-hint')
        for key, label in MANAGED_SERVICES:
            yield ServiceRow(key, label, id=f'svc-{key}')

        yield Label('Diagnostics', classes='section-title', id='diag-section-title')
        yield DiagnosticsPanel(id='diag-panel')

    def on_mount(self) -> None:
        self._update_focus()

    def on_service_row_start_requested(self, event: ServiceRow.StartRequested) -> None:
        self.post_message(self.ServiceStartRequested(event.service_key))

    def on_service_row_stop_requested(self, event: ServiceRow.StopRequested) -> None:
        self.post_message(self.ServiceStopRequested(event.service_key))

    def _update_focus(self) -> None:
        for i, (key, _) in enumerate(MANAGED_SERVICES):
            row = self.query_one(f'#svc-{key}', ServiceRow)
            row.focused = (i == self.cursor)

    def move_cursor_down(self) -> None:
        self.cursor = (self.cursor + 1) % len(MANAGED_SERVICES)
        self._update_focus()

    def move_cursor_up(self) -> None:
        self.cursor = (self.cursor - 1) % len(MANAGED_SERVICES)
        self._update_focus()

    def action_start_focused(self) -> None:
        key = MANAGED_SERVICES[self.cursor][0]
        self.post_message(self.ServiceStartRequested(key))

    def action_stop_focused(self) -> None:
        key = MANAGED_SERVICES[self.cursor][0]
        self.post_message(self.ServiceStopRequested(key))

    def refresh_data(self, state: AppState) -> None:
        for key, _ in MANAGED_SERVICES:
            status = state.containers.get(key, '—')
            self.query_one(f'#svc-{key}', ServiceRow).update_status(status)

        self.query_one('#diag-panel', DiagnosticsPanel).refresh_data(
            state.diag_cache.get_all()
        )
