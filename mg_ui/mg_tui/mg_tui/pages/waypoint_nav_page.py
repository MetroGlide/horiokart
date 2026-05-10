from textual.app import ComposeResult
from textual.containers import Horizontal
from textual.message import Message
from textual.widget import Widget
from textual.widgets import Label, Static, Input

from ..state import AppState, STATE_STYLE, GOAL_STATUS_LABEL
from ..widgets import CursorList

_BASE_ACTIONS = [
    'Start (3s)',
    'Start Now',
    'Stop',
    'Pause',
    'Resume',
    'Reload Waypoints',
]

_SIM_ACTIONS = [
    'Reset Robot Pose',
    'Reset AMCL Pose',
]


class WaypointNavPage(Widget):

    class ActionSelected(Message):
        def __init__(self, label: str) -> None:
            super().__init__()
            self.label = label

    DEFAULT_CSS = """
    WaypointNavPage { height: 1fr; overflow-y: auto; padding: 1; }
    WaypointNavPage .section-title { color: $text-muted; text-style: bold; }
    WaypointNavPage .kv-row { height: 1; }
    WaypointNavPage #jump-row { layout: horizontal; height: 3; margin-top: 1; }
    WaypointNavPage #jump-input { width: 8; }
    WaypointNavPage #jump-hint { width: 1fr; color: $text-muted; }
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._is_simulation = False
        self._action_labels = list(_BASE_ACTIONS)

    def compose(self) -> ComposeResult:
        yield Label('Container', classes='section-title')
        yield Static(id='nav-container-status')

        yield Label('Sequencer', classes='section-title')
        yield Static(id='seq-state')
        yield Static(id='seq-detail')

        yield Label('Safety', classes='section-title')
        yield Static(id='safety-detail')

        yield Label('Nav2', classes='section-title')
        yield Static(id='nav2-detail')

        yield Label('Actions  [Enter] to execute', classes='section-title')
        yield CursorList(self._action_labels, id='action-list')

        with Horizontal(id='jump-row'):
            yield Input(placeholder='idx', id='jump-input')
            yield Label(' [J] Jump to waypoint', id='jump-hint')

    def on_cursor_list_selected(self, event: CursorList.Selected) -> None:
        self.post_message(self.ActionSelected(event.label))

    def move_cursor_down(self) -> None:
        self.query_one('#action-list', CursorList).move_down()

    def move_cursor_up(self) -> None:
        self.query_one('#action-list', CursorList).move_up()

    def execute_cursor(self) -> None:
        self.query_one('#action-list', CursorList).select()

    def get_jump_index(self) -> int | None:
        val = self.query_one('#jump-input', Input).value.strip()
        try:
            return int(val)
        except ValueError:
            return None

    def refresh_data(self, state: AppState) -> None:
        nav_state = state.containers.get('navigation', '—')
        color = 'green' if nav_state == 'running' else 'dim'
        self.query_one('#nav-container-status', Static).update(
            f'Navigation: [{color}]{nav_state}[/]'
        )

        style = STATE_STYLE.get(state.seq_state, '')
        state_text = f'[{style}]{state.seq_state}[/]' if style else state.seq_state
        self.query_one('#seq-state', Static).update(state_text)

        detail_lines = [
            f'Waypoint: {state.seq_index + 1} / {state.seq_total}',
            f'Remaining: {state.seq_distance:.1f} m',
        ]
        if state.seq_countdown_ms > 0:
            detail_lines.append(
                f'[yellow]Countdown: {state.seq_countdown_ms / 1000:.1f} s[/]')
        if state.is_paused:
            detail_lines.append(
                f'[yellow]Paused by: {", ".join(state.pause_requesters)}[/]')
        self.query_one('#seq-detail', Static).update('\n'.join(detail_lines))

        es_color = 'red' if state.emergency_stop else 'green'
        es_label = 'ACTIVE' if state.emergency_stop else 'OK'
        col_text = ', '.join(state.collision_polygons_active) or 'none'
        self.query_one('#safety-detail', Static).update(
            f'Emergency Stop: [{es_color}]{es_label}[/]\n'
            f'Collision Polygons: {col_text}'
        )

        action_label = (
            GOAL_STATUS_LABEL.get(state.nav2_action_status,
                                  str(state.nav2_action_status))
            if state.nav2_action_status is not None
            else '—'
        )
        self.query_one('#nav2-detail', Static).update(
            f'Nav2 Action: {action_label}\n'
            f'AMCL cov: {state.amcl_cov_xy:.3f}'
        )

        new_actions = list(_BASE_ACTIONS) + \
            (_SIM_ACTIONS if state.is_simulation else [])
        if new_actions != self._action_labels:
            self._action_labels = new_actions
            self.query_one(
                '#action-list', CursorList).update_items(new_actions)
        self._is_simulation = state.is_simulation
