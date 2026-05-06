from textual.app import ComposeResult
from textual.message import Message
from textual.widget import Widget
from textual.widgets import Label, Static, Input

from ..state import AppState
from ..widgets import CursorList

_BASE_ACTIONS = [
    'Start SLAM',
    'Stop SLAM',
    'Save Map',
]

_SIM_ACTIONS = [
    'Reset Robot Pose',
]


class SlamPage(Widget):

    class ActionSelected(Message):
        def __init__(self, label: str) -> None:
            super().__init__()
            self.label = label

    DEFAULT_CSS = """
    SlamPage { height: 1fr; overflow-y: auto; padding: 1; }
    SlamPage .section-title { color: $text-muted; text-style: bold; }
    """

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._action_labels = list(_BASE_ACTIONS)

    def compose(self) -> ComposeResult:
        yield Label('Container', classes='section-title')
        yield Static(id='slam-container-status')

        yield Label('Actions  [Enter] to execute', classes='section-title')
        yield CursorList(self._action_labels, id='action-list')

    def on_cursor_list_selected(self, event: CursorList.Selected) -> None:
        self.post_message(self.ActionSelected(event.label))

    def move_cursor_down(self) -> None:
        self.query_one('#action-list', CursorList).move_down()

    def move_cursor_up(self) -> None:
        self.query_one('#action-list', CursorList).move_up()

    def execute_cursor(self) -> None:
        self.query_one('#action-list', CursorList).select()

    def refresh_data(self, state: AppState) -> None:
        slam_state = state.containers.get('slam', '—')
        color = 'green' if slam_state == 'running' else 'dim'
        self.query_one('#slam-container-status', Static).update(
            f'SLAM: [{color}]{slam_state}[/]'
        )

        new_actions = list(_BASE_ACTIONS) + \
            (_SIM_ACTIONS if state.is_simulation else [])
        if new_actions != self._action_labels:
            self._action_labels = new_actions
            self.query_one(
                '#action-list', CursorList).update_items(new_actions)
