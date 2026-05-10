from textual.app import ComposeResult
from textual.message import Message
from textual.reactive import reactive
from textual.widget import Widget
from textual.widgets import Label
from textual.containers import Horizontal


def _status_markup(status: str) -> str:
    color = 'green' if status == 'running' else 'dim'
    return f'[{color}]{status}[/]'


class ServiceRow(Widget):
    """サービス名・コンテナ状態・Start/Stop 操作行。"""

    DEFAULT_CSS = """
    ServiceRow { height: 1; layout: horizontal; }
    ServiceRow .svc-name { width: 22; color: $text-muted; }
    ServiceRow .svc-status { width: 10; }
    ServiceRow .svc-hint { width: 14; color: $text-muted; }
    ServiceRow.row--focused .svc-name { color: $text; text-style: bold reverse; }
    """

    class StartRequested(Message):
        def __init__(self, service_key: str) -> None:
            super().__init__()
            self.service_key = service_key

    class StopRequested(Message):
        def __init__(self, service_key: str) -> None:
            super().__init__()
            self.service_key = service_key

    focused: reactive[bool] = reactive(False)

    def __init__(self, service_key: str, label: str, **kwargs) -> None:
        super().__init__(**kwargs)
        self.service_key = service_key
        self._label = label

    def compose(self) -> ComposeResult:
        yield Label(self._label, classes='svc-name')
        yield Label('—', id='svc-status', classes='svc-status')
        yield Label('[s]Start [x]Stop', classes='svc-hint')

    def update_status(self, status: str) -> None:
        self.query_one('#svc-status', Label).update(_status_markup(status))

    def watch_focused(self, focused: bool) -> None:
        if focused:
            self.add_class('row--focused')
        else:
            self.remove_class('row--focused')

    def action_start(self) -> None:
        self.post_message(self.StartRequested(self.service_key))

    def action_stop(self) -> None:
        self.post_message(self.StopRequested(self.service_key))
