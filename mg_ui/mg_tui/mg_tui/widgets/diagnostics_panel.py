from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Label

from ..state import DIAG_LEVEL


def _level_markup(level: int, name: str, message: str) -> str:
    label = DIAG_LEVEL.get(level, '?')
    color = 'green' if level == 0 else ('yellow' if level == 1 else 'red')
    return f'[{color}]{label:5}[/] {name}: {message}'


class DiagnosticsPanel(Widget):
    """Diagnosticsキャッシュをレベル別色付きリストで表示するパネル。"""

    DEFAULT_CSS = """
    DiagnosticsPanel { height: auto; overflow-y: auto; }
    DiagnosticsPanel Label { height: 1; }
    """

    def compose(self) -> ComposeResult:
        yield Label('waiting…', id='diag-content')

    def refresh_data(self, items: list[tuple[int, str, str]]) -> None:
        if not items:
            self.query_one('#diag-content', Label).update('waiting…')
            return
        lines = [_level_markup(level, name, message)
                 for level, name, message in items]
        self.query_one('#diag-content', Label).update('\n'.join(lines))
