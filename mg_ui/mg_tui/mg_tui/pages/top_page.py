from textual.app import ComposeResult
from textual.widget import Widget
from textual.widgets import Label, Static

from ..state import AppState, DIAG_LEVEL
from ..widgets import DiagnosticsPanel


def _health_markup(error_count: int, warn_count: int) -> str:
    if error_count > 0:
        return f'[bold red]{error_count} ERROR[/]'
    if warn_count > 0:
        return f'[bold yellow]{warn_count} WARN[/]'
    return '[bold green]OK[/]'


class TopPage(Widget):
    DEFAULT_CSS = """
    TopPage { height: 1fr; overflow-y: auto; padding: 1; }
    TopPage .section-title { color: $text-muted; text-style: bold; margin-bottom: 1; }
    TopPage #health-summary { margin-bottom: 1; }
    TopPage #alert-section { border: solid $warning; padding: 1; margin-top: 1; }
    """

    def compose(self) -> ComposeResult:
        yield Label('System Health', classes='section-title')
        yield Static(id='health-summary')
        yield Static(id='health-detail')
        yield Static(id='alert-section')

    def refresh_data(self, state: AppState) -> None:
        items = state.diag_cache.get_all()
        error_count = sum(1 for lvl, _, _ in items if lvl >= 2)
        warn_count = sum(1 for lvl, _, _ in items if lvl == 1)
        total = len(items)

        self.query_one('#health-summary', Static).update(
            _health_markup(error_count, warn_count)
        )
        monitored = f'{total} items monitored' if total > 0 else 'waiting…'
        self.query_one('#health-detail', Static).update(monitored)

        alerts = [item for item in items if item[0] >= 1]
        if alerts:
            lines = [f'[dim]Alerts:[/]']
            for level, name, message in alerts:
                color = 'yellow' if level == 1 else 'red'
                label = DIAG_LEVEL.get(level, '?')
                lines.append(f'  [{color}]{label:5}[/] {name}: {message}')
            self.query_one('#alert-section', Static).update('\n'.join(lines))
        else:
            self.query_one('#alert-section', Static).update('')
