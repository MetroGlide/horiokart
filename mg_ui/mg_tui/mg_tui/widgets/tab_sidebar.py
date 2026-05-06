from textual.app import ComposeResult
from textual.reactive import reactive
from textual.widget import Widget
from textual.widgets import Label

TAB_LABELS = [
    ('1', 'Top'),
    ('2', 'Waypoint'),
    ('3', 'SLAM'),
    ('4', 'System'),
    ('5', 'Setting'),
]

TAB_IDS = ['top', 'waypoint', 'slam', 'system', 'setting']


class TabSidebar(Widget):
    DEFAULT_CSS = """
    TabSidebar {
        width: 16;
        border: solid $primary;
        padding: 1;
    }
    TabSidebar .tab-item { color: $text-muted; }
    TabSidebar .tab-item--active { color: $text; text-style: bold; }
    """

    active_tab: reactive[str] = reactive('top')

    def compose(self) -> ComposeResult:
        for num, label in TAB_LABELS:
            yield Label(f'[{num}] {label}', id=f'tab-{num}', classes='tab-item')

    def watch_active_tab(self, tab_id: str) -> None:
        for num, _ in TAB_LABELS:
            idx = TAB_IDS[int(num) - 1]
            widget = self.query_one(f'#tab-{num}', Label)
            if idx == tab_id:
                widget.add_class('tab-item--active')
                widget.remove_class('tab-item')
                widget.update(f'> [{num}] {TAB_LABELS[int(num) - 1][1]}')
            else:
                widget.remove_class('tab-item--active')
                widget.add_class('tab-item')
                widget.update(f'  [{num}] {TAB_LABELS[int(num) - 1][1]}')
