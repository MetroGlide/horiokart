from textual.app import ComposeResult
from textual.message import Message
from textual.reactive import reactive
from textual.widget import Widget
from textual.widgets import Label


class CursorList(Widget):
    """j/k カーソル移動対応のシンプルなリストウィジェット。"""

    DEFAULT_CSS = """
    CursorList { height: auto; }
    CursorList .cursor-item { color: $text-muted; }
    CursorList .cursor-item--selected { color: $text; text-style: bold reverse; }
    """

    class Selected(Message):
        def __init__(self, index: int, label: str) -> None:
            super().__init__()
            self.index = index
            self.label = label

    cursor: reactive[int] = reactive(0)

    def __init__(self, items: list[str], **kwargs) -> None:
        super().__init__(**kwargs)
        self._items = items

    def compose(self) -> ComposeResult:
        for i, item in enumerate(self._items):
            classes = 'cursor-item--selected' if i == 0 else 'cursor-item'
            yield Label(item, id=f'cli-{i}', classes=classes)

    def update_items(self, items: list[str]) -> None:
        if items == self._items:
            return
        self._items = items
        self.cursor = min(self.cursor, max(0, len(items) - 1))
        self.remove_children()
        for i, item in enumerate(items):
            classes = 'cursor-item--selected' if i == self.cursor else 'cursor-item'
            self.mount(Label(item, id=f'cli-{i}', classes=classes))

    def watch_cursor(self, cursor: int) -> None:
        for i, _ in enumerate(self._items):
            try:
                widget = self.query_one(f'#cli-{i}', Label)
                if i == cursor:
                    widget.remove_class('cursor-item')
                    widget.add_class('cursor-item--selected')
                else:
                    widget.remove_class('cursor-item--selected')
                    widget.add_class('cursor-item')
            except Exception:
                pass

    def move_down(self) -> None:
        if self._items:
            self.cursor = (self.cursor + 1) % len(self._items)

    def move_up(self) -> None:
        if self._items:
            self.cursor = (self.cursor - 1) % len(self._items)

    def select(self) -> None:
        if self._items:
            self.post_message(self.Selected(
                self.cursor, self._items[self.cursor]))
