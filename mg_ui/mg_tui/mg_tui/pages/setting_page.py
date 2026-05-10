from textual.app import ComposeResult
from textual.message import Message
from textual.widget import Widget
from textual.widgets import Label, Static

from ..state import AppState


class SettingPage(Widget):

    class SimulationToggled(Message):
        def __init__(self, is_simulation: bool) -> None:
            super().__init__()
            self.is_simulation = is_simulation

    DEFAULT_CSS = """
    SettingPage { height: 1fr; overflow-y: auto; padding: 1; }
    SettingPage .section-title { color: $text-muted; text-style: bold; }
    SettingPage #sim-status { margin-top: 1; }
    SettingPage #sim-hint { color: $text-muted; }
    """

    def compose(self) -> ComposeResult:
        yield Label('Setting', classes='section-title')
        yield Static(id='sim-status')
        yield Label('[s] Toggle Simulation Mode', id='sim-hint')

    def toggle_simulation(self, state: AppState) -> None:
        state.is_simulation = not state.is_simulation
        self.post_message(self.SimulationToggled(state.is_simulation))
        self._render_sim_status(state.is_simulation)

    def refresh_data(self, state: AppState) -> None:
        self._render_sim_status(state.is_simulation)

    def _render_sim_status(self, is_simulation: bool) -> None:
        color = 'bold blue' if is_simulation else 'dim'
        label = 'ON' if is_simulation else 'OFF'
        self.query_one('#sim-status', Static).update(
            f'Simulation Mode: [{color}]{label}[/]'
        )
