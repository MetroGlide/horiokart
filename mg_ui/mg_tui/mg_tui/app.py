from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import ContentSwitcher, Footer, Header, Static

from .state import AppState
from .ros_backend import RosBackend, Services
from .system_client import SystemClient
from .widgets import TabSidebar
from .pages import TopPage, WaypointNavPage, SlamPage, SystemPage, SettingPage
from .pages.waypoint_nav_page import WaypointNavPage as _WaypointNavPage
from .pages.slam_page import SlamPage as _SlamPage
from .pages.system_page import SystemPage as _SystemPage
from .pages.setting_page import SettingPage as _SettingPage

_TAB_IDS = ['top', 'waypoint', 'slam', 'system', 'setting']


class MgTuiApp(App):
    AUTO_FOCUS = ""

    CSS = """
    Screen { layout: vertical; }
    #main-row { height: 1fr; layout: horizontal; }
    #content-area { width: 1fr; }
    #status-bar { height: 1; background: $surface; }
    """

    BINDINGS = [
        Binding('1', 'switch_tab("top")', 'Top', show=False, priority=True),
        Binding('2', 'switch_tab("waypoint")',
                'Waypoint', show=False, priority=True),
        Binding('3', 'switch_tab("slam")', 'SLAM', show=False, priority=True),
        Binding('4', 'switch_tab("system")',
                'System', show=False, priority=True),
        Binding('5', 'switch_tab("setting")',
                'Setting', show=False, priority=True),
        Binding('h', 'prev_tab', '[h] Prev Tab', priority=True),
        Binding('l', 'next_tab', '[l] Next Tab', priority=True),
        Binding('j', 'cursor_down', '[j] Down'),
        Binding('k', 'cursor_up', '[k] Up'),
        Binding('enter', 'execute', 'Execute'),
        Binding('s', 'page_action_s', '[s] Start/Sim', show=False),
        Binding('x', 'page_action_x', '[x] Stop', show=False),
        Binding('J', 'jump_waypoint', '[J] Jump', show=False, priority=True),
        Binding('q', 'quit', '[q] Quit', priority=True),
    ]

    def __init__(self, backend: RosBackend, sys_client: SystemClient, state: AppState):
        super().__init__()
        self._backend = backend
        self._sys_client = sys_client
        self._state = state
        self._current_tab = 'top'

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id='main-row'):
            yield TabSidebar(id='tab-sidebar')
            with ContentSwitcher(initial='top', id='content-area'):
                yield TopPage(id='top')
                yield WaypointNavPage(id='waypoint')
                yield SlamPage(id='slam')
                yield SystemPage(id='system')
                yield SettingPage(id='setting')
        yield Static(id='status-bar')
        yield Footer()

    def on_mount(self) -> None:
        self.set_interval(0.5, self._update_ui)

    def _update_ui(self) -> None:
        s = self._state
        self.query_one('#status-bar', Static).update(s.last_service_msg)

        tab = self._current_tab
        if tab == 'top':
            self.query_one('#top', TopPage).refresh_data(s)
        elif tab == 'waypoint':
            self.query_one('#waypoint', WaypointNavPage).refresh_data(s)
        elif tab == 'slam':
            self.query_one('#slam', SlamPage).refresh_data(s)
        elif tab == 'system':
            self.query_one('#system', SystemPage).refresh_data(s)
        elif tab == 'setting':
            self.query_one('#setting', SettingPage).refresh_data(s)

    def action_switch_tab(self, tab_id: str) -> None:
        self._current_tab = tab_id
        self.query_one('#content-area', ContentSwitcher).current = tab_id
        self.query_one('#tab-sidebar', TabSidebar).active_tab = tab_id

    def action_prev_tab(self) -> None:
        idx = _TAB_IDS.index(self._current_tab)
        self.action_switch_tab(_TAB_IDS[(idx - 1) % len(_TAB_IDS)])

    def action_next_tab(self) -> None:
        idx = _TAB_IDS.index(self._current_tab)
        self.action_switch_tab(_TAB_IDS[(idx + 1) % len(_TAB_IDS)])

    def action_cursor_down(self) -> None:
        tab = self._current_tab
        if tab == 'waypoint':
            self.query_one('#waypoint', WaypointNavPage).move_cursor_down()
        elif tab == 'slam':
            self.query_one('#slam', SlamPage).move_cursor_down()
        elif tab == 'system':
            self.query_one('#system', SystemPage).move_cursor_down()

    def action_cursor_up(self) -> None:
        tab = self._current_tab
        if tab == 'waypoint':
            self.query_one('#waypoint', WaypointNavPage).move_cursor_up()
        elif tab == 'slam':
            self.query_one('#slam', SlamPage).move_cursor_up()
        elif tab == 'system':
            self.query_one('#system', SystemPage).move_cursor_up()

    def action_execute(self) -> None:
        tab = self._current_tab
        if tab == 'waypoint':
            self.query_one('#waypoint', WaypointNavPage).execute_cursor()
        elif tab == 'slam':
            self.query_one('#slam', SlamPage).execute_cursor()

    def action_page_action_s(self) -> None:
        tab = self._current_tab
        if tab == 'system':
            self.query_one('#system', SystemPage).action_start_focused()
        elif tab == 'setting':
            self.query_one('#setting', SettingPage).toggle_simulation(
                self._state)

    def action_page_action_x(self) -> None:
        if self._current_tab == 'system':
            self.query_one('#system', SystemPage).action_stop_focused()

    def action_jump_waypoint(self) -> None:
        if self._current_tab == 'waypoint':
            page = self.query_one('#waypoint', WaypointNavPage)
            idx = page.get_jump_index()
            if idx is not None:
                self._backend.publish_jump(idx)
                self._state.last_service_msg = f'jump → waypoint {idx}'

    # --- WaypointNavPage action routing ---

    def on_waypoint_nav_page_action_selected(
        self, event: _WaypointNavPage.ActionSelected
    ) -> None:
        label = event.label
        if label == 'Start (3s)':
            self._backend.call_trigger(Services.WAYPOINT_START)
        elif label == 'Start Now':
            self._backend.call_trigger(Services.WAYPOINT_START)
        elif label == 'Stop':
            self._backend.call_trigger(Services.WAYPOINT_STOP)
        elif label == 'Pause':
            self._backend.publish_pause(active=True)
        elif label == 'Resume':
            self._backend.publish_pause(active=False)
        elif label == 'Reload Waypoints':
            self._backend.call_trigger(Services.WAYPOINT_RELOAD)
        elif label == 'Reset Robot Pose':
            self._sys_client.reset_pose(0.0, 0.0, 0.0, 0.0)
        elif label == 'Reset AMCL Pose':
            self._state.last_service_msg = 'Reset AMCL Pose: not available via TUI'

    # --- SlamPage action routing ---

    def on_slam_page_action_selected(self, event: _SlamPage.ActionSelected) -> None:
        label = event.label
        if label == 'Start SLAM':
            self._sys_client.start_service('slam')
        elif label == 'Stop SLAM':
            self._sys_client.stop_service('slam')
        elif label == 'Save Map':
            self._sys_client.save_map()
        elif label == 'Reset Robot Pose':
            self._sys_client.reset_pose(0.0, 0.0, 0.0, 0.0)

    # --- SystemPage service routing ---

    def on_system_page_service_start_requested(
        self, event: _SystemPage.ServiceStartRequested
    ) -> None:
        self._sys_client.start_service(event.service_key)

    def on_system_page_service_stop_requested(
        self, event: _SystemPage.ServiceStopRequested
    ) -> None:
        self._sys_client.stop_service(event.service_key)
