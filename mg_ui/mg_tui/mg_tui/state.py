import time
from dataclasses import dataclass, field

DIAG_LEVEL = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}

STATE_STYLE = {
    'IDLE': 'dim',
    'NAVIGATING': 'bold cyan',
    'SUSPENDED': 'bold yellow',
    'ERROR': 'bold red',
    'GOAL_REACHED': 'bold green',
    'ON_STARTING': 'yellow',
    'ON_ARRIVING': 'cyan',
}

GOAL_STATUS_LABEL = {
    1: 'ACCEPTED',
    2: 'EXECUTING',
    4: 'SUCCEEDED',
    5: 'CANCELED',
    6: 'ABORTED',
}

_STALE_TIMEOUT = 5.0


class DiagnosticsCache:
    def __init__(self) -> None:
        self._entries: dict[str, tuple[int, str, str, float]] = {}

    def update(self, statuses) -> None:
        now = time.monotonic()
        for s in statuses:
            level = s.level if isinstance(s.level, int) else s.level[0]
            self._entries[s.name] = (level, s.name, s.message, now)

    def get_all(self) -> list[tuple[int, str, str]]:
        now = time.monotonic()
        result = []
        for level, name, message, ts in self._entries.values():
            if now - ts > _STALE_TIMEOUT:
                result.append((3, name, 'stale'))
            else:
                result.append((level, name, message))
        result.sort(key=lambda x: (-x[0], x[1]))
        return result


@dataclass
class AppState:
    seq_state: str = '—'
    seq_index: int = 0
    seq_total: int = 0
    seq_distance: float = 0.0
    seq_countdown_ms: int = 0
    is_paused: bool = False
    pause_requesters: list[str] = field(default_factory=list)
    diag_cache: DiagnosticsCache = field(default_factory=DiagnosticsCache)
    containers: dict[str, str] = field(default_factory=dict)
    amcl_cov_xy: float = 0.0
    emergency_stop: bool = False
    collision_polygons_active: list[str] = field(default_factory=list)
    nav2_action_status: int | None = None
    is_simulation: bool = False
    last_service_msg: str = ''
