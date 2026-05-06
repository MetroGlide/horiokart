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


@dataclass
class AppState:
    seq_state: str = '—'
    seq_index: int = 0
    seq_total: int = 0
    seq_distance: float = 0.0
    seq_countdown_ms: int = 0
    is_paused: bool = False
    pause_requesters: list[str] = field(default_factory=list)
    diag_items: list[tuple[int, str, str]] = field(default_factory=list)
    containers: dict[str, str] = field(default_factory=dict)
    amcl_cov_xy: float = 0.0
    last_service_msg: str = ''
