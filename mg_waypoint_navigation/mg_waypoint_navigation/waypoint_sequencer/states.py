import enum
from typing import Dict, FrozenSet, NamedTuple


class CommandResult(NamedTuple):
    success: bool
    message: str


class SequencerState(enum.Enum):
    # Primary states (stable)
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    GOAL_REACHED = "GOAL_REACHED"
    SUSPENDED = "SUSPENDED"
    ERROR = "ERROR"

    # Transitioning states (processing in progress)
    ON_STARTING = "ON_STARTING"
    ON_ARRIVING = "ON_ARRIVING"


ALLOWED_TRANSITIONS: Dict[SequencerState, FrozenSet[SequencerState]] = {
    SequencerState.IDLE: frozenset({
        SequencerState.ON_STARTING,
    }),
    SequencerState.ON_STARTING: frozenset({
        SequencerState.NAVIGATING,
        SequencerState.SUSPENDED,
        SequencerState.IDLE,
    }),
    SequencerState.NAVIGATING: frozenset({
        SequencerState.NAVIGATING,
        SequencerState.ON_ARRIVING,
        SequencerState.GOAL_REACHED,
        SequencerState.IDLE,
        SequencerState.ERROR,
        SequencerState.SUSPENDED,
    }),
    SequencerState.ON_ARRIVING: frozenset({
        SequencerState.NAVIGATING,
        SequencerState.GOAL_REACHED,
        SequencerState.IDLE,
        SequencerState.SUSPENDED,
    }),
    SequencerState.GOAL_REACHED: frozenset({
        SequencerState.ON_STARTING,
        SequencerState.IDLE,
    }),
    SequencerState.SUSPENDED: frozenset({
        SequencerState.ON_STARTING,
        SequencerState.NAVIGATING,
        SequencerState.IDLE,
    }),
    SequencerState.ERROR: frozenset({
        SequencerState.IDLE,
    }),
}
