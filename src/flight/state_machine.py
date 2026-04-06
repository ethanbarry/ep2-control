from __future__ import annotations

from dataclasses import dataclass, field

from src.domain.enums import MissionState

# this is the sate of the drone for the mission. defines the allowed transitions and keeps a history.

ALLOWED_TRANSITIONS: dict[MissionState, set[MissionState]] = {
    MissionState.IDLE: {MissionState.WAIT_FOR_GUIDED},
    MissionState.WAIT_FOR_GUIDED: {MissionState.ACQUIRE_HOME, MissionState.ABORT_TO_STABILIZE},
    MissionState.ACQUIRE_HOME: {MissionState.SPIN_SCAN_1, MissionState.RETURN_TO_HOME_REPLAY},
    MissionState.SPIN_SCAN_1: {
        MissionState.COMPUTE_PERPENDICULAR_OFFSET,
        MissionState.RETURN_TO_HOME_REPLAY,
        MissionState.SAFE_HOLD,
    },
    MissionState.COMPUTE_PERPENDICULAR_OFFSET: {
        MissionState.TRANSIT_TO_OFFSET,
        MissionState.RETURN_TO_HOME_REPLAY,
    },
    MissionState.TRANSIT_TO_OFFSET: {
        MissionState.SPIN_SCAN_2,
        MissionState.RETURN_TO_HOME_REPLAY,
        MissionState.SAFE_HOLD,
    },
    MissionState.SPIN_SCAN_2: {
        MissionState.TRIANGULATE,
        MissionState.RETURN_TO_HOME_REPLAY,
        MissionState.SAFE_HOLD,
    },
    MissionState.TRIANGULATE: {
        MissionState.TRANSIT_TO_TARGET,
        MissionState.RETURN_TO_HOME_REPLAY,
        MissionState.SAFE_HOLD,
    },
    MissionState.TRANSIT_TO_TARGET: {
        MissionState.VERIFY_TARGET,
        MissionState.RETURN_TO_HOME_REPLAY,
        MissionState.SAFE_HOLD,
    },
    MissionState.VERIFY_TARGET: {
        MissionState.COMPLETE,
        MissionState.RETURN_TO_HOME_REPLAY,
        MissionState.SAFE_HOLD,
    },
    MissionState.RETURN_TO_HOME_REPLAY: {MissionState.COMPLETE, MissionState.SAFE_HOLD},
    MissionState.SAFE_HOLD: {MissionState.ABORT_TO_STABILIZE, MissionState.COMPLETE},
    MissionState.ABORT_TO_STABILIZE: {MissionState.COMPLETE},
    MissionState.COMPLETE: set(),
}


@dataclass
class MissionStateMachine:
    current_state: MissionState = MissionState.IDLE
    history: list[MissionState] = field(default_factory=lambda: [MissionState.IDLE])

    def transition_to(self, next_state: MissionState) -> None:
        allowed = ALLOWED_TRANSITIONS[self.current_state]
        assert next_state in allowed, f"invalid transition {self.current_state.value} -> {next_state.value}"
        self.current_state = next_state
        self.history.append(next_state)
