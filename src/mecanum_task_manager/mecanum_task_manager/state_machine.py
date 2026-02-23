# Copyright (c) 2025 Tarik Kahraman
# SPDX-License-Identifier: MIT
"""
State machine definitions for Mecanum AMR task execution.
"""

from enum import Enum, auto


class RobotState(Enum):
    IDLE              = auto()   # Idle, no command
    READING_QR        = auto()   # Scanning for QR
    NAVIGATING_TO_LOAD = auto()  # Going to load position
    ALIGNING          = auto()   # Fine alignment under load
    LIFTING           = auto()   # Lift up
    NAVIGATING_TO_DROP = auto()  # Going to drop position
    LOWERING          = auto()   # Lift down
    RETURNING_HOME    = auto()   # Return to home
    ERROR             = auto()   # Error state
    RECOVERY          = auto()   # Recovery maneuver


# Valid state transitions
VALID_TRANSITIONS: dict[RobotState, list[RobotState]] = {
    RobotState.IDLE:               [RobotState.READING_QR],
    RobotState.READING_QR:         [RobotState.NAVIGATING_TO_LOAD, RobotState.IDLE, RobotState.ERROR],
    RobotState.NAVIGATING_TO_LOAD: [RobotState.ALIGNING, RobotState.ERROR, RobotState.RECOVERY],
    RobotState.ALIGNING:           [RobotState.LIFTING, RobotState.ERROR, RobotState.RECOVERY],
    RobotState.LIFTING:            [RobotState.NAVIGATING_TO_DROP, RobotState.ERROR],
    RobotState.NAVIGATING_TO_DROP: [RobotState.LOWERING, RobotState.ERROR, RobotState.RECOVERY],
    RobotState.LOWERING:           [RobotState.RETURNING_HOME, RobotState.ERROR],
    RobotState.RETURNING_HOME:     [RobotState.IDLE, RobotState.ERROR, RobotState.RECOVERY],
    RobotState.ERROR:              [RobotState.RECOVERY, RobotState.IDLE],
    RobotState.RECOVERY:           [RobotState.IDLE, RobotState.ERROR],
}


class StateMachine:
    """Simple state machine manager."""

    def __init__(self, initial_state: RobotState = RobotState.IDLE):
        self.state    = initial_state
        self.history: list[RobotState] = [initial_state]

    def transition(self, new_state: RobotState) -> bool:
        """
        Check and apply transition to new_state. Returns False if invalid.
        """
        allowed = VALID_TRANSITIONS.get(self.state, [])
        if new_state not in allowed:
            return False
        self.state = new_state
        self.history.append(new_state)
        return True

    def can_transition(self, new_state: RobotState) -> bool:
        return new_state in VALID_TRANSITIONS.get(self.state, [])

    def __repr__(self) -> str:
        return f'StateMachine(state={self.state.name})'

