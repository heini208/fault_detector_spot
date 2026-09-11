"""Typed outcomes shared by arm movement execution layers."""

from dataclasses import dataclass
from enum import Enum


class ArmMovementOutcome(Enum):
    """Typed outcome of one arm executor lifecycle update."""

    RUNNING = "running"
    SUCCESS = "success"
    BUSY = "busy"
    ACTION_SERVER_UNAVAILABLE = "action_server_unavailable"
    GOAL_RESPONSE_TIMEOUT = "goal_response_timeout"
    GOAL_REJECTED = "goal_rejected"
    RESULT_TIMEOUT = "result_timeout"
    MOTION_FAILED = "motion_failed"
    ARM_STATE_UNAVAILABLE = "arm_state_unavailable"
    ARM_STATE_STALE = "arm_state_stale"
    ARM_STATE_UNKNOWN = "arm_state_unknown"
    FORCE_UNAVAILABLE = "force_unavailable"
    FORCE_STALE = "force_stale"
    FORCE_UNSTABLE = "force_unstable"
    SETTLING_TIMEOUT = "settling_timeout"
    SETTLING_UNAVAILABLE = "settling_unavailable"
    STOP_UNCONFIRMED = "stop_unconfirmed"
    RETREAT_FAILED = "retreat_failed"
    CONTACT = "contact"
    EXECUTION_ERROR = "execution_error"


@dataclass(frozen=True)
class ArmMovementUpdate:
    """Current nonblocking arm execution outcome and diagnostic detail."""

    outcome: ArmMovementOutcome
    detail: str


__all__ = [
    "ArmMovementOutcome",
    "ArmMovementUpdate",
]
