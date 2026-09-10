"""Track Spot's authoritative manipulator stow state."""

from enum import Enum
from threading import RLock
import math
import time

from bosdyn_api_msgs.msg import ManipulatorState, ManipulatorStateStowState


MANIPULATOR_STATE_TOPIC = "manipulation_state"
DEFAULT_ARM_STATE_STALE_AFTER_SEC = 1.5


class ArmStowState(str, Enum):
    UNKNOWN = "unknown"
    STOWED = "stowed"
    DEPLOYED = "deployed"


class ArmStateSource:
    """Cache fresh stow state published by the Spot driver."""

    def __init__(
        self,
        node,
        stale_after_sec: float = DEFAULT_ARM_STATE_STALE_AFTER_SEC,
        monotonic_clock=time.monotonic,
    ):
        if node is None:
            raise RuntimeError("ArmStateSource requires a ROS node")
        stale_after_sec = float(stale_after_sec)
        if not math.isfinite(stale_after_sec) or stale_after_sec <= 0.0:
            raise ValueError("Arm state stale timeout must be positive")
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")

        self.node = node
        self.stale_after_sec = stale_after_sec
        self._monotonic_clock = monotonic_clock
        self._lock = RLock()
        self._stow_state = ArmStowState.UNKNOWN
        self._last_received_at = None
        self._subscription = node.create_subscription(
            ManipulatorState,
            MANIPULATOR_STATE_TOPIC,
            self._receive_state,
            10,
        )

    @property
    def last_received_at(self):
        with self._lock:
            return self._last_received_at

    def stow_state(self, now: float = None):
        """Return fresh physical stow state, or None when unavailable/stale."""
        current = self._monotonic_clock() if now is None else float(now)
        with self._lock:
            received_at = self._last_received_at
            state = self._stow_state
        if received_at is None:
            return None
        if current - received_at > self.stale_after_sec:
            return None
        return state

    def is_stale(self, now: float = None) -> bool:
        current = self._monotonic_clock() if now is None else float(now)
        with self._lock:
            received_at = self._last_received_at
        if received_at is None:
            return True
        return current - received_at > self.stale_after_sec

    def _receive_state(self, message) -> None:
        value = int(message.stow_state.value)
        if value == ManipulatorStateStowState.STOWSTATE_STOWED:
            state = ArmStowState.STOWED
        elif value == ManipulatorStateStowState.STOWSTATE_DEPLOYED:
            state = ArmStowState.DEPLOYED
        else:
            state = ArmStowState.UNKNOWN
        with self._lock:
            self._stow_state = state
            self._last_received_at = self._monotonic_clock()

    def destroy(self) -> None:
        subscription = self._subscription
        self._subscription = None
        if subscription is not None:
            self.node.destroy_subscription(subscription)


__all__ = [
    "ArmStateSource",
    "ArmStowState",
    "DEFAULT_ARM_STATE_STALE_AFTER_SEC",
    "MANIPULATOR_STATE_TOPIC",
]
