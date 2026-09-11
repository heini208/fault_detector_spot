"""Track Spot driver's reported standing/sitting state."""

from enum import Enum
from threading import RLock
import math
import time

from spot_msgs.msg import Feedback


POSTURE_STATE_TOPIC = "status/feedback"
DEFAULT_POSTURE_STATE_STALE_AFTER_SEC = 1.5


class PostureState(str, Enum):
    UNKNOWN = "unknown"
    SITTING = "sitting"
    STANDING = "standing"


class PostureStateSource:
    """Cache fresh posture published by the Spot driver."""

    def __init__(
        self,
        node,
        stale_after_sec: float = DEFAULT_POSTURE_STATE_STALE_AFTER_SEC,
        monotonic_clock=time.monotonic,
    ):
        if node is None:
            raise RuntimeError("PostureStateSource requires a ROS node")
        stale_after_sec = float(stale_after_sec)
        if not math.isfinite(stale_after_sec) or stale_after_sec <= 0.0:
            raise ValueError("Posture state stale timeout must be positive")
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")

        self.node = node
        self.stale_after_sec = stale_after_sec
        self._monotonic_clock = monotonic_clock
        self._lock = RLock()
        self._posture = PostureState.UNKNOWN
        self._last_received_at = None
        self._subscription = node.create_subscription(
            Feedback,
            POSTURE_STATE_TOPIC,
            self._receive_state,
            10,
        )

    @property
    def last_received_at(self):
        with self._lock:
            return self._last_received_at

    def posture(self, now: float = None):
        """Return fresh physical posture, or None when unavailable/stale."""
        current = self._monotonic_clock() if now is None else float(now)
        with self._lock:
            received_at = self._last_received_at
            state = self._posture
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
        if message.sitting and not message.standing and not message.moving:
            state = PostureState.SITTING
        elif message.standing and not message.sitting:
            state = PostureState.STANDING
        else:
            state = PostureState.UNKNOWN
        with self._lock:
            self._posture = state
            self._last_received_at = self._monotonic_clock()

    def destroy(self) -> None:
        subscription = self._subscription
        self._subscription = None
        if subscription is not None:
            self.node.destroy_subscription(subscription)


__all__ = [
    "PostureStateSource",
    "PostureState",
    "DEFAULT_POSTURE_STATE_STALE_AFTER_SEC",
    "POSTURE_STATE_TOPIC",
]
