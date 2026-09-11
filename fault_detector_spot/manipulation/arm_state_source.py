"""Track Spot's authoritative manipulator state."""

from dataclasses import dataclass
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


@dataclass(frozen=True)
class HandVelocitySample:
    """One measured hand velocity sample in Spot's vision frame."""

    received_at: float
    linear_x_mps: float
    linear_y_mps: float
    linear_z_mps: float
    angular_x_rad_s: float
    angular_y_rad_s: float
    angular_z_rad_s: float

    @property
    def linear_speed_mps(self) -> float:
        return math.sqrt(
            self.linear_x_mps * self.linear_x_mps
            + self.linear_y_mps * self.linear_y_mps
            + self.linear_z_mps * self.linear_z_mps
        )

    @property
    def angular_speed_rad_s(self) -> float:
        return math.sqrt(
            self.angular_x_rad_s * self.angular_x_rad_s
            + self.angular_y_rad_s * self.angular_y_rad_s
            + self.angular_z_rad_s * self.angular_z_rad_s
        )


class ArmStateSource:
    """Cache fresh manipulator state published by the Spot driver."""

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
        self._hand_velocity_sample = None
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
        if not self._is_fresh(received_at, current):
            return None
        return state

    def hand_velocity_sample(
        self,
        now: float = None,
    ) -> HandVelocitySample | None:
        """Return the latest fresh vision-frame hand velocity sample."""
        current = self._monotonic_clock() if now is None else float(now)
        with self._lock:
            received_at = self._last_received_at
            sample = self._hand_velocity_sample
        if not self._is_fresh(received_at, current):
            return None
        return sample

    def is_stale(self, now: float = None) -> bool:
        current = self._monotonic_clock() if now is None else float(now)
        with self._lock:
            received_at = self._last_received_at
        return not self._is_fresh(received_at, current)

    def _receive_state(self, message) -> None:
        received_at = self._monotonic_clock()
        value = int(message.stow_state.value)
        if value == ManipulatorStateStowState.STOWSTATE_STOWED:
            state = ArmStowState.STOWED
        elif value == ManipulatorStateStowState.STOWSTATE_DEPLOYED:
            state = ArmStowState.DEPLOYED
        else:
            state = ArmStowState.UNKNOWN

        velocity_sample = self._read_hand_velocity(
            message,
            received_at,
        )
        with self._lock:
            self._stow_state = state
            self._hand_velocity_sample = velocity_sample
            self._last_received_at = received_at

    @staticmethod
    def _read_hand_velocity(message, received_at):
        field_mask = int(message.has_field)
        velocity_mask = int(
            ManipulatorState.VELOCITY_OF_HAND_IN_VISION_FIELD_SET
        )
        if not field_mask & velocity_mask:
            return None

        velocity = message.velocity_of_hand_in_vision
        values = (
            float(velocity.linear.x),
            float(velocity.linear.y),
            float(velocity.linear.z),
            float(velocity.angular.x),
            float(velocity.angular.y),
            float(velocity.angular.z),
        )
        if not all(math.isfinite(value) for value in values):
            return None

        return HandVelocitySample(
            received_at=float(received_at),
            linear_x_mps=values[0],
            linear_y_mps=values[1],
            linear_z_mps=values[2],
            angular_x_rad_s=values[3],
            angular_y_rad_s=values[4],
            angular_z_rad_s=values[5],
        )

    def _is_fresh(self, received_at, current: float) -> bool:
        if received_at is None:
            return False
        return current - received_at <= self.stale_after_sec

    def destroy(self) -> None:
        subscription = self._subscription
        self._subscription = None
        if subscription is not None:
            self.node.destroy_subscription(subscription)


__all__ = [
    "ArmStateSource",
    "ArmStowState",
    "DEFAULT_ARM_STATE_STALE_AFTER_SEC",
    "HandVelocitySample",
    "MANIPULATOR_STATE_TOPIC",
]
