"""Track Spot arm joint position, velocity, and effort telemetry."""

from dataclasses import dataclass
from types import MappingProxyType
from threading import RLock
from typing import Mapping
import math
import time

from sensor_msgs.msg import JointState

from fault_detector_spot.shared.runtime_source import RuntimeSource


ARM_JOINT_STATE_TOPIC = "joint_states"
DEFAULT_ARM_JOINT_STATE_STALE_AFTER_SEC = 1.5
ARM_JOINT_NAMES = (
    "arm_sh0",
    "arm_sh1",
    "arm_el0",
    "arm_el1",
    "arm_wr0",
    "arm_wr1",
)


@dataclass(frozen=True)
class ArmJointSample:
    """One measured state for a controllable Spot arm joint."""

    position_rad: float
    velocity_rad_s: float
    effort_nm: float


@dataclass(frozen=True)
class ArmJointStateSample:
    """One coherent six-joint arm state sample."""

    received_at: float
    joints: Mapping[str, ArmJointSample]


class ArmJointStateSource(RuntimeSource):
    """Cache fresh six-axis arm joint telemetry from the Spot driver."""

    def __init__(
        self,
        node,
        stale_after_sec: float = DEFAULT_ARM_JOINT_STATE_STALE_AFTER_SEC,
        monotonic_clock=time.monotonic,
    ):
        if node is None:
            raise RuntimeError("ArmJointStateSource requires a ROS node")
        stale_after_sec = float(stale_after_sec)
        if not math.isfinite(stale_after_sec) or stale_after_sec <= 0.0:
            raise ValueError("Arm joint state stale timeout must be positive")
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")

        self.node = node
        self.stale_after_sec = stale_after_sec
        self._monotonic_clock = monotonic_clock
        self._lock = RLock()
        self._sample = None
        self._last_received_at = None
        self._subscription = node.create_subscription(
            JointState,
            ARM_JOINT_STATE_TOPIC,
            self._receive_state,
            10,
        )

    @property
    def last_received_at(self):
        with self._lock:
            return self._last_received_at

    def sample(self, now: float = None) -> ArmJointStateSample | None:
        """Return the latest complete fresh arm sample."""
        current = self._monotonic_clock() if now is None else float(now)
        with self._lock:
            sample = self._sample
        if sample is None:
            return None
        if not self._is_fresh(sample.received_at, current):
            return None
        return sample

    def is_stale(self, now: float = None) -> bool:
        current = self._monotonic_clock() if now is None else float(now)
        with self._lock:
            received_at = self._last_received_at
        return not self._is_fresh(received_at, current)

    def _receive_state(self, message) -> None:
        received_at = self._monotonic_clock()
        sample = self._read_arm_sample(message, received_at)
        with self._lock:
            self._sample = sample
            self._last_received_at = received_at

    @classmethod
    def _read_arm_sample(cls, message, received_at):
        names = tuple(str(name) for name in message.name)
        positions = tuple(message.position)
        velocities = tuple(message.velocity)
        efforts = tuple(message.effort)
        count = len(names)
        if not (
            len(positions) == count
            and len(velocities) == count
            and len(efforts) == count
        ):
            return None

        indexes = {}
        for index, name in enumerate(names):
            canonical = cls._canonical_joint_name(name)
            if canonical in ARM_JOINT_NAMES:
                if canonical in indexes:
                    return None
                indexes[canonical] = index
        if any(name not in indexes for name in ARM_JOINT_NAMES):
            return None

        joints = {}
        for name in ARM_JOINT_NAMES:
            index = indexes[name]
            values = (
                float(positions[index]),
                float(velocities[index]),
                float(efforts[index]),
            )
            if not all(math.isfinite(value) for value in values):
                return None
            joints[name] = ArmJointSample(
                position_rad=values[0],
                velocity_rad_s=values[1],
                effort_nm=values[2],
            )

        return ArmJointStateSample(
            received_at=float(received_at),
            joints=MappingProxyType(joints),
        )

    @staticmethod
    def _canonical_joint_name(name: str) -> str:
        return str(name).rsplit("/", 1)[-1]

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
    "ARM_JOINT_NAMES",
    "ARM_JOINT_STATE_TOPIC",
    "ArmJointSample",
    "ArmJointStateSample",
    "ArmJointStateSource",
    "DEFAULT_ARM_JOINT_STATE_STALE_AFTER_SEC",
]
