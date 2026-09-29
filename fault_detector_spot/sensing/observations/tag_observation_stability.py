"""Evaluate stability of timestamped tag observations."""

from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math

from fault_detector_msgs.msg import TagElement

from fault_detector_spot.inspection.geometry.rotation import (
    quaternion_to_rpy,
)
from fault_detector_spot.inspection.model.models import QuaternionData


@dataclass(frozen=True)
class TagObservationStabilityConfig:
    """Explicit acceptance limits for one stable observation window."""

    required_samples: int
    maximum_position_span_m: float
    maximum_yaw_span_rad: float
    maximum_sample_span_sec: float

    def __post_init__(self):
        if (
            isinstance(self.required_samples, bool)
            or not isinstance(self.required_samples, int)
            or self.required_samples < 2
        ):
            raise ValueError(
                "Tag stability required_samples must be an integer >= 2"
            )
        for name in (
            "maximum_position_span_m",
            "maximum_yaw_span_rad",
            "maximum_sample_span_sec",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"Tag stability {name} must be positive and finite"
                )


class StableTagObservationTracker:
    """Accept a tag only after several consistent camera observations."""

    def __init__(self, config: TagObservationStabilityConfig):
        if not isinstance(config, TagObservationStabilityConfig):
            raise TypeError(
                "StableTagObservationTracker requires "
                "TagObservationStabilityConfig"
            )
        self.config = config
        self._samples = deque(maxlen=config.required_samples)
        self._boundary_stamp_sec = None
        self._frame_id = None
        self._tag_id = None
        self._last_stamp_sec = None

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def reset(self) -> None:
        self._boundary_stamp_sec = None
        self._reset_samples()

    def _reset_samples(self) -> None:
        self._samples.clear()
        self._frame_id = None
        self._tag_id = None
        self._last_stamp_sec = None

    def update(
        self,
        observation: TagElement | None,
        after_stamp_sec: float,
    ) -> TagElement | None:
        boundary = float(after_stamp_sec)
        if not math.isfinite(boundary):
            raise ValueError(
                "Tag stability observation boundary must be finite"
            )
        if (
            self._boundary_stamp_sec is None
            or boundary != self._boundary_stamp_sec
        ):
            self._reset_samples()
            self._boundary_stamp_sec = boundary

        if observation is None:
            return None
        if not isinstance(observation, TagElement):
            raise TypeError(
                "Tag stability requires a TagElement observation"
            )

        stamp_sec = self._stamp_sec(observation)
        if stamp_sec is None or stamp_sec <= boundary:
            return None

        frame_id = observation.pose.header.frame_id.strip()
        if not frame_id:
            self._reset_samples()
            return None

        tag_id = int(observation.id)
        if self._tag_id is not None and tag_id != self._tag_id:
            self._reset_samples()

        if (
            self._last_stamp_sec is not None
            and stamp_sec < self._last_stamp_sec
        ):
            self._reset_samples()

        if stamp_sec == self._last_stamp_sec:
            return self._stable_observation()

        if self._frame_id is not None and frame_id != self._frame_id:
            self._reset_samples()

        try:
            sample = self._sample(observation, stamp_sec)
        except (TypeError, ValueError):
            self._reset_samples()
            return None

        self._frame_id = frame_id
        self._tag_id = tag_id
        self._last_stamp_sec = stamp_sec
        self._samples.append(sample)
        self._discard_expired_window(stamp_sec)

        if len(self._samples) < self.config.required_samples:
            return None
        if not self._window_is_stable():
            latest = self._samples[-1]
            self._samples.clear()
            self._samples.append(latest)
            return None
        return deepcopy(self._samples[-1][4])

    def _stable_observation(self):
        if (
            len(self._samples) == self.config.required_samples
            and self._window_is_stable()
        ):
            return deepcopy(self._samples[-1][4])
        return None

    def _discard_expired_window(self, latest_stamp_sec: float) -> None:
        maximum_span = self.config.maximum_sample_span_sec
        while (
            self._samples
            and latest_stamp_sec - self._samples[0][0] > maximum_span
        ):
            self._samples.popleft()

    def _window_is_stable(self) -> bool:
        samples = tuple(self._samples)
        for index, first in enumerate(samples):
            for second in samples[index + 1:]:
                position_error = math.hypot(
                    first[1] - second[1],
                    first[2] - second[2],
                )
                yaw_error = abs(
                    math.atan2(
                        math.sin(first[3] - second[3]),
                        math.cos(first[3] - second[3]),
                    )
                )
                if (
                    position_error
                    > self.config.maximum_position_span_m
                    or yaw_error
                    > self.config.maximum_yaw_span_rad
                ):
                    return False
        return True

    @classmethod
    def _sample(cls, observation: TagElement, stamp_sec: float):
        pose = observation.pose.pose
        orientation = pose.orientation
        _, _, yaw = quaternion_to_rpy(
            QuaternionData(
                x=float(orientation.x),
                y=float(orientation.y),
                z=float(orientation.z),
                w=float(orientation.w),
            )
        )
        values = (
            float(pose.position.x),
            float(pose.position.y),
            float(yaw),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                "Tag observation contains non-finite planar pose values"
            )
        return (
            stamp_sec,
            values[0],
            values[1],
            values[2],
            deepcopy(observation),
        )

    @staticmethod
    def _stamp_sec(observation: TagElement):
        stamp = observation.pose.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if math.isfinite(value) else None


__all__ = [
    "StableTagObservationTracker",
    "TagObservationStabilityConfig",
]
