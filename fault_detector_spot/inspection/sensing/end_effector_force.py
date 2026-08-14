"""End-effector force baseline and probe-axis projection helpers."""

import math
import statistics
from dataclasses import dataclass

from fault_detector_spot.inspection.model.models import Vector3Data


DEFAULT_MAXIMUM_BASELINE_COMPONENT_SPAN_N = 1.0


@dataclass(frozen=True)
class EndEffectorForceSample:
    """One timestamped hand-frame force estimate from Spot."""

    force_hand: Vector3Data
    stamp_seconds: float
    receipt_time: float
    frame_id: str

    def validate(self) -> None:
        self.force_hand.validate()
        if not math.isfinite(self.stamp_seconds) or self.stamp_seconds <= 0.0:
            raise ValueError("End-effector force timestamp must be positive")
        if not math.isfinite(self.receipt_time) or self.receipt_time < 0.0:
            raise ValueError("End-effector force receipt time must be valid")
        if not self.frame_id.strip():
            raise ValueError("End-effector force frame must not be empty")


@dataclass(frozen=True)
class ForceBaseline:
    """Robust force baseline valid for one stationary arm pose."""

    force_hand: Vector3Data
    frame_id: str
    sample_count: int
    sample_span_sec: float
    maximum_component_span_n: float

    def validate(self) -> None:
        self.force_hand.validate()
        if not self.frame_id.strip():
            raise ValueError("Force baseline frame must not be empty")
        if self.sample_count < 3:
            raise ValueError("Force baseline requires at least three samples")
        if not math.isfinite(self.sample_span_sec) or self.sample_span_sec <= 0.0:
            raise ValueError("Force baseline sample span must be positive")
        if (
            not math.isfinite(self.maximum_component_span_n)
            or self.maximum_component_span_n < 0.0
        ):
            raise ValueError("Force baseline component span must be valid")


@dataclass(frozen=True)
class ProbeForceDelta:
    """Baseline-subtracted force resolved along the probe axis."""

    delta_force_hand: Vector3Data
    axial_force_n: float
    lateral_force_n: float
    total_force_n: float

    def validate(self) -> None:
        self.delta_force_hand.validate()
        values = (
            self.axial_force_n,
            self.lateral_force_n,
            self.total_force_n,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Probe force delta contains a non-finite value")
        if self.lateral_force_n < 0.0 or self.total_force_n < 0.0:
            raise ValueError("Probe force magnitudes must not be negative")


def estimate_force_baseline(
    samples,
    minimum_samples: int = 10,
    minimum_span_sec: float = 0.50,
    maximum_allowed_component_span_n: float = (
        DEFAULT_MAXIMUM_BASELINE_COMPONENT_SPAN_N
    ),
) -> ForceBaseline:
    """Estimate a stationary force baseline from one fresh sample window."""
    if (
        isinstance(minimum_samples, bool)
        or not isinstance(minimum_samples, int)
        or minimum_samples < 3
    ):
        raise ValueError("Force baseline minimum samples must be at least three")
    if not math.isfinite(minimum_span_sec) or minimum_span_sec <= 0.0:
        raise ValueError("Force baseline minimum span must be positive")
    if (
        not math.isfinite(maximum_allowed_component_span_n)
        or maximum_allowed_component_span_n <= 0.0
    ):
        raise ValueError(
            "Force baseline maximum component span must be positive"
        )

    validated = []
    for sample in samples:
        if not isinstance(sample, EndEffectorForceSample):
            raise TypeError("Force baseline requires force samples")
        sample.validate()
        validated.append(sample)
    validated.sort(key=lambda sample: sample.receipt_time)

    if len(validated) < minimum_samples:
        raise ValueError(
            f"Force baseline requires at least {minimum_samples} samples"
        )
    frame_ids = {sample.frame_id for sample in validated}
    if len(frame_ids) != 1:
        raise ValueError("Force baseline samples do not share one frame")
    sample_span_sec = (
        validated[-1].receipt_time - validated[0].receipt_time
    )
    if sample_span_sec + 1e-9 < minimum_span_sec:
        raise ValueError("Force baseline samples do not span enough time")

    components = tuple(
        [getattr(sample.force_hand, axis) for sample in validated]
        for axis in ("x", "y", "z")
    )
    force_hand = Vector3Data(
        x=float(statistics.median(components[0])),
        y=float(statistics.median(components[1])),
        z=float(statistics.median(components[2])),
    )
    maximum_component_span_n = max(
        max(values) - min(values)
        for values in components
    )
    if (
        maximum_component_span_n
        > maximum_allowed_component_span_n + 1e-9
    ):
        raise ValueError(
            "Force baseline is not stationary enough: component span "
            f"{maximum_component_span_n:.3f} N exceeds "
            f"{maximum_allowed_component_span_n:.3f} N"
        )
    baseline = ForceBaseline(
        force_hand=force_hand,
        frame_id=validated[-1].frame_id,
        sample_count=len(validated),
        sample_span_sec=sample_span_sec,
        maximum_component_span_n=float(maximum_component_span_n),
    )
    baseline.validate()
    return baseline


def project_probe_force_delta(
    sample: EndEffectorForceSample,
    baseline: ForceBaseline,
    probe_axis_hand: Vector3Data,
) -> ProbeForceDelta:
    """Project one baseline-subtracted hand force onto the probe axis."""
    sample.validate()
    baseline.validate()
    probe_axis_hand.validate()
    if sample.frame_id != baseline.frame_id:
        raise ValueError("Force sample and baseline frames differ")

    axis_norm = math.sqrt(
        probe_axis_hand.x * probe_axis_hand.x
        + probe_axis_hand.y * probe_axis_hand.y
        + probe_axis_hand.z * probe_axis_hand.z
    )
    if axis_norm <= 1e-12:
        raise ValueError("Probe axis must have non-zero length")
    axis = Vector3Data(
        x=probe_axis_hand.x / axis_norm,
        y=probe_axis_hand.y / axis_norm,
        z=probe_axis_hand.z / axis_norm,
    )
    delta = Vector3Data(
        x=sample.force_hand.x - baseline.force_hand.x,
        y=sample.force_hand.y - baseline.force_hand.y,
        z=sample.force_hand.z - baseline.force_hand.z,
    )
    axial_force_n = (
        delta.x * axis.x
        + delta.y * axis.y
        + delta.z * axis.z
    )
    total_force_n = math.sqrt(
        delta.x * delta.x
        + delta.y * delta.y
        + delta.z * delta.z
    )
    lateral_squared = max(
        0.0,
        total_force_n * total_force_n - axial_force_n * axial_force_n,
    )
    result = ProbeForceDelta(
        delta_force_hand=delta,
        axial_force_n=axial_force_n,
        lateral_force_n=math.sqrt(lateral_squared),
        total_force_n=total_force_n,
    )
    result.validate()
    return result


__all__ = [
    "DEFAULT_MAXIMUM_BASELINE_COMPONENT_SPAN_N",
    "EndEffectorForceSample",
    "ForceBaseline",
    "ProbeForceDelta",
    "estimate_force_baseline",
    "project_probe_force_delta",
]
