"""Collect a fresh stationary end-effector force baseline."""

from dataclasses import dataclass
from enum import Enum
import math
import statistics
import time


FORCE_BASELINE_MINIMUM_SAMPLES_PARAMETER = (
    "arm.force_baseline.minimum_samples"
)
FORCE_BASELINE_MINIMUM_SPAN_PARAMETER = (
    "arm.force_baseline.minimum_span_sec"
)
FORCE_BASELINE_MAXIMUM_COMPONENT_SPAN_PARAMETER = (
    "arm.force_baseline.maximum_component_span_n"
)
FORCE_BASELINE_TIMEOUT_PARAMETER = "arm.force_baseline.timeout_sec"

DEFAULT_FORCE_BASELINE_MINIMUM_SAMPLES = 10
DEFAULT_FORCE_BASELINE_MINIMUM_SPAN_SEC = 0.50
DEFAULT_FORCE_BASELINE_MAXIMUM_COMPONENT_SPAN_N = 3.0
DEFAULT_FORCE_BASELINE_TIMEOUT_SEC = 2.0


class ForceBaselineOutcome(str, Enum):
    RUNNING = "running"
    READY = "ready"
    FORCE_UNAVAILABLE = "force_unavailable"
    FORCE_STALE = "force_stale"
    FORCE_UNSTABLE = "force_unstable"
    TIMEOUT = "timeout"


@dataclass(frozen=True)
class HandForceBaseline:
    """Robust hand-frame baseline valid for one stationary arm pose."""

    x_n: float
    y_n: float
    z_n: float
    sample_count: int
    sample_span_sec: float
    maximum_component_span_n: float


@dataclass(frozen=True)
class ForceBaselineUpdate:
    outcome: ForceBaselineOutcome
    detail: str
    baseline: HandForceBaseline | None = None


class ForceBaselineSampler:
    """Build a baseline only from new fresh force samples."""

    def __init__(
        self,
        arm_state_source,
        minimum_samples: int = DEFAULT_FORCE_BASELINE_MINIMUM_SAMPLES,
        minimum_span_sec: float = DEFAULT_FORCE_BASELINE_MINIMUM_SPAN_SEC,
        maximum_component_span_n: float = (
            DEFAULT_FORCE_BASELINE_MAXIMUM_COMPONENT_SPAN_N
        ),
        timeout_sec: float = DEFAULT_FORCE_BASELINE_TIMEOUT_SEC,
        monotonic_clock=time.monotonic,
    ):
        if arm_state_source is None:
            raise RuntimeError(
                "ForceBaselineSampler requires an arm state source"
            )
        if isinstance(minimum_samples, bool) or int(minimum_samples) < 3:
            raise ValueError(
                "Force baseline minimum samples must be at least three"
            )
        if not callable(monotonic_clock):
            raise TypeError("Monotonic clock must be callable")

        self.arm_state_source = arm_state_source
        self.minimum_samples = int(minimum_samples)
        self.minimum_span_sec = self._positive_finite(
            minimum_span_sec,
            "Force baseline minimum span",
        )
        self.maximum_component_span_n = self._positive_finite(
            maximum_component_span_n,
            "Force baseline maximum component span",
        )
        self.timeout_sec = self._positive_finite(
            timeout_sec,
            "Force baseline timeout",
        )
        if self.timeout_sec < self.minimum_span_sec:
            raise ValueError(
                "Force baseline timeout must be at least the minimum span"
            )

        self._monotonic_clock = monotonic_clock
        self.reset()

    @classmethod
    def from_node(
        cls,
        node,
        arm_state_source,
        monotonic_clock=time.monotonic,
    ):
        if node is None:
            raise RuntimeError(
                "ForceBaselineSampler requires a ROS node"
            )

        parameters = (
            (
                FORCE_BASELINE_MINIMUM_SAMPLES_PARAMETER,
                DEFAULT_FORCE_BASELINE_MINIMUM_SAMPLES,
            ),
            (
                FORCE_BASELINE_MINIMUM_SPAN_PARAMETER,
                DEFAULT_FORCE_BASELINE_MINIMUM_SPAN_SEC,
            ),
            (
                FORCE_BASELINE_MAXIMUM_COMPONENT_SPAN_PARAMETER,
                DEFAULT_FORCE_BASELINE_MAXIMUM_COMPONENT_SPAN_N,
            ),
            (
                FORCE_BASELINE_TIMEOUT_PARAMETER,
                DEFAULT_FORCE_BASELINE_TIMEOUT_SEC,
            ),
        )
        values = {}
        for name, default in parameters:
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            values[name] = node.get_parameter(name).value

        return cls(
            arm_state_source=arm_state_source,
            minimum_samples=int(
                values[FORCE_BASELINE_MINIMUM_SAMPLES_PARAMETER]
            ),
            minimum_span_sec=float(
                values[FORCE_BASELINE_MINIMUM_SPAN_PARAMETER]
            ),
            maximum_component_span_n=float(
                values[
                    FORCE_BASELINE_MAXIMUM_COMPONENT_SPAN_PARAMETER
                ]
            ),
            timeout_sec=float(
                values[FORCE_BASELINE_TIMEOUT_PARAMETER]
            ),
            monotonic_clock=monotonic_clock,
        )

    @property
    def active(self) -> bool:
        return self._active

    def start(self) -> ForceBaselineUpdate:
        """Start one bounded baseline acquisition."""
        self.reset()
        self._active = True
        self._started_at = self._monotonic_clock()
        return self.poll()

    def poll(self) -> ForceBaselineUpdate:
        """Advance baseline acquisition without blocking."""
        if not self._active:
            raise RuntimeError(
                "No force baseline acquisition is active"
            )

        now = self._monotonic_clock()
        sample = self.arm_state_source.hand_force_sample()

        if (
            sample is not None
            and sample.received_at != self._last_received_at
        ):
            self._last_received_at = sample.received_at
            self._samples.append(sample)

            baseline = self._candidate_baseline()
            if baseline is not None:
                self._active = False
                return ForceBaselineUpdate(
                    ForceBaselineOutcome.READY,
                    "Stationary force baseline established from "
                    f"{baseline.sample_count} samples over "
                    f"{baseline.sample_span_sec:.2f} s",
                    baseline,
                )

        if now - self._started_at < self.timeout_sec:
            return ForceBaselineUpdate(
                ForceBaselineOutcome.RUNNING,
                "Collecting stationary force baseline "
                f"({len(self._samples)}/{self.minimum_samples} samples)",
            )

        self._active = False
        if self.arm_state_source.last_received_at is None:
            outcome = ForceBaselineOutcome.FORCE_UNAVAILABLE
            detail = "Manipulator state was unavailable during force baseline"
        elif self.arm_state_source.is_stale():
            outcome = ForceBaselineOutcome.FORCE_STALE
            detail = "Manipulator state became stale during force baseline"
        elif not self._samples:
            outcome = ForceBaselineOutcome.FORCE_UNAVAILABLE
            detail = (
                "Manipulator state did not contain end-effector force "
                "during force baseline"
            )
        elif self._last_unstable_span is not None:
            outcome = ForceBaselineOutcome.FORCE_UNSTABLE
            detail = (
                "End-effector force did not remain stationary enough: "
                f"component span {self._last_unstable_span:.3f} N "
                f"exceeded {self.maximum_component_span_n:.3f} N"
            )
        else:
            outcome = ForceBaselineOutcome.TIMEOUT
            detail = (
                "Force baseline did not collect enough fresh samples "
                f"within {self.timeout_sec:.1f} s"
            )
        return ForceBaselineUpdate(outcome, detail)

    def reset(self) -> None:
        self._active = False
        self._started_at = None
        self._last_received_at = None
        self._samples = []
        self._last_unstable_span = None

    def _candidate_baseline(self):
        if len(self._samples) < self.minimum_samples:
            return None

        span_sec = (
            self._samples[-1].received_at
            - self._samples[0].received_at
        )
        if span_sec + 1e-9 < self.minimum_span_sec:
            return None

        components = tuple(
            [getattr(sample, axis) for sample in self._samples]
            for axis in ("x_n", "y_n", "z_n")
        )
        maximum_span = max(
            max(values) - min(values)
            for values in components
        )
        if maximum_span > self.maximum_component_span_n + 1e-9:
            self._last_unstable_span = float(maximum_span)
            self._samples = [self._samples[-1]]
            return None

        baseline = HandForceBaseline(
            x_n=float(statistics.median(components[0])),
            y_n=float(statistics.median(components[1])),
            z_n=float(statistics.median(components[2])),
            sample_count=len(self._samples),
            sample_span_sec=float(span_sec),
            maximum_component_span_n=float(maximum_span),
        )
        return baseline

    @staticmethod
    def _positive_finite(value, label: str) -> float:
        normalized = float(value)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(
                f"{label} must be positive and finite"
            )
        return normalized


__all__ = [
    "DEFAULT_FORCE_BASELINE_MAXIMUM_COMPONENT_SPAN_N",
    "DEFAULT_FORCE_BASELINE_MINIMUM_SAMPLES",
    "DEFAULT_FORCE_BASELINE_MINIMUM_SPAN_SEC",
    "DEFAULT_FORCE_BASELINE_TIMEOUT_SEC",
    "FORCE_BASELINE_MAXIMUM_COMPONENT_SPAN_PARAMETER",
    "FORCE_BASELINE_MINIMUM_SAMPLES_PARAMETER",
    "FORCE_BASELINE_MINIMUM_SPAN_PARAMETER",
    "FORCE_BASELINE_TIMEOUT_PARAMETER",
    "ForceBaselineOutcome",
    "ForceBaselineSampler",
    "ForceBaselineUpdate",
    "HandForceBaseline",
]
