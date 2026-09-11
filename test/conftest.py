"""Shared test adaptation for guarded ArmMovementExecutor public calls."""

import pytest

from fault_detector_spot.manipulation.arm_force_baseline import (
    ForceBaselineOutcome,
    ForceBaselineUpdate,
    HandForceBaseline,
)
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)
from fault_detector_spot.manipulation.arm_state_source import HandForceSample
from fault_detector_spot.manipulation.hand_settling_detector import (
    HandSettlingOutcome,
    HandSettlingUpdate,
)


class _ImmediateSettlingDetector:

    def start(self):
        return HandSettlingUpdate(
            HandSettlingOutcome.SETTLED,
            "settled",
        )

    def poll(self):
        raise AssertionError("Immediate settling should not need polling")

    def reset(self):
        pass


class _ImmediateForceBaselineSampler:

    def start(self):
        return ForceBaselineUpdate(
            ForceBaselineOutcome.READY,
            "baseline ready",
            HandForceBaseline(
                x_n=0.0,
                y_n=0.0,
                z_n=0.0,
                sample_count=10,
                sample_span_sec=0.5,
                maximum_component_span_n=0.0,
                last_received_at=0.0,
            ),
        )

    def poll(self):
        raise AssertionError("Immediate baseline should not need polling")

    def reset(self):
        pass


class _PermissiveForceContactPolicy:
    consecutive_samples = 2

    @staticmethod
    def threshold_for(_linear_speed_mps):
        return 1e9


@pytest.fixture(autouse=True)
def _guarded_arm_executor_defaults(request, monkeypatch):
    if request.node.fspath.basename != "test_arm_movement_executor.py":
        yield
        return

    original_init = ArmMovementExecutor.__init__

    def guarded_init(self, *args, **kwargs):
        kwargs.setdefault(
            "settling_detector",
            _ImmediateSettlingDetector(),
        )
        kwargs.setdefault(
            "force_baseline_sampler",
            _ImmediateForceBaselineSampler(),
        )
        kwargs.setdefault(
            "force_contact_policy",
            _PermissiveForceContactPolicy(),
        )
        original_init(self, *args, **kwargs)

        source = self.arm_state_source
        if source is not None and not hasattr(source, "hand_force_sample"):
            source.hand_force_sample = lambda: HandForceSample(
                received_at=self._monotonic_clock(),
                x_n=0.0,
                y_n=0.0,
                z_n=0.0,
            )

    monkeypatch.setattr(ArmMovementExecutor, "__init__", guarded_init)
    yield
