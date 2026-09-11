"""Focused tests for stationary force baseline acquisition."""

from fault_detector_spot.manipulation.arm_force_baseline import (
    ForceBaselineOutcome,
    ForceBaselineSampler,
)
from fault_detector_spot.manipulation.arm_state_source import (
    HandForceSample,
)


class ManualClock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeArmStateSource:

    def __init__(self):
        self.sample = None
        self.last_received_at = 0.0
        self.stale = False

    def hand_force_sample(self):
        if self.stale:
            return None
        return self.sample

    def is_stale(self):
        return self.stale


def force_sample(received_at, x=1.0, y=2.0, z=3.0):
    return HandForceSample(
        received_at=received_at,
        x_n=x,
        y_n=y,
        z_n=z,
    )


def test_baseline_requires_new_samples_and_minimum_span():
    clock = ManualClock()
    state = FakeArmStateSource()
    sampler = ForceBaselineSampler(
        state,
        minimum_samples=3,
        minimum_span_sec=0.2,
        maximum_component_span_n=1.0,
        timeout_sec=1.0,
        monotonic_clock=clock,
    )

    state.sample = force_sample(0.0)
    assert sampler.start().outcome is ForceBaselineOutcome.RUNNING

    clock.now = 0.1
    assert sampler.poll().outcome is ForceBaselineOutcome.RUNNING

    state.sample = force_sample(0.1, x=1.1)
    sampler.poll()

    clock.now = 0.2
    state.sample = force_sample(0.2, x=0.9)
    ready = sampler.poll()

    assert ready.outcome is ForceBaselineOutcome.READY
    assert ready.baseline.sample_count == 3
    assert ready.baseline.sample_span_sec == 0.2
    assert ready.baseline.last_received_at == 0.2
    assert ready.baseline.x_n == 1.0


def test_unstable_candidate_resets_and_can_recover():
    clock = ManualClock()
    state = FakeArmStateSource()
    sampler = ForceBaselineSampler(
        state,
        minimum_samples=3,
        minimum_span_sec=0.2,
        maximum_component_span_n=1.0,
        timeout_sec=2.0,
        monotonic_clock=clock,
    )

    state.sample = force_sample(0.0, x=0.0)
    sampler.start()

    clock.now = 0.1
    state.sample = force_sample(0.1, x=5.0)
    sampler.poll()

    clock.now = 0.2
    state.sample = force_sample(0.2, x=0.0)
    assert sampler.poll().outcome is ForceBaselineOutcome.RUNNING

    clock.now = 0.3
    state.sample = force_sample(0.3, x=0.1)
    sampler.poll()

    clock.now = 0.4
    state.sample = force_sample(0.4, x=0.0)
    ready = sampler.poll()

    assert ready.outcome is ForceBaselineOutcome.READY
    assert ready.baseline.sample_count == 3
    assert ready.baseline.sample_span_sec == 0.2


def test_missing_force_field_reports_unavailable_after_timeout():
    clock = ManualClock()
    state = FakeArmStateSource()
    state.sample = None
    sampler = ForceBaselineSampler(
        state,
        minimum_samples=3,
        minimum_span_sec=0.2,
        timeout_sec=0.5,
        monotonic_clock=clock,
    )

    sampler.start()
    clock.now = 0.5
    failed = sampler.poll()

    assert failed.outcome is ForceBaselineOutcome.FORCE_UNAVAILABLE


def test_stale_manipulator_state_reports_force_stale():
    clock = ManualClock()
    state = FakeArmStateSource()
    sampler = ForceBaselineSampler(
        state,
        minimum_samples=3,
        minimum_span_sec=0.2,
        timeout_sec=0.5,
        monotonic_clock=clock,
    )

    sampler.start()
    state.stale = True
    clock.now = 0.5
    failed = sampler.poll()

    assert failed.outcome is ForceBaselineOutcome.FORCE_STALE


def test_baseline_ignores_force_received_before_acquisition_started():
    clock = ManualClock()
    state = FakeArmStateSource()
    state.sample = force_sample(0.5)
    clock.now = 1.0
    sampler = ForceBaselineSampler(
        state,
        minimum_samples=3,
        minimum_span_sec=0.2,
        timeout_sec=1.0,
        monotonic_clock=clock,
    )

    first = sampler.start()

    assert first.outcome is ForceBaselineOutcome.RUNNING

    state.sample = force_sample(1.0)
    sampler.poll()
    clock.now = 1.1
    state.sample = force_sample(1.1)
    sampler.poll()
    clock.now = 1.2
    state.sample = force_sample(1.2)
    ready = sampler.poll()

    assert ready.outcome is ForceBaselineOutcome.READY
    assert ready.baseline.last_received_at == 1.2
