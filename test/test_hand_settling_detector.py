"""Focused tests for physical hand settling detection."""

from types import SimpleNamespace

import pytest

from fault_detector_spot.manipulation.arm_state_source import (
    HandVelocitySample,
)
from fault_detector_spot.manipulation.hand_settling_detector import (
    HandSettlingDetector,
    HandSettlingOutcome,
)


class ManualClock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeArmStateSource:

    def __init__(self):
        self.sample = None

    def hand_velocity_sample(self):
        return self.sample


class FakeTransformer:

    def __init__(self):
        self.value = None
        self.fail = False
        self.calls = []

    def lookup_a_tform_b(self, target, source, timeout_sec=0.0):
        self.calls.append((target, source, timeout_sec))
        if self.fail or self.value is None:
            raise RuntimeError("TF unavailable")
        return self.value


def velocity_sample(
    received_at,
    linear=0.0,
    angular=0.0,
):
    return HandVelocitySample(
        received_at=received_at,
        linear_x_mps=linear,
        linear_y_mps=0.0,
        linear_z_mps=0.0,
        angular_x_rad_s=angular,
        angular_y_rad_s=0.0,
        angular_z_rad_s=0.0,
    )


def transform(x=0.0, yaw=0.0):
    import math

    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(
                x=x,
                y=0.0,
                z=0.0,
            ),
            rotation=SimpleNamespace(
                x=0.0,
                y=0.0,
                z=math.sin(yaw * 0.5),
                w=math.cos(yaw * 0.5),
            ),
        )
    )


def detector(clock, state, tf, **kwargs):
    return HandSettlingDetector(
        arm_state_source=state,
        tf_listener=tf,
        stable_duration_sec=kwargs.pop(
            "stable_duration_sec",
            0.4,
        ),
        timeout_sec=kwargs.pop("timeout_sec", 3.0),
        monotonic_clock=clock,
        **kwargs,
    )


def test_velocity_requires_continuously_low_new_samples():
    clock = ManualClock()
    state = FakeArmStateSource()
    tf = FakeTransformer()
    settling = detector(clock, state, tf)

    state.sample = velocity_sample(0.0, linear=0.0)
    first = settling.start()
    assert first.outcome is HandSettlingOutcome.RUNNING

    clock.now = 0.2
    state.sample = velocity_sample(0.2, linear=0.02)
    moving = settling.poll()
    assert moving.outcome is HandSettlingOutcome.RUNNING
    assert "still moving" in moving.detail

    clock.now = 0.3
    state.sample = velocity_sample(0.3, linear=0.0)
    settling.poll()

    clock.now = 0.5
    state.sample = velocity_sample(0.5, linear=0.0)
    assert settling.poll().outcome is HandSettlingOutcome.RUNNING

    clock.now = 0.7
    state.sample = velocity_sample(0.7, linear=0.0)
    settled = settling.poll()

    assert settled.outcome is HandSettlingOutcome.SETTLED
    assert settled.source == "velocity"
    assert not settling.active
    assert tf.calls == []


def test_repeated_old_zero_velocity_sample_does_not_fake_settling():
    clock = ManualClock()
    state = FakeArmStateSource()
    tf = FakeTransformer()
    settling = detector(
        clock,
        state,
        tf,
        stable_duration_sec=0.2,
        timeout_sec=0.5,
    )

    state.sample = velocity_sample(0.0, linear=0.0)
    settling.start()

    clock.now = 0.25
    repeated = settling.poll()

    assert repeated.outcome is HandSettlingOutcome.RUNNING
    assert "new measured" in repeated.detail

    clock.now = 0.5
    timed_out = settling.poll()

    assert timed_out.outcome is HandSettlingOutcome.TIMEOUT


def test_pose_fallback_estimates_motion_and_can_settle():
    clock = ManualClock()
    state = FakeArmStateSource()
    tf = FakeTransformer()
    tf.value = transform(x=0.0)
    settling = detector(clock, state, tf)

    first = settling.start()
    assert first.outcome is HandSettlingOutcome.RUNNING

    clock.now = 0.1
    tf.value = transform(x=0.0)
    below = settling.poll()

    assert below.outcome is HandSettlingOutcome.RUNNING
    assert below.source == "pose"
    assert below.linear_speed_mps == pytest.approx(0.0)

    clock.now = 0.5
    tf.value = transform(x=0.0)
    settled = settling.poll()

    assert settled.outcome is HandSettlingOutcome.SETTLED
    assert settled.source == "pose"


def test_pose_fallback_resets_stable_window_when_hand_moves():
    clock = ManualClock()
    state = FakeArmStateSource()
    tf = FakeTransformer()
    tf.value = transform(x=0.0)
    settling = detector(clock, state, tf)

    settling.start()

    clock.now = 0.1
    tf.value = transform(x=0.0)
    settling.poll()

    clock.now = 0.2
    tf.value = transform(x=0.02)
    moving = settling.poll()

    assert moving.outcome is HandSettlingOutcome.RUNNING
    assert moving.linear_speed_mps == pytest.approx(0.2)

    clock.now = 0.3
    tf.value = transform(x=0.02)
    settling.poll()

    clock.now = 0.7
    tf.value = transform(x=0.02)
    settled = settling.poll()

    assert settled.outcome is HandSettlingOutcome.SETTLED


def test_unavailable_velocity_and_tf_fail_after_bounded_wait():
    clock = ManualClock()
    state = FakeArmStateSource()
    tf = FakeTransformer()
    tf.fail = True
    settling = detector(
        clock,
        state,
        tf,
        timeout_sec=1.0,
    )

    first = settling.start()
    assert first.outcome is HandSettlingOutcome.RUNNING

    clock.now = 1.0
    failed = settling.poll()

    assert failed.outcome is HandSettlingOutcome.SENSING_UNAVAILABLE
    assert not settling.active


def test_continuous_motion_returns_timeout_not_sensing_failure():
    clock = ManualClock()
    state = FakeArmStateSource()
    tf = FakeTransformer()
    settling = detector(
        clock,
        state,
        tf,
        timeout_sec=1.0,
    )

    state.sample = velocity_sample(0.0, linear=0.02)
    settling.start()

    clock.now = 0.5
    state.sample = velocity_sample(0.5, linear=0.02)
    settling.poll()

    clock.now = 1.0
    state.sample = velocity_sample(1.0, linear=0.02)
    failed = settling.poll()

    assert failed.outcome is HandSettlingOutcome.TIMEOUT
    assert not settling.active
