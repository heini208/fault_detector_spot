"""Tests for measured hand motion cached by ArmStateSource."""

from types import SimpleNamespace

import pytest
from bosdyn_api_msgs.msg import ManipulatorState, ManipulatorStateStowState

from fault_detector_spot.manipulation.arm_state_source import (
    ArmStateSource,
    ArmStowState,
)


class ManualClock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeNode:

    def __init__(self):
        self.callback = None
        self.destroyed = []

    def create_subscription(
        self,
        _message_type,
        _topic,
        callback,
        _depth,
    ):
        self.callback = callback
        return object()

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)


def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def manipulator_state(
    *,
    velocity_present=True,
    linear=(0.0, 0.0, 0.0),
    angular=(0.0, 0.0, 0.0),
):
    has_field = 0
    if velocity_present:
        has_field |= int(
            ManipulatorState.VELOCITY_OF_HAND_IN_VISION_FIELD_SET
        )

    return SimpleNamespace(
        stow_state=SimpleNamespace(
            value=ManipulatorStateStowState.STOWSTATE_DEPLOYED
        ),
        has_field=has_field,
        velocity_of_hand_in_vision=SimpleNamespace(
            linear=vector(*linear),
            angular=vector(*angular),
        ),
    )


def test_source_caches_measured_vision_frame_hand_velocity():
    clock = ManualClock()
    node = FakeNode()
    source = ArmStateSource(
        node,
        monotonic_clock=clock,
    )

    clock.now = 1.25
    node.callback(
        manipulator_state(
            linear=(0.03, 0.04, 0.0),
            angular=(0.0, 0.0, 0.12),
        )
    )

    assert source.stow_state() is ArmStowState.DEPLOYED
    sample = source.hand_velocity_sample()

    assert sample.received_at == pytest.approx(1.25)
    assert sample.linear_speed_mps == pytest.approx(0.05)
    assert sample.angular_speed_rad_s == pytest.approx(0.12)


def test_source_does_not_reuse_velocity_when_latest_field_is_absent():
    clock = ManualClock()
    node = FakeNode()
    source = ArmStateSource(
        node,
        monotonic_clock=clock,
    )

    node.callback(
        manipulator_state(
            linear=(0.01, 0.0, 0.0),
        )
    )
    assert source.hand_velocity_sample() is not None

    clock.now = 0.1
    node.callback(
        manipulator_state(
            velocity_present=False,
        )
    )

    assert source.hand_velocity_sample() is None


def test_velocity_sample_becomes_unavailable_when_state_is_stale():
    clock = ManualClock()
    node = FakeNode()
    source = ArmStateSource(
        node,
        stale_after_sec=1.0,
        monotonic_clock=clock,
    )

    node.callback(manipulator_state())
    assert source.hand_velocity_sample() is not None

    clock.now = 1.01

    assert source.hand_velocity_sample() is None
    assert source.stow_state() is None
    assert source.is_stale()
