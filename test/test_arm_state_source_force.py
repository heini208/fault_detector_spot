"""Tests for end-effector force cached by ArmStateSource."""

from types import SimpleNamespace

import pytest
from bosdyn_api_msgs.msg import ManipulatorState, ManipulatorStateStowState

from fault_detector_spot.manipulation.arm_state_source import (
    ArmStateSource,
)


class ManualClock:

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeNode:

    def create_subscription(
        self,
        _message_type,
        _topic,
        callback,
        _depth,
    ):
        self.callback = callback
        self.subscription = object()
        return self.subscription

    def destroy_subscription(self, _subscription):
        pass


def manipulator_state(
    *,
    force_present=True,
    force=(0.0, 0.0, 0.0),
):
    has_field = 0
    if force_present:
        has_field |= int(
            ManipulatorState
            .ESTIMATED_END_EFFECTOR_FORCE_IN_HAND_FIELD_SET
        )
    return SimpleNamespace(
        stow_state=SimpleNamespace(
            value=ManipulatorStateStowState.STOWSTATE_DEPLOYED
        ),
        has_field=has_field,
        velocity_of_hand_in_vision=SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        estimated_end_effector_force_in_hand=SimpleNamespace(
            x=force[0],
            y=force[1],
            z=force[2],
        ),
    )


def test_source_exposes_fresh_hand_frame_force():
    clock = ManualClock()
    node = FakeNode()
    source = ArmStateSource(
        node,
        monotonic_clock=clock,
    )

    clock.now = 1.25
    node.callback(
        manipulator_state(force=(3.0, 4.0, 12.0))
    )

    sample = source.hand_force_sample()

    assert sample.received_at == pytest.approx(1.25)
    assert sample.x_n == pytest.approx(3.0)
    assert sample.y_n == pytest.approx(4.0)
    assert sample.z_n == pytest.approx(12.0)
    assert sample.magnitude_n == pytest.approx(13.0)


def test_latest_message_without_force_does_not_reuse_old_force():
    clock = ManualClock()
    node = FakeNode()
    source = ArmStateSource(
        node,
        monotonic_clock=clock,
    )

    node.callback(manipulator_state(force=(1.0, 0.0, 0.0)))
    assert source.hand_force_sample() is not None

    clock.now = 0.1
    node.callback(
        manipulator_state(force_present=False)
    )

    assert source.hand_force_sample() is None


def test_force_becomes_unavailable_with_stale_manipulator_state():
    clock = ManualClock()
    node = FakeNode()
    source = ArmStateSource(
        node,
        stale_after_sec=1.0,
        monotonic_clock=clock,
    )

    node.callback(manipulator_state())
    assert source.hand_force_sample() is not None

    clock.now = 1.01

    assert source.hand_force_sample() is None
