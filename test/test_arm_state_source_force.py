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
        *,
        callback_group=None,
    ):
        self.callback = callback
        self.callback_group = callback_group
        self.subscription = object()
        return self.subscription

    def destroy_subscription(self, _subscription):
        pass


def manipulator_state(
    *,
    force_present=True,
    force=(0.0, 0.0, 0.0),
    velocity_present=False,
    linear_velocity=(0.0, 0.0, 0.0),
    angular_velocity=(0.0, 0.0, 0.0),
):
    has_field = 0
    if force_present:
        has_field |= int(
            ManipulatorState
            .ESTIMATED_END_EFFECTOR_FORCE_IN_HAND_FIELD_SET
        )
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
            linear=SimpleNamespace(
                x=linear_velocity[0],
                y=linear_velocity[1],
                z=linear_velocity[2],
            ),
            angular=SimpleNamespace(
                x=angular_velocity[0],
                y=angular_velocity[1],
                z=angular_velocity[2],
            ),
        ),
        estimated_end_effector_force_in_hand=SimpleNamespace(
            x=force[0],
            y=force[1],
            z=force[2],
        ),
    )


def test_source_uses_dedicated_callback_group():
    node = FakeNode()

    source = ArmStateSource(node)

    assert node.callback_group is source._callback_group
    assert node.callback_group is not None


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


def test_force_listeners_run_after_update_outside_lock_and_can_be_removed():
    node = FakeNode()
    source = ArmStateSource(node)
    observations = []

    def observe(sample):
        assert not source._lock._is_owned()
        assert source.hand_force_sample() is sample
        observations.append(sample)

    source.add_force_listener(observe)
    node.callback(manipulator_state(force=(1.0, 2.0, 3.0)))
    node.callback(manipulator_state(force_present=False))
    source.remove_force_listener(observe)
    node.callback(manipulator_state())
    assert len(observations) == 2
    assert observations[0].x_n == 1.0
    assert observations[1] is None


def test_failed_listener_cannot_prevent_other_listeners_or_later_updates():
    node = FakeNode()
    source = ArmStateSource(node)
    observations = []

    def broken(_sample):
        raise RuntimeError('listener failed')

    source.add_force_listener(broken)
    source.add_force_listener(observations.append)
    node.callback(manipulator_state(force=(1.0, 2.0, 3.0)))
    node.callback(manipulator_state(force=(4.0, 5.0, 6.0)))
    assert [sample.x_n for sample in observations] == [1.0, 4.0]
    assert source.hand_force_sample().x_n == 4.0
