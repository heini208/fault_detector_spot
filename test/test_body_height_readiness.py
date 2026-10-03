"""Offline measurements and command-ordering tests for walking preparation."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import TransformStamped

from fault_detector_spot.navigation.body_height_readiness import (
    BodyHeightReadiness, BodyHeightSample, BodyHeightSource,
)
from fault_detector_spot.navigation.base_movement_executor import (
    BaseMovementExecutor, BaseMovementOutcome,
)
from fault_detector_spot.navigation.posture_state_source import PostureState
from test_base_movement_executor import (
    ManualClock, ManualFuture, FakeActionClient, FakeGoalHandle,
    FakePostureStateSource, movement_plan,
)


class HeightSource:
    def __init__(self, clock):
        self.clock = clock
        self.height = 0.5
        self.available = True
        self.stamp = None

    def sample(self):
        if not self.available:
            return None
        return BodyHeightSample(self.height, self.clock.now if self.stamp is None else self.stamp)


def rig(known=True):
    clock = ManualClock()
    clock.now = 10.0
    source = HeightSource(clock)
    readiness = BodyHeightReadiness(source)
    if known:
        readiness.nominal_height_m = 0.5
    send = ManualFuture()
    client = FakeActionClient(send)
    executor = BaseMovementExecutor(
        object(), action_client=client,
        posture_state_source=FakePostureStateSource(PostureState.STANDING),
        height_readiness=readiness, monotonic_clock=clock, ros_time_sec=clock,
    )
    built = []
    plan = movement_plan(executor)
    executor.motion_planner.resolve_relative = lambda command: built.append(command) or plan
    executor.motion_planner.prepare_tag_request = lambda command: command
    executor.motion_planner.resolve_tag = lambda command, source: built.append(command) or plan
    return executor, clock, source, readiness, client, built


def complete_stand(executor, client, success=True):
    result = ManualFuture()
    client.send_future.set_result(FakeGoalHandle(result))
    executor.poll()
    result.set_result(SimpleNamespace(result=SimpleNamespace(success=success, message="reset failed")))
    update = executor.poll()
    client.send_future = ManualFuture()
    return update


@pytest.mark.parametrize("operation", ["relative", "tag", "prepare_for_navigation"])
def test_nominal_height_skips_reset(operation):
    executor, clock, source, readiness, client, built = rig()
    update = getattr(executor, operation)(*(() if operation == "prepare_for_navigation" else (object(),)))
    assert client.send_calls == (0 if operation == "prepare_for_navigation" else 1)
    assert update.outcome is (BaseMovementOutcome.SUCCESS if operation == "prepare_for_navigation" else BaseMovementOutcome.RUNNING)
    assert len(built) == (0 if operation == "prepare_for_navigation" else 1)


@pytest.mark.parametrize("operation", ["relative", "tag", "prepare_for_navigation"])
@pytest.mark.parametrize("known", [False, True])
def test_reset_then_fresh_settled_height_then_movement(operation, known):
    executor, clock, source, readiness, client, built = rig(known)
    source.height = 0.35
    update = getattr(executor, operation)(*(() if operation == "prepare_for_navigation" else (object(),)))
    assert update.outcome is BaseMovementOutcome.RUNNING
    assert client.send_calls == 1
    assert built == []
    assert complete_stand(executor, client).outcome is BaseMovementOutcome.RUNNING
    # A successful action and an old height sample do not release movement.
    assert executor.poll().outcome is BaseMovementOutcome.RUNNING
    assert built == []
    source.height = 0.5
    clock.now += 0.1
    assert executor.poll().outcome is BaseMovementOutcome.RUNNING
    assert client.send_calls == 1
    clock.now += 0.31
    update = executor.poll()
    assert update.outcome is (BaseMovementOutcome.SUCCESS if operation == "prepare_for_navigation" else BaseMovementOutcome.RUNNING)
    assert client.send_calls == (1 if operation == "prepare_for_navigation" else 2)
    assert len(built) == (0 if operation == "prepare_for_navigation" else 1)
    assert readiness.nominal_height_m == pytest.approx(0.5)
    assert not readiness.reset_required


@pytest.mark.parametrize("failure", ["missing", "stale", "future", "nan"])
def test_unreliable_height_blocks_without_dispatch(failure):
    executor, clock, source, readiness, client, built = rig()
    if failure == "missing":
        source.available = False
    elif failure == "stale":
        source.stamp = 1.0
    elif failure == "future":
        source.stamp = 100.0
    else:
        source.height = float("nan")
    assert executor.prepare_for_navigation().outcome is BaseMovementOutcome.RUNNING
    clock.now += 2.1
    assert executor.poll().outcome is BaseMovementOutcome.HEIGHT_STATE_UNAVAILABLE
    assert client.send_calls == 0
    assert not executor.active


@pytest.mark.parametrize("failure", ["failed", "rejected", "wrong_height", "stale_after_reset"])
def test_reset_failure_never_releases_movement(failure):
    executor, clock, source, readiness, client, built = rig()
    source.height = 0.35
    executor.relative(object())
    if failure == "rejected":
        client.send_future.set_result(FakeGoalHandle(ManualFuture(), accepted=False))
        assert executor.poll().outcome is BaseMovementOutcome.GOAL_REJECTED
    elif failure == "failed":
        assert complete_stand(executor, client, False).outcome is BaseMovementOutcome.MOTION_FAILED
    else:
        complete_stand(executor, client)
        if failure == "stale_after_reset":
            source.stamp = clock.now
            source.height = 0.5
        clock.now += 5.1
        assert executor.poll().outcome is BaseMovementOutcome.HEIGHT_RESET_TIMEOUT
    assert built == []
    assert client.send_calls == 1
    assert not executor.active


def test_cancelling_preparation_never_dispatches_motion():
    executor, clock, source, readiness, client, built = rig()
    source.height = 0.35
    executor.prepare_for_navigation()
    result = ManualFuture()
    handle = FakeGoalHandle(result)
    client.send_future.set_result(handle)
    executor.poll()
    assert executor.relative(object()).outcome is BaseMovementOutcome.BUSY
    executor.cancel()
    assert handle.cancel_calls == 1
    result.set_result(SimpleNamespace(result=SimpleNamespace(success=False)))
    assert not executor.active
    assert built == []


def test_repeated_timestamp_cannot_confirm_settling():
    executor, clock, source, readiness, client, built = rig()
    readiness.begin_confirmation(clock.now)
    clock.now += 0.1
    source.stamp = clock.now
    assert not readiness.confirm_reset(clock.now)
    clock.now += 0.31
    assert not readiness.confirm_reset(clock.now)
    source.stamp = clock.now
    assert readiness.confirm_reset(clock.now)


def test_height_change_invalidates_quick_check_even_before_feedback():
    executor, clock, source, readiness, client, built = rig()
    executor.change_height(0.1)
    assert readiness.reset_required
    assert not readiness.at_nominal_height(source.sample())


def test_tf_source_uses_namespaced_feet_to_body_and_handles_missing_data():
    transform = TransformStamped()
    transform.header.stamp.sec = 7
    transform.transform.translation.z = 0.5
    listener = Mock()
    listener.lookup_a_tform_b.return_value = transform
    source = BodyHeightSource(listener, "spot")
    assert source.sample() == BodyHeightSample(0.5, 7.0)
    listener.lookup_a_tform_b.assert_called_once_with("spot/feet_center", "spot/body", timeout_sec=0.0)
    listener.lookup_a_tform_b.side_effect = RuntimeError("missing TF")
    assert source.sample() is None



def test_posture_loss_while_waiting_for_height_does_not_start_navigation():
    executor, clock, source, readiness, client, built = rig()
    source.available = False
    executor.prepare_for_navigation()
    source.available = True
    executor.posture_state_source.state = None
    assert executor.poll().outcome is BaseMovementOutcome.POSTURE_STATE_UNKNOWN
    assert client.send_calls == 0


def test_cancel_during_height_confirmation_keeps_reset_required():
    executor, clock, source, readiness, client, built = rig()
    source.height = 0.35
    executor.relative(object())
    complete_stand(executor, client)
    executor.cancel()
    source.height = 0.5
    clock.now += 0.1
    assert not executor.active
    assert readiness.reset_required
    assert built == []
    assert client.send_calls == 1


def test_confirmation_restarts_after_measurement_gap_or_height_motion():
    executor, clock, source, readiness, client, built = rig()
    readiness.begin_confirmation(clock.now)
    clock.now += 0.1
    assert not readiness.confirm_reset(clock.now)
    clock.now += 0.6
    assert not readiness.confirm_reset(clock.now)
    clock.now += 0.1
    source.height = 0.508
    assert not readiness.confirm_reset(clock.now)
    clock.now += 0.31
    assert readiness.confirm_reset(clock.now)


