"""Cancellation retains ownership through action completion and physical stop."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus

from fault_detector_spot.shared.execution.movement_executor import MovementExecutor
from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementExecutor, _ArmOperation
from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome, ArmMovementUpdate
from test_base_movement_executor import ManualFuture, FakeGoalHandle, FakeActionClient


class RequestedResultGoalHandle(FakeGoalHandle):
    """rclpy sends a new request/future for every get_result_async call."""

    def __init__(self):
        super().__init__(None)
        self.requests = []

    def get_result_async(self):
        future = ManualFuture()
        self.requests.append(future)
        return future


@pytest.mark.parametrize("observe_settlement_first", [False, True])
def test_completed_arm_goal_reuses_terminal_result_for_next_motion(observe_settlement_first):
    sent = ManualFuture()
    executor = ArmMovementExecutor(object(), action_client=FakeActionClient(sent))
    executor._operation = _ArmOperation.MOVEMENT
    handle = RequestedResultGoalHandle()
    assert executor._start_goal(object).outcome is ArmMovementOutcome.RUNNING
    sent.set_result(handle)
    if observe_settlement_first:
        assert not executor.goals_settled()
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    assert len(handle.requests) == 1
    handle.requests[0].set_result(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED, result=SimpleNamespace(success=True),
    ))
    assert executor.poll().outcome is ArmMovementOutcome.SUCCESS

    # Completion is already known; a return must not be rejected while an
    # unnecessary second result request makes the action look unsettled.
    assert executor._motion_admission_error() is None
    assert len(handle.requests) == 1
    executor._stop_unconfirmed = True
    assert executor._motion_admission_error().outcome is ArmMovementOutcome.STOP_UNCONFIRMED


@pytest.mark.parametrize("poll_acceptance", [False, True])
def test_cancellation_and_settlement_share_the_same_result_request(poll_acceptance):
    sent = ManualFuture()
    executor = MovementExecutor(object(), action_client=FakeActionClient(sent))
    executor.OUTCOME_TYPE = ArmMovementOutcome
    executor.UPDATE_TYPE = ArmMovementUpdate
    handle = RequestedResultGoalHandle()
    executor._start_goal(object)
    sent.set_result(handle)
    if poll_acceptance:
        executor.poll()
    executor.cancel()
    assert len(handle.requests) == 1
    assert not executor.goals_settled()
    assert len(handle.requests) == 1
    handle.requests[0].set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert not executor.active
    assert executor.goals_settled()
    assert len(handle.requests) == 1


def test_late_acceptance_and_cancel_ack_do_not_release_movement():
    executor = MovementExecutor(object())
    executor._active = True
    sent = ManualFuture()
    result = ManualFuture()
    executor._send_goal_future = sent
    executor.cancel()
    assert executor.active and executor.cancelling
    handle = FakeGoalHandle(result)
    sent.set_result(handle)
    assert handle.cancel_calls == 1
    assert executor.active
    executor.cancel()
    assert handle.cancel_calls == 1
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert not executor.active


def test_failed_result_future_does_not_confirm_stop():
    executor = MovementExecutor(object())
    executor._active = True
    sent = ManualFuture()
    result = ManualFuture()
    executor._send_goal_future = sent
    sent.set_result(FakeGoalHandle(result))
    executor.cancel()
    result.result = Mock(side_effect=RuntimeError("connection lost"))
    result.set_result(None)
    assert executor.active and executor.cancelling
    assert "unconfirmed" in executor._cancellation_detail


def test_arm_requires_terminal_goal_and_subsequent_physical_stop():
    executor = ArmMovementExecutor(object(), monotonic_clock=lambda: 10.)
    executor._active = True
    executor._cancelling = True
    executor._cancel_goal_done = False
    executor._reset_operation = Mock(wraps=executor._reset_operation)
    executor.guarded_probe_execution = Mock()
    executor._guarded_probe_monitor = Mock()
    stopped = ArmMovementUpdate(ArmMovementOutcome.TRAJECTORY_CANCELLED, "stopped")
    executor._cancel_stop_finished(stopped)
    executor._reset_operation.assert_not_called()
    assert executor.active and executor.cancelling
    executor._cancellation_goal_finished()
    executor._guarded_probe_monitor.cancel.assert_called_once_with(
        executor._cancel_stop_finished, continue_after_timeout=True,
    )
    executor._reset_operation.assert_not_called()
    assert executor.active and executor.cancelling
    executor._cancel_stop_finished(ArmMovementUpdate(ArmMovementOutcome.STOP_UNCONFIRMED, "stale velocity"))
    executor._reset_operation.assert_not_called()
    assert executor.active and executor.cancelling
    executor._cancel_stop_finished(stopped)
    executor._reset_operation.assert_called_once()
    assert not executor.active and not executor.cancelling
