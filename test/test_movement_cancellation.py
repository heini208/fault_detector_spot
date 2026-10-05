"""Cancellation retains ownership through action completion and physical stop."""

from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus

from fault_detector_spot.shared.execution.movement_executor import MovementExecutor
from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementExecutor
from fault_detector_spot.manipulation.arm_movement_result import ArmMovementOutcome, ArmMovementUpdate
from test_base_movement_executor import ManualFuture, FakeGoalHandle


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
    executor = ArmMovementExecutor.__new__(ArmMovementExecutor)
    executor._execution_lock = RLock()
    executor._cancel_goal_done = False
    executor._reset_operation = Mock()
    executor.guarded_probe_execution = Mock()
    executor._guarded_probe_monitor = Mock()
    stopped = ArmMovementUpdate(ArmMovementOutcome.TRAJECTORY_CANCELLED, "stopped")
    executor._cancel_stop_finished(stopped)
    executor._reset_operation.assert_not_called()
    executor._cancellation_goal_finished()
    executor._guarded_probe_monitor.cancel.assert_called_once()
    executor._reset_operation.assert_not_called()
    executor._cancel_stop_finished(ArmMovementUpdate(ArmMovementOutcome.STOP_UNCONFIRMED, "stale velocity"))
    executor._reset_operation.assert_not_called()
    executor._cancel_stop_finished(stopped)
    executor._reset_operation.assert_called_once()
