"""Gripper commands use fresh robot state and the shared arm owner."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from py_trees.common import Status
from action_msgs.msg import GoalStatus
from bosdyn_api_msgs.msg import ManipulatorState

from fault_detector_spot.manipulation.arm_state_source import ArmStateSource
from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementOutcome
from fault_detector_spot.manipulation.behaviours.close_gripper_action import CloseGripperAction
from fault_detector_spot.manipulation.behaviours.toggle_gripper_action import ToggleGripperAction
import fault_detector_spot.manipulation.arm_movement_executor as executor_module
from test_arm_movement_executor import executor_with_client, FakeGoalHandle, ManualFuture


@pytest.mark.parametrize("percentage", [0.0, 25.0, 100.0, -1.0, 101.0, float("nan"), float("inf")])
def test_measured_gripper_opening_validity_and_freshness(percentage):
    now = [0.0]
    source = ArmStateSource(Mock(), stale_after_sec=1.0, monotonic_clock=lambda: now[0])
    assert source.gripper_open_percentage() is None
    message = ManipulatorState()
    message.gripper_open_percentage = percentage
    source._receive_state(message)
    if 0.0 <= percentage <= 100.0:
        assert source.gripper_open_percentage() == percentage
    else:
        assert source.gripper_open_percentage() is None
    now[0] = 1.1
    assert source.gripper_open_percentage() is None


@pytest.mark.parametrize("percentage,target", [(0.0, 1.0), (25.0, 1.0), (50.0, 1.0), (75.0, 0.0), (100.0, 0.0)])
def test_toggle_uses_measured_position_and_rejection_does_not_flip_it(monkeypatch, percentage, target):
    state = SimpleNamespace(gripper_open_percentage=lambda: percentage)
    executor, client = executor_with_client(object(), arm_state_source=state)
    build = Mock(return_value=object())
    monkeypatch.setattr(executor_module, "build_gripper_goal", build)
    assert executor.toggle_gripper().outcome is ArmMovementOutcome.RUNNING
    build.assert_called_once_with(target)
    assert executor.close_gripper().outcome is ArmMovementOutcome.BUSY
    client.send_future.set_result(FakeGoalHandle(accepted=False))
    assert executor.poll().outcome is ArmMovementOutcome.GOAL_REJECTED
    client.send_future = ManualFuture()
    assert executor.toggle_gripper().outcome is ArmMovementOutcome.RUNNING
    assert [call.args[0] for call in build.call_args_list] == [target, target]


def finish(executor, client, success=True):
    result = ManualFuture()
    client.send_future.set_result(FakeGoalHandle(result))
    assert executor.poll().outcome is ArmMovementOutcome.RUNNING
    result.set_result(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED if success else GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(success=success, message=""),
    ))
    return executor.poll()


def test_close_and_external_state_changes_are_seen_by_next_toggle(monkeypatch):
    state = SimpleNamespace(gripper_open_percentage=lambda: 100.0)
    executor, client = executor_with_client(object(), arm_state_source=state)
    build = Mock(return_value=object())
    monkeypatch.setattr(executor_module, "build_gripper_goal", build)
    assert executor.close_gripper().outcome is ArmMovementOutcome.RUNNING
    assert finish(executor, client).outcome is ArmMovementOutcome.SUCCESS
    for measured, expected in [(0.0, 1.0), (100.0, 0.0)]:
        state.gripper_open_percentage = lambda: measured
        client.send_future = ManualFuture()
        assert executor.toggle_gripper().outcome is ArmMovementOutcome.RUNNING
        assert build.call_args.args == (expected,)
        assert finish(executor, client, success=False).outcome is ArmMovementOutcome.MOTION_FAILED


@pytest.mark.parametrize("source", [None, SimpleNamespace(gripper_open_percentage=lambda: None)])
def test_toggle_requires_feedback_but_explicit_close_does_not(monkeypatch, source):
    executor, client = executor_with_client(object(), arm_state_source=source)
    monkeypatch.setattr(executor_module, "build_gripper_goal", lambda fraction: fraction)
    assert executor.toggle_gripper().outcome is ArmMovementOutcome.ARM_STATE_UNAVAILABLE
    assert not client.sent_goals
    assert executor.close_gripper().outcome is ArmMovementOutcome.RUNNING
    assert client.sent_goals == [0.0]


@pytest.mark.parametrize("behaviour_type,method", [
    (ToggleGripperAction, "toggle_gripper"), (CloseGripperAction, "close_gripper"),
])
def test_gripper_behaviours_delegate_to_same_executor(behaviour_type, method):
    executor = Mock()
    getattr(executor, method).return_value = SimpleNamespace(
        outcome=ArmMovementOutcome.RUNNING, detail="moving",
    )
    resources = Mock()
    resources.get_arm_movement_executor.return_value = executor
    behaviour = behaviour_type(robot_command_resources=resources)
    assert behaviour.update() is Status.RUNNING
    assert behaviour.executor is executor
    getattr(executor, method).assert_called_once_with()
    behaviour.terminate(Status.INVALID)
    executor.cancel.assert_called_once_with()
