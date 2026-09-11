"""Focused tests for ready/stow behavior-tree executor adapters."""

from types import SimpleNamespace

from py_trees.common import Status

from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.behaviours.ready_arm_action import (
    ReadyArmActionSimple,
)
from fault_detector_spot.manipulation.behaviours.stow_arm_action import (
    StowArmActionSimple,
)


class FakeExecutor:

    def __init__(self, start_update, poll_update):
        self.start_update = start_update
        self.poll_update = poll_update
        self.prepare_calls = 0
        self.stow_calls = 0
        self.poll_calls = 0
        self.cancel_calls = 0

    def prepare(self):
        self.prepare_calls += 1
        return self.start_update

    def stow(self):
        self.stow_calls += 1
        return self.start_update

    def poll(self):
        self.poll_calls += 1
        return self.poll_update

    def cancel(self):
        self.cancel_calls += 1


def blackboard(request_id):
    return SimpleNamespace(
        last_command=SimpleNamespace(request_id=request_id),
        command_failure_request_id="",
        command_failure_detail="",
    )


def test_ready_action_is_thin_prepare_adapter():
    executor = FakeExecutor(
        ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "Goal sent",
        ),
        ArmMovementUpdate(
            ArmMovementOutcome.SUCCESS,
            "Arm deployed",
        ),
    )
    action = ReadyArmActionSimple(
        robot_command_resources=object()
    )
    action.arm_movement_executor = executor
    action.blackboard = blackboard("ready-request")

    action.initialise()
    assert action.update() is Status.RUNNING
    assert executor.prepare_calls == 1
    assert executor.poll_calls == 0

    assert action.update() is Status.SUCCESS
    assert executor.prepare_calls == 1
    assert executor.poll_calls == 1
    assert action.feedback_message == "Arm deployed"


def test_stow_action_is_thin_stow_adapter():
    executor = FakeExecutor(
        ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "Goal sent",
        ),
        ArmMovementUpdate(
            ArmMovementOutcome.SUCCESS,
            "Arm stowed",
        ),
    )
    action = StowArmActionSimple(
        robot_command_resources=object()
    )
    action.arm_movement_executor = executor
    action.blackboard = blackboard("stow-request")

    action.initialise()
    assert action.update() is Status.RUNNING
    assert executor.stow_calls == 1
    assert executor.poll_calls == 0

    assert action.update() is Status.SUCCESS
    assert executor.stow_calls == 1
    assert executor.poll_calls == 1
    assert action.feedback_message == "Arm stowed"


def test_ready_action_cancels_executor_when_invalidated():
    executor = FakeExecutor(
        ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "Waiting for manipulator stow state",
        ),
        ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "Waiting for manipulator stow state",
        ),
    )
    action = ReadyArmActionSimple(
        robot_command_resources=object()
    )
    action.arm_movement_executor = executor
    action.blackboard = blackboard("ready-cancel")

    action.initialise()
    assert action.update() is Status.RUNNING

    action.terminate(Status.INVALID)

    assert executor.cancel_calls == 1


def test_stow_action_preserves_correlated_failure_detail():
    executor = FakeExecutor(
        ArmMovementUpdate(
            ArmMovementOutcome.ARM_STATE_STALE,
            "Fresh manipulator stow state was unavailable for 2.0 s",
        ),
        ArmMovementUpdate(
            ArmMovementOutcome.SUCCESS,
            "unused",
        ),
    )
    action = StowArmActionSimple(
        robot_command_resources=object()
    )
    action.arm_movement_executor = executor
    action.blackboard = blackboard("stow-failure")

    action.initialise()
    assert action.update() is Status.FAILURE

    assert (
        action.blackboard.command_failure_request_id
        == "stow-failure"
    )
    assert action.blackboard.command_failure_detail == (
        "Fresh manipulator stow state was unavailable for 2.0 s"
    )
