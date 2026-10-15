"""Focused tests for arm hierarchy executor adapters."""

from types import SimpleNamespace

from py_trees.common import Status

from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementOutcome,
    ArmMovementUpdate,
)
from fault_detector_spot.manipulation.behaviours.arm_movement_behaviour import (
    ArmMovementBehaviour,
)
from fault_detector_spot.manipulation.behaviours.ready_arm_behaviour import (
    ReadyArmBehaviour,
)
from fault_detector_spot.manipulation.behaviours.stow_arm_behaviour import (
    StowArmBehaviour,
)


class FakeExecutor:

    def __init__(self, start_update, poll_update):
        self.start_update = start_update
        self.poll_update = poll_update
        self.prepare_calls = 0
        self.stow_calls = 0
        self.poll_calls = 0
        self.cancel_calls = 0
        self.tf_listener = object()

    def prepare(self, speed=None):
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


def test_ready_and_stow_share_arm_movement_behaviour():
    assert issubclass(ReadyArmBehaviour, ArmMovementBehaviour)
    assert issubclass(StowArmBehaviour, ArmMovementBehaviour)


def test_ready_action_only_starts_prepare_then_polls():
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
    action = ReadyArmBehaviour(robot_command_resources=object())
    action.executor = executor
    action.blackboard = blackboard("ready-request")

    action.initialise()
    assert action.update() is Status.RUNNING
    assert executor.prepare_calls == 1
    assert executor.poll_calls == 0

    assert action.update() is Status.SUCCESS
    assert executor.prepare_calls == 1
    assert executor.poll_calls == 1


def test_stow_action_only_starts_stow_then_polls():
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
    action = StowArmBehaviour(robot_command_resources=object())
    action.executor = executor
    action.blackboard = blackboard("stow-request")

    action.initialise()
    assert action.update() is Status.RUNNING
    assert executor.stow_calls == 1
    assert executor.poll_calls == 0

    assert action.update() is Status.SUCCESS
    assert executor.stow_calls == 1
    assert executor.poll_calls == 1


def test_common_movement_behaviour_cancels_executor_when_invalidated():
    executor = FakeExecutor(
        ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "Goal sent",
        ),
        ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "Moving",
        ),
    )
    action = ReadyArmBehaviour(robot_command_resources=object())
    action.executor = executor
    action.blackboard = blackboard("ready-cancel")

    action.initialise()
    assert action.update() is Status.RUNNING
    action.terminate(Status.INVALID)

    assert executor.cancel_calls == 1


def test_common_movement_behaviour_preserves_correlated_failure_detail():
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
    action = StowArmBehaviour(robot_command_resources=object())
    action.executor = executor
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


def test_emergency_stow_waits_for_arm_and_navigation_stop():
    calls = []
    navigation = SimpleNamespace(stopping=True)
    resources = SimpleNamespace(navigation_stopping=lambda: navigation.stopping)
    action = StowArmBehaviour(preempt=True, robot_command_resources=resources)
    action.executor = SimpleNamespace(
        active=True,
        cancel=lambda: calls.append("cancel"),
        poll=lambda: ArmMovementUpdate(
            ArmMovementOutcome.RUNNING,
            "Physical stop unconfirmed; continuing to monitor",
        ),
        stow=lambda: calls.append("stow") or ArmMovementUpdate(
            ArmMovementOutcome.RUNNING, "stowing"
        ),
    )
    action.blackboard = blackboard("cancel-all")
    action.initialise()
    assert action.update() is Status.RUNNING
    assert calls == ["cancel"]
    assert "Physical stop unconfirmed; continuing to monitor" in action.feedback_message
    action.executor.active = False
    assert action.update() is Status.RUNNING
    assert calls == ["cancel"]
    navigation.stopping = False
    assert action.update() is Status.RUNNING
    assert calls == ["cancel", "stow"]


def test_arm_failure_replacement_preserves_settings_and_is_cleared_on_next_command():
    from geometry_msgs.msg import PoseStamped
    from fault_detector_spot.application.commanding.command_ids import CommandID
    from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
    from fault_detector_spot.manipulation.behaviours.arm_goal_behaviour import ArmGoalBehaviour
    from fault_detector_spot.manipulation.commands.manipulator_move_relative_command import ManipulatorMoveRelativeCommand
    from builtin_interfaces.msg import Time

    target = PoseStamped()
    target.header.frame_id = "odom"
    target.pose.position.x = 4.0
    target.pose.orientation.w = 1.0
    semantic = SemanticCommand(CommandID.MOVE_ARM_RELATIVE, arm_speed_scale=0.3,
                               precision_mode=True, tag_position_tolerance_m=0.02,
                               ignore_environment_collisions=True)
    command = ManipulatorMoveRelativeCommand(CommandID.MOVE_ARM_RELATIVE, Time())
    command.semantic_command = semantic
    executor = FakeExecutor(None, None)
    executor.relative = lambda *args, **kwargs: ArmMovementUpdate(
        ArmMovementOutcome.MOTION_FAILED, "Stopped", retry_safe=True, retry_target=target,
    )
    action = ArmGoalBehaviour("test", robot_command_resources=object())
    action.executor = executor
    action.blackboard = blackboard(command.request_id)
    action.blackboard.last_command = command
    action.blackboard.exists = lambda key: hasattr(action.blackboard, key)
    action.initialise()
    assert action.update() is Status.FAILURE
    replacement = command.retry_command
    assert replacement.command_id is CommandID.MOVE_ARM_TO_POSE
    assert replacement.offset.frame_id == "odom"
    assert replacement.offset.position.x == 4.0
    assert replacement.arm_speed_scale == 0.3
    assert replacement.precision_mode
    assert replacement.tag_position_tolerance_m == 0.02
    assert replacement.ignore_environment_collisions
    assert command.retry_safe
    action.initialise()
    assert command.retry_command is None
    assert not command.retry_safe
