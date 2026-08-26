"""Tests for the shared bounded behavior-tree action lifecycle."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time
import pytest
from fault_detector_spot.application.behaviour_tree.behaviours.move_command_action import (
    MoveCommandAction,
)
from fault_detector_spot.application.behaviour_tree.behaviours.spot_action import (
    BoundedActionClientBehaviour,
    RobotCommandActionBehaviour,
    WorkflowActionBehaviour,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.manipulation.behaviours.manipulator_move_close_to_surface_action import (
    ManipulatorMoveCloseToSurfaceAction,
)
from fault_detector_spot.manipulation.commands.move_close_to_surface_command import (
    MoveCloseToSurfaceCommand,
)
from py_trees.common import Status


class ManualClock:
    """Controllable monotonic clock."""

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class ManualFuture:
    """Minimal manually completed future."""

    def __init__(self):
        self._done = False
        self._result = None
        self._exception = None
        self._callbacks = []

    def done(self):
        return self._done

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def set_result(self, result):
        self._result = result
        self._done = True
        callbacks = tuple(self._callbacks)
        self._callbacks = []
        for callback in callbacks:
            callback(self)


class FakeGoalHandle:
    """Action goal handle with observable cancellation."""

    def __init__(self, result_future, accepted=True):
        self.accepted = accepted
        self.result_future = result_future
        self.cancel_count = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        return ManualFuture()


class FakeActionClient:
    """Action client returning one configured send future."""

    def __init__(self, send_future):
        self.send_future = send_future
        self.feedback_callback = None

    def send_goal_async(self, _goal, feedback_callback=None):
        self.feedback_callback = feedback_callback
        return self.send_future


class ExampleBoundedAction(BoundedActionClientBehaviour):
    """Minimal concrete action used to exercise the base lifecycle."""

    def __init__(self, client, clock, goal_timeout=2.0, result_timeout=3.0):
        super().__init__(
            "ExampleAction",
            goal_response_timeout_sec=goal_timeout,
            result_timeout_sec=result_timeout,
            monotonic_clock=clock,
        )
        self._client = client

    def _init_client(self):
        self.initialized = True
        return True

    def _build_goal(self):
        return object()


class FakeBlackboard(SimpleNamespace):
    """Blackboard test double with the py_trees existence API."""

    def exists(self, key):
        return hasattr(self, key)


def close_surface_command(
    request_id="00000000-0000-4000-8000-000000000001",
):
    return MoveCloseToSurfaceCommand(
        CommandID.MOVE_CLOSE_TO_SURFACE,
        stamp=Time(),
        target_surface_distance_m=0.05,
        aligned_preapproach_distance_m=0.20,
        request_id=request_id,
    )


def test_action_hierarchy_separates_robot_commands_and_typed_workflows():
    assert issubclass(
        RobotCommandActionBehaviour,
        BoundedActionClientBehaviour,
    )
    assert issubclass(MoveCommandAction, RobotCommandActionBehaviour)
    assert issubclass(WorkflowActionBehaviour, BoundedActionClientBehaviour)
    assert issubclass(
        ManipulatorMoveCloseToSurfaceAction,
        WorkflowActionBehaviour,
    )
    assert not issubclass(
        ManipulatorMoveCloseToSurfaceAction,
        MoveCommandAction,
    )


@pytest.mark.parametrize("timeout", [None, 0.0, -1.0, float("inf")])
def test_bounded_action_rejects_unbounded_or_invalid_result_timeouts(timeout):
    with pytest.raises((TypeError, ValueError)):
        ExampleBoundedAction(
            FakeActionClient(ManualFuture()),
            ManualClock(),
            result_timeout=timeout,
        )


def test_goal_response_timeout_cancels_a_goal_accepted_late():
    clock = ManualClock()
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future)
    action = ExampleBoundedAction(
        FakeActionClient(send_future),
        clock,
    )
    action.initialise()

    assert action.update() is Status.RUNNING
    clock.now = 2.0
    assert action.update() is Status.FAILURE
    assert "goal response timed out" in action.feedback_message.lower()

    send_future.set_result(handle)
    assert handle.cancel_count == 1


def test_result_timeout_requests_public_goal_cancellation():
    clock = ManualClock()
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future)
    send_future.set_result(handle)
    action = ExampleBoundedAction(
        FakeActionClient(send_future),
        clock,
    )
    action.initialise()

    assert action.update() is Status.RUNNING
    assert action.update() is Status.RUNNING
    clock.now = 3.0
    assert action.update() is Status.FAILURE
    assert "result timed out" in action.feedback_message.lower()
    assert handle.cancel_count == 1


def test_close_surface_leaf_preserves_feedback_and_correlated_result_detail():
    clock = ManualClock()
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future)
    client = FakeActionClient(send_future)
    command = close_surface_command()
    action = ManipulatorMoveCloseToSurfaceAction(monotonic_clock=clock)
    action.blackboard = FakeBlackboard(last_command=command)
    action._client = client
    action.initialized = True
    action.initialise()

    assert action.update() is Status.RUNNING
    sent_goal = client.send_future
    assert sent_goal is send_future
    send_future.set_result(handle)
    assert action.update() is Status.RUNNING

    client.feedback_callback(
        SimpleNamespace(
            feedback=SimpleNamespace(
                phase="moving",
                detail="Approach step 2",
            )
        )
    )
    assert action.update() is Status.RUNNING
    assert action.feedback_message == "Approach step 2"

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(
                success=True,
                detail="Surface stand-off reached",
            )
        )
    )
    assert action.update() is Status.SUCCESS
    assert action.feedback_message == "Surface stand-off reached"
    assert action.blackboard.command_failure_request_id == command.request_id
    assert action.blackboard.command_failure_detail == ""


def test_close_surface_leaf_correlates_action_failure_detail():
    clock = ManualClock()
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future)
    send_future.set_result(handle)
    command = close_surface_command(
        "00000000-0000-4000-8000-000000000002"
    )
    action = ManipulatorMoveCloseToSurfaceAction(monotonic_clock=clock)
    action.blackboard = FakeBlackboard(last_command=command)
    action._client = FakeActionClient(send_future)
    action.initialized = True
    action.initialise()

    assert action.update() is Status.RUNNING
    assert action.update() is Status.RUNNING
    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(
                success=False,
                detail="Force data became stale",
            )
        )
    )

    assert action.update() is Status.FAILURE
    assert action.blackboard.command_failure_request_id == command.request_id
    assert action.blackboard.command_failure_detail == "Force data became stale"
