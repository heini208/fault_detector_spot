"""Tests for the shared bounded behavior-tree action lifecycle."""

from types import SimpleNamespace

import pytest
from fault_detector_spot.application.behaviour_tree.behaviours.spot_action import (
    BoundedActionClientBehaviour,
)
from py_trees.common import Status


class ManualClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class ManualFuture:
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
    def __init__(self, send_future):
        self.send_future = send_future

    def send_goal_async(self, _goal, feedback_callback=None):
        return self.send_future


class ExampleBoundedAction(BoundedActionClientBehaviour):
    def __init__(
        self,
        client,
        clock,
        goal_timeout=2.0,
        result_timeout=3.0,
    ):
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


@pytest.mark.parametrize("timeout", [None, 0.0, -1.0, float("inf")])
def test_bounded_action_rejects_unbounded_or_invalid_result_timeouts(
    timeout,
):
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
