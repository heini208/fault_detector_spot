"""Focused tests for BaseMovementExecutor lifecycle."""

from types import SimpleNamespace

from fault_detector_spot.navigation.base_movement_executor import (
    BaseMovementExecutor,
    BaseMovementOutcome,
)


class ManualFuture:

    def __init__(self):
        self._done = False
        self._result = None
        self._callbacks = []

    def done(self):
        return self._done

    def result(self):
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
        self._callbacks.clear()
        for callback in callbacks:
            callback(self)


class FakeGoalHandle:

    def __init__(self, result_future, accepted=True):
        self.accepted = accepted
        self._result_future = result_future
        self.cancel_calls = 0

    def get_result_async(self):
        return self._result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return ManualFuture()


class FakeActionClient:

    def __init__(self, send_future):
        self.send_future = send_future
        self.send_calls = 0

    def wait_for_server(self, timeout_sec=0.0):
        assert timeout_sec == 0.0
        return True

    def send_goal_async(self, _goal):
        self.send_calls += 1
        return self.send_future


def test_stand_uses_executor_lifecycle_until_success():
    send_future = ManualFuture()
    result_future = ManualFuture()
    client = FakeActionClient(send_future)
    executor = BaseMovementExecutor(
        tf_listener=object(),
        action_client=client,
    )
    executor._build_stand_goal = lambda: object()

    update = executor.stand()
    assert update.outcome is BaseMovementOutcome.RUNNING
    assert client.send_calls == 1

    handle = FakeGoalHandle(result_future)
    send_future.set_result(handle)
    update = executor.poll()
    assert update.outcome is BaseMovementOutcome.RUNNING

    result_future.set_result(
        SimpleNamespace(
            result=SimpleNamespace(success=True)
        )
    )
    update = executor.poll()
    assert update.outcome is BaseMovementOutcome.SUCCESS
    assert not executor.active


def test_executor_rejects_second_base_operation_while_active():
    send_future = ManualFuture()
    executor = BaseMovementExecutor(
        tf_listener=object(),
        action_client=FakeActionClient(send_future),
    )
    executor._build_stand_goal = lambda: object()

    assert (
        executor.stand().outcome
        is BaseMovementOutcome.RUNNING
    )
    assert (
        executor.stand().outcome
        is BaseMovementOutcome.BUSY
    )


def test_cancel_requests_goal_cancellation_and_releases_executor():
    send_future = ManualFuture()
    result_future = ManualFuture()
    handle = FakeGoalHandle(result_future)
    send_future.set_result(handle)

    executor = BaseMovementExecutor(
        tf_listener=object(),
        action_client=FakeActionClient(send_future),
    )
    executor._build_stand_goal = lambda: object()

    assert (
        executor.stand().outcome
        is BaseMovementOutcome.RUNNING
    )
    assert (
        executor.poll().outcome
        is BaseMovementOutcome.RUNNING
    )

    executor.cancel()

    assert handle.cancel_calls == 1
    assert not executor.active
