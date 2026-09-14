"""Verify native Spot ArmStopCommand service execution."""

from types import SimpleNamespace

import fault_detector_spot.manipulation.arm_movement_executor as executor_module
from fault_detector_spot.manipulation.arm_movement_executor import (
    ArmMovementExecutor,
)
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome,
)


class Future:

    def __init__(self, response=None, done=True):
        self._response = response
        self._done = done
        self.cancel_count = 0

    def done(self):
        return self._done

    def result(self):
        return self._response

    def cancel(self):
        self.cancel_count += 1


class ServiceClient:

    def __init__(self, future, ready=True):
        self.future = future
        self.ready = ready
        self.requests = []

    def wait_for_service(self, timeout_sec=0.0):
        assert timeout_sec == 0.0
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return self.future


def test_arm_stop_request_converts_native_arm_stop_command(monkeypatch):
    captured = {}

    def capture(source, _target):
        captured["source"] = source

    monkeypatch.setattr(executor_module, "convert", capture)
    executor = ArmMovementExecutor(object())

    executor._build_arm_stop_request()

    command = captured["source"]
    assert command.HasField("synchronized_command")
    arm = command.synchronized_command.arm_command
    assert arm.HasField("arm_stop_command")


def test_arm_stop_service_success_completes_without_action_feedback():
    future = Future(SimpleNamespace(success=True, message="accepted"))
    client = ServiceClient(future)
    executor = ArmMovementExecutor(
        object(),
        arm_stop_service_client=client,
    )

    started = executor._guard_start_arm_stop()
    finished = executor._guard_poll_arm_stop()

    assert started.outcome is ArmMovementOutcome.RUNNING
    assert finished.outcome is ArmMovementOutcome.SUCCESS
    assert finished.detail == "accepted"
    assert len(client.requests) == 1
    assert executor._arm_stop_service_future is None


def test_arm_stop_service_rejection_is_reported_immediately():
    future = Future(SimpleNamespace(success=False, message="rejected"))
    client = ServiceClient(future)
    executor = ArmMovementExecutor(
        object(),
        arm_stop_service_client=client,
    )

    assert executor._guard_start_arm_stop().outcome is ArmMovementOutcome.RUNNING
    finished = executor._guard_poll_arm_stop()

    assert finished.outcome is ArmMovementOutcome.MOTION_FAILED
    assert finished.detail == "rejected"
