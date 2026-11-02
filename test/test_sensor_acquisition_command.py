"""Focused checks for recordable sensor acquisition commands."""

from threading import RLock
from types import SimpleNamespace
from uuid import UUID

import pytest
from rclpy.clock import ClockType

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    InspectionSelection,
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    CommandExecutionStatus,
)
from fault_detector_spot.application.coordinators.sensor_acquisition_command_handler import (
    SensorAcquisitionCommandHandler,
)
from fault_detector_spot.application.coordinators.sensor_acquisition_coordinator import (
    SensorAcquisitionState,
    SensorAcquisitionStatus,
)


class FakeTimer:
    def __init__(self, callback, clock):
        self.callback = callback
        self.clock = clock
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeNode:
    def __init__(self):
        self.timers = []

    def create_subscription(self, *args):
        return object()

    def create_timer(self, period, callback, **kwargs):
        timer = FakeTimer(callback, kwargs["clock"])
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        pass

    def destroy_subscription(self, subscription):
        pass

    def drain(self):
        while self.timers:
            timer = self.timers.pop(0)
            if not timer.cancelled:
                timer.callback()


class FakeController:
    def __init__(self, request_id):
        self.active_request_id = request_id
        self.statuses = []
        self.listeners = []
        self.owners = {}

    def bind_execution_owner(self, request_id, instance_id):
        self.owners[request_id] = instance_id

    def add_status_listener(self, listener):
        self.listeners.append(listener)

    def remove_status_listener(self, listener):
        self.listeners.remove(listener)

    def handle_execution_status(self, status):
        self.statuses.append(status)
        return True

    def request_stop(self):
        status = SimpleNamespace(
            request_id=self.active_request_id,
            state=CommandControllerState.STOP_REQUESTED,
        )
        for listener in self.listeners:
            listener(status)


class FakeCoordinator:
    def __init__(self, start_state, stopped):
        self.execution_lock = RLock()
        self.callback_group = object()
        self.listeners = []
        self.request = None
        self.start_state = start_state
        self.state = start_state
        self.recording_stopped = stopped
        self.stop_calls = []
        self.start_calls = []
        self.start_error = None
        self.stop_error = None

    def add_state_listener(self, listener):
        self.listeners.append(listener)

    def remove_state_listener(self, listener):
        self.listeners.remove(listener)

    def snapshot(self):
        return self.state

    def set_state(self, status, *, stopped, detail="Acquisition update"):
        with self.execution_lock:
            self.state = SensorAcquisitionState(status, detail=detail)
            self.recording_stopped = stopped
            for listener in self.listeners:
                listener(self.state)

    def start(self, request):
        self.start_calls.append(request)
        self.request = request
        if self.start_error is not None:
            raise self.start_error
        self.set_state(self.start_state.status, stopped=self.recording_stopped,
                       detail=self.start_state.detail)
        return self.state

    def stop(self, *, cancelled=False):
        self.stop_calls.append(cancelled)
        if self.stop_error is not None:
            raise self.stop_error
        self.set_state(
            SensorAcquisitionStatus.IDLE if self.recording_stopped
            else SensorAcquisitionStatus.STOPPING,
            stopped=self.recording_stopped,
        )
        return self.state


def rig(command_id=CommandID.START_SENSOR_RECORDING, *, stopped=False):
    request = CommandRequest.create(
        command=SemanticCommand(command_id, inspection=InspectionSelection()),
        client_id="test",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )
    node = FakeNode()
    controller = FakeController(request.request_id)
    coordinator = FakeCoordinator(SensorAcquisitionState(
        SensorAcquisitionStatus.IDLE if stopped else SensorAcquisitionStatus.STARTING,
        detail="Sensor head is offline; acquisition skipped" if stopped else "Starting",
    ), stopped)
    handler = SensorAcquisitionCommandHandler(node, controller, coordinator)
    return SimpleNamespace(request=request, node=node, controller=controller,
                           coordinator=coordinator, handler=handler)


@pytest.mark.parametrize("offline", [False, True])
def test_start_command_waits_for_recording_or_completes_offline_skip(offline):
    env = rig(stopped=offline)
    assert env.handler.dispatch(env.request)
    env.node.drain()

    assert env.coordinator.request.object_id == ""
    assert env.coordinator.request.object_pose_execution is not None
    if not offline:
        assert all(status.state is CommandControllerState.RUNNING
                   for status in env.controller.statuses)
        env.coordinator.set_state(SensorAcquisitionStatus.RECORDING, stopped=False)
        env.node.drain()
    assert env.controller.statuses[-1].state is CommandControllerState.SUCCEEDED
    assert env.controller.statuses[-1].execution_idle
    assert env.handler._pending_request is None


@pytest.mark.parametrize("when", ["before_timer", "during_acknowledgement"])
def test_startup_timer_cannot_start_acquisition_after_stop_intent(when):
    env = rig(stopped=True)
    assert env.handler.dispatch(env.request)
    if when == "before_timer":
        env.controller.request_stop()
    else:
        report = env.controller.handle_execution_status

        def request_stop_after_running(status):
            report(status)
            if status.state is CommandControllerState.RUNNING:
                env.controller.request_stop()

        env.controller.handle_execution_status = request_stop_after_running
    env.node.drain()

    assert env.coordinator.request is None
    assert env.coordinator.stop_calls == [True]
    assert env.controller.statuses[-1].state is CommandControllerState.STOP_CONFIRMED
    assert env.controller.statuses[-1].execution_idle


def test_requested_stop_retains_owner_until_coordinator_proves_quiescence():
    env = rig()
    env.handler.dispatch(env.request)
    env.node.drain()
    env.controller.request_stop()
    # The listener must not acquire the coordinator lock under the controller lock.
    assert not env.coordinator.stop_calls
    env.node.drain()
    assert env.coordinator.stop_calls == [True]
    assert env.controller.statuses[-1].state is CommandControllerState.STOP_REQUESTED
    assert not env.controller.statuses[-1].execution_idle

    env.coordinator.set_state(SensorAcquisitionStatus.FAILED, stopped=False,
                              detail="Stop acknowledgement timed out")
    env.node.drain()
    assert env.controller.statuses[-1].state is CommandControllerState.STOP_UNCONFIRMED
    assert not env.controller.statuses[-1].execution_idle
    assert env.handler._pending_request is env.request

    env.coordinator.set_state(SensorAcquisitionStatus.IDLE, stopped=True)
    env.node.drain()
    assert env.controller.statuses[-1].state is CommandControllerState.STOP_CONFIRMED
    assert env.controller.statuses[-1].execution_idle
    assert env.handler._pending_request is None


def test_stop_command_requires_quiescence_even_when_snapshot_says_idle():
    env = rig(CommandID.STOP_SENSOR_RECORDING)
    env.handler.dispatch(env.request)
    env.node.drain()
    env.coordinator.set_state(SensorAcquisitionStatus.IDLE, stopped=False)
    env.node.drain()
    assert env.controller.statuses[-1].state is CommandControllerState.UNKNOWN
    assert not env.controller.statuses[-1].execution_idle
    assert env.handler._pending_request is env.request

    env.coordinator.set_state(SensorAcquisitionStatus.IDLE, stopped=True)
    env.node.drain()
    assert env.controller.statuses[-1].state is CommandControllerState.SUCCEEDED
    assert env.controller.statuses[-1].execution_idle


@pytest.mark.parametrize("stopped", [False, True])
def test_submission_exception_cannot_claim_completion_without_quiescence(stopped):
    env = rig(stopped=stopped)
    env.coordinator.start_error = RuntimeError("Start submission failed")
    env.handler.dispatch(env.request)
    env.node.drain()

    assert env.controller.statuses[-1].state is (
        CommandControllerState.FAILED if stopped else CommandControllerState.UNKNOWN
    )
    assert env.controller.statuses[-1].execution_idle is stopped
    if not stopped:
        assert env.handler._pending_request is env.request
        env.coordinator.set_state(SensorAcquisitionStatus.IDLE, stopped=True)
        env.node.drain()
        assert env.controller.statuses[-1].state is CommandControllerState.FAILED
        assert env.controller.statuses[-1].detail == "Start submission failed"
        assert env.controller.statuses[-1].execution_idle


def test_stop_submission_exception_keeps_the_owner_unresolved():
    env = rig()
    env.handler.dispatch(env.request)
    env.node.drain()
    env.coordinator.stop_error = RuntimeError("Stop unavailable")
    env.controller.request_stop()
    env.node.drain()

    assert env.controller.statuses[-1].state is CommandControllerState.STOP_UNCONFIRMED
    assert not env.controller.statuses[-1].execution_idle
    assert env.handler._pending_request is env.request


@pytest.mark.parametrize("defer_notifications", [False, True])
def test_failed_start_remains_failed_when_cleanup_later_becomes_idle(defer_notifications):
    env = rig()
    env.handler.dispatch(env.request)
    env.node.drain()
    env.coordinator.set_state(SensorAcquisitionStatus.FAILED, stopped=False,
                              detail="Sensor startup failed")
    if not defer_notifications:
        env.node.drain()
        assert env.controller.statuses[-1].state is CommandControllerState.UNKNOWN
        assert not env.controller.statuses[-1].execution_idle

    env.coordinator.set_state(SensorAcquisitionStatus.IDLE, stopped=True)
    env.node.drain()
    assert env.controller.statuses[-1].state is CommandControllerState.FAILED
    assert env.controller.statuses[-1].detail == "Sensor startup failed"
    assert env.controller.statuses[-1].execution_idle


def test_coordinator_callback_reports_only_after_releasing_owner_locks():
    env = rig()
    env.handler.dispatch(env.request)
    env.node.drain()
    report = env.controller.handle_execution_status

    def assert_unlocked(status):
        assert not env.coordinator.execution_lock._is_owned()
        assert not env.handler._lock._is_owned()
        return report(status)

    env.controller.handle_execution_status = assert_unlocked
    env.handler.query_status(env.request.request_id)
    env.node.drain()
    env.coordinator.set_state(SensorAcquisitionStatus.RECORDING, stopped=False)
    assert env.controller.statuses[-1].state is CommandControllerState.RUNNING
    env.node.drain()
    assert env.controller.statuses[-1].state is CommandControllerState.SUCCEEDED


def test_old_state_callback_cannot_complete_the_next_request_before_execution():
    env = rig()
    env.handler.dispatch(env.request)
    env.node.drain()
    next_request = CommandRequest.create(
        command=env.request.command,
        client_id="test",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )
    report = env.controller.handle_execution_status

    def dispatch_next_after_success(status):
        report(status)
        if (status.request_id == env.request.request_id
                and status.state is CommandControllerState.SUCCEEDED):
            env.controller.active_request_id = next_request.request_id
            env.coordinator.start_error = RuntimeError("Acquisition is already active")
            env.handler.dispatch(next_request)

    env.controller.handle_execution_status = dispatch_next_after_success
    # Both notifications precede the next request's startup timer. The second
    # notification must not mistake the previous recording for a new success.
    env.coordinator.set_state(SensorAcquisitionStatus.RECORDING, stopped=False)
    env.coordinator.set_state(SensorAcquisitionStatus.RECORDING, stopped=False)
    env.node.drain()

    next_states = [status for status in env.controller.statuses
                   if status.request_id == next_request.request_id]
    assert [status.state for status in next_states] == [
        CommandControllerState.RUNNING, CommandControllerState.UNKNOWN,
    ]
    assert not next_states[-1].execution_idle
    assert env.handler._pending_request is next_request


@pytest.mark.parametrize("wire_first", [False, True])
def test_controller_waits_for_both_local_acquisition_and_emergency_stop_proofs(wire_first):
    env = rig()
    controller = CommandController()
    node = FakeNode()
    coordinator = FakeCoordinator(env.coordinator.state, stopped=False)
    handler = SensorAcquisitionCommandHandler(node, controller, coordinator)
    dispatched = []
    statuses = []

    def dispatch(request):
        if not handler.dispatch(request):
            controller.bind_execution_owner(request.request_id, "test-bt-instance")
            dispatched.append(request)

    controller.configure_dispatch(dispatch)
    controller.add_status_listener(statuses.append)
    controller.submit(env.request)
    node.drain()
    stop_id = controller.cancel_all()
    controller.poll()
    next_request = CommandRequest.create(
        command=SemanticCommand(CommandID.STAND_UP),
        client_id="test",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )
    controller.submit(next_request)
    node.drain()
    assert coordinator.stop_calls == [True]
    assert controller.active_request_id == env.request.request_id

    def confirm_wire_stop():
        controller.handle_execution_status(CommandExecutionStatus(
            request_id=stop_id,
            state=CommandControllerState.STOP_CONFIRMED,
            execution_idle=True,
            execution_instance_id="test-bt-instance",
            status_sequence=1,
        ))

    def confirm_local_stop():
        coordinator.set_state(SensorAcquisitionStatus.IDLE, stopped=True)
        node.drain()

    first, second = ((confirm_wire_stop, confirm_local_stop) if wire_first
                     else (confirm_local_stop, confirm_wire_stop))
    first()
    assert [request.request_id for request in dispatched] == [stop_id]
    assert controller.queued_request_ids == (next_request.request_id,)
    second()

    assert [request.request_id for request in dispatched] == [stop_id, next_request.request_id]
    assert controller.active_request_id == next_request.request_id
    stopped = [status for status in statuses
               if status.request_id == env.request.request_id
               and status.state is CommandControllerState.STOP_CONFIRMED]
    assert len(stopped) == 1
    assert stopped[0].execution_idle


@pytest.mark.parametrize("command_id", [CommandID.START_SENSOR_RECORDING,
                                       CommandID.STOP_SENSOR_RECORDING])
def test_local_queries_report_fresh_same_owner_status_without_repeating_work(command_id):
    env = rig(command_id)
    env.handler.dispatch(env.request)
    assert env.controller.owners == {env.request.request_id: env.handler.execution_instance_id}
    assert not env.coordinator.start_calls and not env.coordinator.stop_calls
    assert all(timer.clock.clock_type == ClockType.STEADY_TIME for timer in env.node.timers)
    env.node.drain()
    start_count, stop_count = len(env.coordinator.start_calls), len(env.coordinator.stop_calls)
    first = env.controller.statuses[-1]
    env.coordinator.state = SensorAcquisitionState(
        env.coordinator.state.status, detail="Waiting for sensor acknowledgement",
    )

    for _ in range(2):
        env.handler.query_status(env.request.request_id)
        assert env.controller.statuses[-1] is first
        env.node.drain()
        current = env.controller.statuses[-1]
        assert current.request_id == first.request_id
        assert current.state is CommandControllerState.RUNNING
        assert current.detail == env.coordinator.snapshot().detail
        assert not current.execution_idle
        assert current.execution_instance_id == first.execution_instance_id
        assert current.status_sequence > first.status_sequence
        first = current

    assert len(env.coordinator.start_calls) == start_count
    assert len(env.coordinator.stop_calls) == stop_count
    assert len(env.coordinator.start_calls) + len(env.coordinator.stop_calls) == 1


def test_status_query_can_acknowledge_scheduled_work_without_starting_it():
    env = rig()
    env.handler.dispatch(env.request)
    startup = env.node.timers.pop()
    env.handler.query_status(env.request.request_id)
    env.node.drain()
    status = env.controller.statuses[-1]
    assert status.state is CommandControllerState.RUNNING
    assert status.detail == "Sensor acquisition command scheduled; awaiting execution"
    assert not status.execution_idle
    assert not env.coordinator.start_calls and not env.coordinator.stop_calls
    startup.callback()
    env.node.drain()
    assert len(env.coordinator.start_calls) == 1


def test_repeated_queries_and_stop_intent_do_not_repeat_cancellation_or_hide_failure():
    env = rig()
    env.handler.dispatch(env.request)
    env.node.drain()
    env.controller.request_stop()
    env.controller.request_stop()
    env.node.drain()
    assert env.coordinator.stop_calls == [True]
    env.coordinator.set_state(SensorAcquisitionStatus.FAILED, stopped=False,
                              detail="Timed out waiting for the sensor to stop")
    env.node.drain()
    previous = env.controller.statuses[-1]
    for _ in range(3):
        env.controller.request_stop()
        env.handler.query_status(env.request.request_id)
        env.node.drain()
        current = env.controller.statuses[-1]
        assert current.state is CommandControllerState.STOP_UNCONFIRMED
        assert current.detail == previous.detail
        assert current.status_sequence > previous.status_sequence
        assert not current.execution_idle
        previous = current
    assert env.coordinator.stop_calls == [True]
    assert env.handler._pending_request is env.request


def test_terminal_query_replays_original_evidence_with_fresh_sequence():
    env = rig(stopped=True)
    env.handler.dispatch(env.request)
    env.node.drain()
    completed = env.controller.statuses[-1]
    assert completed.state is CommandControllerState.SUCCEEDED
    assert env.handler._pending_request is None
    env.coordinator.set_state(SensorAcquisitionStatus.RECORDING, stopped=False)
    env.node.drain()
    env.handler.query_status(env.request.request_id)
    env.node.drain()
    replay = env.controller.statuses[-1]
    assert replay.request_id == completed.request_id
    assert replay.execution_instance_id == completed.execution_instance_id
    assert replay.status_sequence > completed.status_sequence
    assert replay.state is completed.state and replay.execution_idle
    assert replay.detail == completed.detail
    assert len(env.coordinator.start_calls) == 1
    assert not env.coordinator.stop_calls


def test_new_handler_has_distinct_identity_and_no_old_terminal_evidence():
    first = rig(stopped=True)
    first.handler.dispatch(first.request)
    first.node.drain()
    restarted = rig()
    assert UUID(first.handler.execution_instance_id) != UUID(restarted.handler.execution_instance_id)
    restarted.handler.query_status(first.request.request_id)
    restarted.node.drain()
    status = restarted.controller.statuses[-1]
    assert status.request_id == first.request.request_id
    assert status.execution_instance_id == restarted.handler.execution_instance_id
    assert status.state is CommandControllerState.UNKNOWN
    assert not status.execution_idle
    assert not restarted.coordinator.start_calls and not restarted.coordinator.stop_calls


def test_local_query_is_coalesced_and_close_suppresses_pending_feedback():
    env = rig()
    env.handler.dispatch(env.request)
    env.node.drain()
    previous = tuple(env.controller.statuses)
    for _ in range(3):
        env.handler.query_status(env.request.request_id)
    assert len(env.node.timers) == 1
    assert env.node.timers[0].clock.clock_type == ClockType.STEADY_TIME
    env.handler.close()
    env.node.drain()
    env.handler.query_status(env.request.request_id)
    env.node.drain()
    assert tuple(env.controller.statuses) == previous
    assert not env.node.timers
    assert not env.coordinator.stop_calls


def test_superseded_query_snapshot_cannot_emit_a_newer_running_receipt(monkeypatch):
    env = rig()
    env.handler.dispatch(env.request)
    env.node.drain()
    finish = env.handler._finish
    interrupted = False

    def fail_before_reporting_snapshot(request_id, state, detail, **kwargs):
        nonlocal interrupted
        if not interrupted and state is CommandControllerState.RUNNING:
            interrupted = True
            env.coordinator.set_state(SensorAcquisitionStatus.FAILED, stopped=False,
                                      detail="Sensor transition failed")
            env.handler._process_acquisition_state()
        return finish(request_id, state, detail, **kwargs)

    monkeypatch.setattr(env.handler, "_finish", fail_before_reporting_snapshot)
    previous_count = len(env.controller.statuses)
    env.handler.query_status(env.request.request_id)
    env.node.drain()
    reports = env.controller.statuses[previous_count:]
    assert reports
    assert all(status.state is CommandControllerState.UNKNOWN for status in reports)
    assert all(not status.execution_idle for status in reports)
    assert len(env.coordinator.start_calls) == 1
