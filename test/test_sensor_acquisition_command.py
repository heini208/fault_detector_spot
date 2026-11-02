"""Focused checks for recordable sensor acquisition commands."""

from threading import RLock
from types import SimpleNamespace

import pytest

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
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeNode:
    def __init__(self):
        self.timers = []

    def create_subscription(self, *args):
        return object()

    def create_timer(self, period, callback, **kwargs):
        timer = FakeTimer(callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
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

    def add_status_listener(self, listener):
        self.listeners.append(listener)

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
        self.start_error = None
        self.stop_error = None

    def add_state_listener(self, listener):
        self.listeners.append(listener)

    def snapshot(self):
        return self.state

    def set_state(self, status, *, stopped, detail="Acquisition update"):
        with self.execution_lock:
            self.state = SensorAcquisitionState(status, detail=detail)
            self.recording_stopped = stopped
            for listener in self.listeners:
                listener(self.state)

    def start(self, request):
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
        assert [status.state for status in env.controller.statuses] == [
            CommandControllerState.RUNNING,
        ]
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
