"""Regression tests for hardware-free waits and semantic queue reporting."""

from collections import deque
from types import SimpleNamespace

from fault_detector_msgs.msg import CommandStatus
from py_trees.common import Status
from std_msgs.msg import Bool

from fault_detector_spot.application.behaviour_tree.behaviours.wait_for_duration import (
    WaitForDuration,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    CommandExecutionStatus,
)
from fault_detector_spot.application.recording.record_manager_node import RecordManager


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def __init__(self):
        self.info_messages = []
        self.warning_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def warning(self, message):
        self.warning_messages.append(message)


class RecordManagerHarness:
    handle_playback_status = RecordManager.handle_playback_status
    _dispatch_next_playback_command = RecordManager._dispatch_next_playback_command
    _finish_playback = RecordManager._finish_playback

    def __init__(self):
        self._playback_commands = deque()
        self._playback_request_id = ""
        self._playback_name = ""
        self.command_submission_pub = FakePublisher()
        self.playback_state_pub = FakePublisher()
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


def semantic_request(command_id, wait_time=0.0):
    return CommandRequest.create(
        command=SemanticCommand(
            command_id=command_id,
            wait_time=wait_time,
        ),
        client_id="operator_ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )


def execution_success(request_id):
    return CommandExecutionStatus(
        request_id=request_id,
        state=CommandControllerState.SUCCEEDED,
        buffered_command_count=0,
    )


def playback_success(request_id, buffered_command_count):
    message = CommandStatus()
    message.request_id = request_id
    message.command_id = CommandID.WAIT_TIME.value
    message.state = CommandStatus.STATE_SUCCEEDED
    message.buffered_command_count = buffered_command_count
    return message


def test_wait_deadline_survives_reinitialization_of_same_request():
    clock = FakeClock()
    wait = WaitForDuration(monotonic_clock=clock)
    wait.blackboard = SimpleNamespace(
        last_command=SimpleNamespace(
            request_id="00000000-0000-4000-8000-000000000001",
            duration=2.0,
        )
    )

    wait.initialise()
    assert wait.update() == Status.RUNNING

    clock.advance(1.0)
    wait.initialise()
    clock.advance(1.1)

    assert wait.update() == Status.SUCCESS


def test_second_wait_request_gets_its_own_deadline():
    clock = FakeClock()
    wait = WaitForDuration(monotonic_clock=clock)
    wait.blackboard = SimpleNamespace(
        last_command=SimpleNamespace(
            request_id="00000000-0000-4000-8000-000000000001",
            duration=1.0,
        )
    )

    wait.initialise()
    clock.advance(1.1)
    assert wait.update() == Status.SUCCESS

    wait.blackboard.last_command = SimpleNamespace(
        request_id="00000000-0000-4000-8000-000000000002",
        duration=1.0,
    )
    wait.initialise()

    assert wait.update() == Status.RUNNING
    clock.advance(1.1)
    assert wait.update() == Status.SUCCESS


def test_controller_status_reports_semantic_queue_depth():
    dispatched = []
    statuses = []
    controller = CommandController(dispatch_request=dispatched.append)
    controller.add_status_listener(statuses.append)
    first = semantic_request(CommandID.WAIT_TIME, 1.0)
    second = semantic_request(CommandID.WAIT_TIME, 1.0)
    third = semantic_request(CommandID.WAIT_TIME, 1.0)

    controller.submit(first)
    controller.submit(second)
    controller.submit(third)

    assert controller.active_request_id == first.request_id
    assert controller.queued_request_ids == (
        second.request_id,
        third.request_id,
    )
    assert statuses[-1].request_id == third.request_id
    assert statuses[-1].state is CommandControllerState.QUEUED
    assert statuses[-1].buffered_command_count == 2

    controller.handle_execution_status(execution_success(first.request_id))

    first_terminal = next(
        status
        for status in statuses
        if status.request_id == first.request_id
        and status.state is CommandControllerState.SUCCEEDED
    )
    assert first_terminal.buffered_command_count == 2
    assert statuses[-1].request_id == second.request_id
    assert statuses[-1].state is CommandControllerState.DISPATCHED
    assert statuses[-1].buffered_command_count == 1


def test_playback_advances_on_controller_success_even_with_queue_depth():
    manager = RecordManagerHarness()
    manager._playback_commands = deque([
        SemanticCommand(
            command_id=CommandID.WAIT_TIME,
            wait_time=1.0,
        ),
        SemanticCommand(
            command_id=CommandID.WAIT_TIME,
            wait_time=1.0,
        ),
    ])
    manager._playback_name = "two_waits"

    manager._dispatch_next_playback_command()
    first_request_id = manager._playback_request_id

    assert manager.handle_playback_status(
        playback_success(first_request_id, 2)
    ) is True
    assert len(manager.command_submission_pub.messages) == 2
    second_request_id = manager._playback_request_id
    assert second_request_id != first_request_id

    assert manager.handle_playback_status(
        playback_success(second_request_id, 1)
    ) is True
    assert manager._playback_request_id == ""
    assert manager._playback_commands == deque()
    assert manager.playback_state_pub.messages == [Bool(data=False)]
