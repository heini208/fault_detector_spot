"""Tests for machine-readable command request correlation."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import BasicCommand, CommandStatus, ComplexCommand
from py_trees.common import Status

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.sensing.behaviours.buffer_and_status_publisher import (
    BufferStatusPublisher,
)
from fault_detector_spot.sensing.behaviours.command_subscriber import (
    CommandSubscriber,
)
from fault_detector_spot.application.recording.record_manager_node import (
    assign_playback_request_id,
    clear_recorded_request_id,
)
from fault_detector_spot.application.data.request_identity import new_request_id


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=7, nanosec=11))


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_complex_command_request_id_reaches_internal_motion():
    request_id = new_request_id()
    message = ComplexCommand()
    message.command.command_id = CommandID.MOVE_ARM_TO_TAG
    message.command.request_id = request_id
    subscriber = CommandSubscriber()
    subscriber.blackboard = SimpleNamespace(command_buffer=[])
    subscriber.node = SimpleNamespace(get_clock=lambda: FakeClock())

    subscriber.fire_command(message)

    assert len(subscriber.blackboard.command_buffer) == 1
    assert subscriber.blackboard.command_buffer[0].request_id == request_id


def test_structured_status_preserves_request_and_buffer_count():
    request_id = new_request_id()
    command = SimpleNamespace(
        command_id=CommandID.MOVE_ARM_TO_TAG,
        request_id=request_id,
    )
    publisher = BufferStatusPublisher()
    publisher.node = SimpleNamespace(get_clock=lambda: FakeClock())
    publisher.blackboard = SimpleNamespace(
        last_command=command,
        command_tree_status=Status.RUNNING,
    )

    message = publisher.get_structured_status_message(2)

    assert message.request_id == request_id
    assert message.command_id == CommandID.MOVE_ARM_TO_TAG
    assert message.state == CommandStatus.STATE_RUNNING
    assert message.buffered_command_count == 2


def test_terminal_status_is_not_replaced_by_idle_guard_failure():
    request_id = new_request_id()
    command = SimpleNamespace(
        command_id=CommandID.MOVE_ARM_TO_TAG,
        request_id=request_id,
    )
    publisher = BufferStatusPublisher()
    publisher.node = SimpleNamespace(get_clock=lambda: FakeClock())
    publisher.blackboard = SimpleNamespace(
        command_buffer=[],
        last_command=command,
        command_tree_status=Status.SUCCESS,
    )
    publisher.buffer_pub = FakePublisher()
    publisher.status_pub = FakePublisher()
    publisher.structured_status_pub = FakePublisher()

    publisher.update()
    publisher.blackboard.command_tree_status = Status.FAILURE
    publisher.update()

    messages = publisher.structured_status_pub.messages
    assert len(messages) == 1
    assert messages[0].request_id == request_id
    assert messages[0].state == CommandStatus.STATE_SUCCEEDED
    assert len(publisher.status_pub.messages) == 1


def test_recordings_clear_and_playback_regenerates_request_id():
    original_request_id = new_request_id()
    serialized = {
        "command": {
            "command_id": CommandID.MOVE_ARM_TO_TAG,
            "request_id": original_request_id,
        }
    }

    cleared = clear_recorded_request_id(serialized, is_complex=True)
    assert cleared["command"]["request_id"] == ""

    message = BasicCommand()
    message.request_id = original_request_id
    replay_request_id = assign_playback_request_id(message)

    assert replay_request_id != original_request_id
    assert message.request_id == replay_request_id
