"""Tests for semantic command recording and sequential playback."""

from collections import deque

import pytest
from fault_detector_msgs.msg import CommandStatus
from std_msgs.msg import Bool

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    CommandRequest,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    CommandQuaternion,
    CommandVector3,
    InspectionSelection,
    SemanticCommand,
    SemanticTag,
    StampedPose,
)
from fault_detector_spot.application.recording.record_manager_node import RecordManager
from fault_detector_spot.application.recording.semantic_command_codec import (
    deserialize_recorded_command,
    deserialize_recording,
    serialize_recorded_command,
)
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_to_message,
    semantic_command_request_from_message,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def __init__(self):
        self.info_messages = []
        self.warning_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def warning(self, message):
        self.warning_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


class RecordManagerHarness:
    capture_request = RecordManager.capture_request
    handle_playback_status = RecordManager.handle_playback_status
    _dispatch_next_playback_command = RecordManager._dispatch_next_playback_command
    _finish_playback = RecordManager._finish_playback

    def __init__(self):
        self.recording = False
        self.temp_data = []
        self._recorded_request_ids = set()
        self._playback_commands = deque()
        self._playback_request_id = ""
        self._playback_name = ""
        self.command_submission_pub = FakePublisher()
        self.playback_state_pub = FakePublisher()
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


def make_request(
    command_id,
    origin=CommandOrigin.OPERATIONAL,
    policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
):
    return CommandRequest.create(
        command=SemanticCommand(command_id=command_id),
        client_id="operator_ui",
        origin=origin,
        recording_policy=policy,
    )


def terminal_status(request_id, state, command_id=""):
    message = CommandStatus()
    message.request_id = request_id
    message.command_id = command_id
    message.state = state
    return message


def test_only_included_accepted_requests_are_recorded():
    manager = RecordManagerHarness()
    manager.recording = True
    included = make_request(CommandID.STAND_UP)
    excluded = make_request(
        CommandID.STOW_ARM,
        origin=CommandOrigin.PROBE_SETUP,
        policy=RecordingPolicy.EXCLUDE,
    )

    assert manager.capture_request(command_request_to_message(included)) is True
    assert manager.capture_request(command_request_to_message(excluded)) is False
    assert manager.capture_request(command_request_to_message(included)) is False
    assert len(manager.temp_data) == 1
    assert manager.temp_data[0]["command_id"] == CommandID.STAND_UP.value
    assert "command" not in manager.temp_data[0]


def test_recorded_wait_is_preserved_as_an_explicit_command():
    command = SemanticCommand(
        command_id=CommandID.WAIT_TIME,
        wait_time=4.5,
    )

    restored = deserialize_recorded_command(serialize_recorded_command(command))

    assert restored == command


def test_recording_round_trip_preserves_full_semantic_command():
    pose = StampedPose(
        frame_id="odom",
        stamp_sec=7,
        stamp_nanosec=11,
        position=CommandVector3(x=0.1, y=-0.2, z=0.3),
        orientation=CommandQuaternion(x=0.0, y=0.0, z=0.4, w=0.9),
    )
    command = SemanticCommand(
        command_id=CommandID.MOVE_ARM_TO_TAG,
        tag=SemanticTag(id=4, pose=pose),
        offset=pose,
        orientation_mode="relative_to_tag",
        wait_time=1.25,
        map_name="factory",
        waypoint_name="motor_a",
        inspection=InspectionSelection(
            object_id="motor_a",
            routine_id="magnetic_scan",
            probe_point_id="bearing_1",
        ),
    )

    data = serialize_recorded_command(command)
    restored = deserialize_recorded_command(data)

    assert data["command_id"] == CommandID.MOVE_ARM_TO_TAG.value
    assert data["tag"]["id"] == 4
    assert "request_id" not in data
    assert restored == command


def test_legacy_complex_command_recording_is_rejected():
    with pytest.raises(ValueError, match="missing field: tag"):
        deserialize_recorded_command(
            {
                "command": {
                    "command_id": CommandID.STAND_UP.value,
                }
            }
        )


def test_topic_based_recording_document_is_rejected():
    with pytest.raises(ValueError, match="must be an object"):
        deserialize_recording([
            {"topic": "basic", "timestamp": 9.2, "data": {}}
        ])


def test_playback_dispatches_one_command_after_each_success():
    manager = RecordManagerHarness()
    first = SemanticCommand(command_id=CommandID.STAND_UP)
    second = SemanticCommand(command_id=CommandID.STOW_ARM)
    manager._playback_commands = deque([first, second])
    manager._playback_name = "inspection"

    manager._dispatch_next_playback_command()

    assert len(manager.command_submission_pub.messages) == 1
    first_request = semantic_command_request_from_message(
        manager.command_submission_pub.messages[0]
    )
    assert first_request.origin is CommandOrigin.PLAYBACK
    assert (
        first_request.recording_policy
        is RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE
    )
    assert first_request.command.command_id is CommandID.STAND_UP
    assert manager.handle_playback_status(terminal_status(
        first_request.request_id,
        CommandStatus.STATE_RUNNING,
    )) is True
    assert len(manager.command_submission_pub.messages) == 1

    manager.handle_playback_status(terminal_status(
        first_request.request_id,
        CommandStatus.STATE_SUCCEEDED,
    ))

    assert len(manager.command_submission_pub.messages) == 2
    second_request = semantic_command_request_from_message(
        manager.command_submission_pub.messages[1]
    )
    assert second_request.request_id != first_request.request_id
    assert second_request.command.command_id is CommandID.STOW_ARM

    manager.handle_playback_status(terminal_status(
        second_request.request_id,
        CommandStatus.STATE_SUCCEEDED,
    ))

    assert manager._playback_request_id == ""
    assert manager._playback_commands == deque()
    assert manager.playback_state_pub.messages == [Bool(data=False)]


def test_playback_stops_and_discards_remaining_commands_on_failure():
    manager = RecordManagerHarness()
    first = SemanticCommand(command_id=CommandID.MOVE_TO_WAYPOINT)
    second = SemanticCommand(command_id=CommandID.STOW_ARM)
    manager._playback_commands = deque([first, second])
    manager._playback_name = "inspection"
    manager._dispatch_next_playback_command()
    request = semantic_command_request_from_message(
        manager.command_submission_pub.messages[0]
    )

    manager.handle_playback_status(terminal_status(
        request.request_id,
        CommandStatus.STATE_FAILED,
        CommandID.MOVE_TO_WAYPOINT.value,
    ))

    assert len(manager.command_submission_pub.messages) == 1
    assert manager._playback_commands == deque()
    assert manager._playback_request_id == ""
    assert manager.playback_state_pub.messages == [Bool(data=False)]
    assert manager.logger.warning_messages
