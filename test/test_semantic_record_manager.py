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
from fault_detector_spot.application.recording.recording_repository import RecordingRepository
from fault_detector_spot.application.recording.semantic_command_codec import (
    deserialize_recorded_command,
    deserialize_recording,
    serialize_recorded_command,
)
from fault_detector_spot.application.ros.command_request_adapter import (
    command_request_from_message,
    command_request_to_message,
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
    assert manager.temp_data[0].command_id is CommandID.STAND_UP
    assert isinstance(manager.temp_data[0], SemanticCommand)


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
        target_surface_distance_m=0.12,
        surface_tolerance_m=0.008,
        aligned_preapproach_distance_m=0.23,
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


def test_close_surface_recording_preserves_motion_parameters():
    command = SemanticCommand(
        command_id=CommandID.MOVE_CLOSE_TO_SURFACE,
        target_surface_distance_m=0.1,
        surface_tolerance_m=0.005,
        aligned_preapproach_distance_m=0.23,
    )

    restored = deserialize_recorded_command(
        serialize_recorded_command(command)
    )

    assert restored == command


def test_runtime_sensor_binding_is_not_persisted():
    command = SemanticCommand(
        command_id=CommandID.ORIENT_TO_SURFACE,
        motion_sensor_id="bmm150_01",
    )

    data = serialize_recorded_command(command)
    restored = deserialize_recorded_command(data)

    assert "motion_sensor_id" not in data
    assert restored.motion_sensor_id == ""


def test_missing_persistent_command_field_is_rejected():
    data = serialize_recorded_command(
        SemanticCommand(command_id=CommandID.MOVE_CLOSE_TO_SURFACE)
    )
    del data["target_surface_distance_m"]

    with pytest.raises(
        ValueError,
        match="missing field.*target_surface_distance_m",
    ):
        deserialize_recorded_command(data)


def test_legacy_complex_command_recording_is_rejected():
    with pytest.raises(ValueError, match="missing field"):
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
    first_request = command_request_from_message(
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
    second_request = command_request_from_message(
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
    request = command_request_from_message(
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


class RecordingStorageHarness(RecordManagerHarness):
    publish_recording_state = RecordManager.publish_recording_state
    handle_control = RecordManager.handle_control
    start_recording = RecordManager.start_recording
    stop_recording = RecordManager.stop_recording
    play_recording = RecordManager.play_recording
    delete_recording = RecordManager.delete_recording
    publish_recordings_list = RecordManager.publish_recordings_list

    def __init__(self, root):
        super().__init__()
        self.repository = RecordingRepository(root)
        self.current_name = None
        self.list_pub = FakePublisher()
        self.recording_state_pub = FakePublisher()

    def control(self, mode, name=""):
        from fault_detector_msgs.msg import CommandRecordControl
        self.handle_control(CommandRecordControl(mode=mode, name=name))


@pytest.mark.parametrize("mode", ["start", "play", "delete"])
@pytest.mark.parametrize("name", ["../outside", "absolute", "nested/name", "bad\\name", "bad\x00name", ".", ".."])
def test_recording_control_rejects_unsafe_paths(tmp_path, mode, name):
    root = tmp_path / "recordings"
    root.mkdir()
    outside = tmp_path / "outside.json"
    outside.write_text('{"commands": []}')
    manager = RecordingStorageHarness(root)
    if name == "absolute":
        name = str(outside.with_suffix(""))
    manager.control(mode, name)
    assert outside.read_text() == '{"commands": []}'
    assert list(root.iterdir()) == []
    assert not manager.recording
    assert not manager.command_submission_pub.messages
    assert not manager.playback_state_pub.messages
    assert manager.logger.error_messages


@pytest.mark.parametrize("mode", ["start", "play", "delete"])
def test_recording_control_rejects_symlinks(tmp_path, mode):
    root = tmp_path / "recordings"
    root.mkdir()
    outside = tmp_path / "outside.json"
    outside.write_text('{"commands": []}')
    link = root / "linked.json"
    link.symlink_to(outside)
    manager = RecordingStorageHarness(root)
    manager.control(mode, "linked")
    assert outside.read_text() == '{"commands": []}'
    assert link.is_symlink()
    assert not manager.recording
    assert not manager.playback_state_pub.messages
    assert manager.logger.error_messages
    manager.publish_recordings_list()
    assert manager.list_pub.messages[-1].names == []


def test_recording_save_rechecks_path_and_keeps_unsaved_data(tmp_path):
    root = tmp_path / "recordings"
    root.mkdir()
    outside = tmp_path / "outside.json"
    outside.write_text("do not overwrite")
    manager = RecordingStorageHarness(root)
    manager.control("start", "session")
    manager.temp_data.append(SemanticCommand(command_id=CommandID.STAND_UP))
    (root / "session.json").symlink_to(outside)
    manager.control("stop")
    assert outside.read_text() == "do not overwrite"
    assert manager.recording
    assert manager.temp_data == [SemanticCommand(command_id=CommandID.STAND_UP)]
    assert manager.logger.error_messages


def test_valid_recording_storage_lifecycle(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    (tmp_path / "directory.json").mkdir()
    manager.control("start", "inspection run-1")
    manager.control("stop")
    assert not manager.recording
    assert (tmp_path / "inspection run-1.json").is_file()
    assert manager.list_pub.messages[-1].names == ["inspection run-1"]
    manager.control("play", "inspection run-1")
    assert [message.data for message in manager.playback_state_pub.messages] == [True, False]
    manager.control("delete", "inspection run-1")
    assert not (tmp_path / "inspection run-1.json").exists()
    assert manager.list_pub.messages[-1].names == []
    assert not manager.logger.error_messages


def test_recording_state_reports_actual_start_stop_and_failures(tmp_path, monkeypatch):
    manager = RecordingStorageHarness(tmp_path)
    manager.publish_recording_state()
    manager.control("start", "../invalid")
    manager.control("start", "valid")
    manager.temp_data.append(SemanticCommand(command_id=CommandID.STAND_UP))
    manager.control("start", "duplicate")
    assert manager.current_name == "valid"
    assert manager.temp_data == [SemanticCommand(command_id=CommandID.STAND_UP)]

    def fail_open(*_args, **_kwargs):
        raise OSError("disk full")

    with monkeypatch.context() as patch:
        patch.setattr(manager.repository, "save", fail_open)
        manager.control("stop")
    assert manager.recording
    assert manager.temp_data == [SemanticCommand(command_id=CommandID.STAND_UP)]
    manager.control("stop")
    assert [message.data for message in manager.recording_state_pub.messages] == [
        False, False, True, True, True, False,
    ]
