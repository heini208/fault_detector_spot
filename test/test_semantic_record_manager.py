"""Tests for semantic command recording and sequential playback."""

from collections import deque
import json

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
from fault_detector_spot.application.recording.recorded_step import RecordedStep
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
    _publish_retry_setting = RecordManager._publish_retry_setting
    set_recording_retries = RecordManager.set_recording_retries
    _dispatch_playback_step = RecordManager._dispatch_playback_step
    _publish_recording_feedback = RecordManager._publish_recording_feedback
    _publish_command_recording_outcome = RecordManager._publish_command_recording_outcome
    handle_command_status = RecordManager.handle_command_status
    handle_recording_status = RecordManager.handle_recording_status
    _refresh_successful_commands = RecordManager._refresh_successful_commands
    _has_pending_commands = RecordManager._has_pending_commands
    capture_request = RecordManager.capture_request
    handle_playback_status = RecordManager.handle_playback_status
    _dispatch_next_playback_command = RecordManager._dispatch_next_playback_command
    _finish_playback = RecordManager._finish_playback

    def __init__(self):
        self.recording = False
        self.recording_paused = False
        self.temp_data = []
        self._recorded_request_ids = set()
        self._recording_commands = {}
        self._recording_results = {}
        self._recording_stop_requested = False
        self._last_recording_request_id = ""
        self.recording_feedback_pub = FakePublisher()
        self._playback_commands = deque()
        self._playback_request_id = ""
        self._playback_name = ""
        self._playback_step = None
        self._playback_retries_left = 0
        self._playback_acquisition_active = False
        self._retry_generation = 0
        self._retry_settings = {0: 0}
        self._retry_tokens = {}
        self._retry_consumed = set()
        self._recording_retries = {}
        self.recording_retries_pub = FakePublisher()
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
    assert manager.temp_data == []
    manager.handle_command_status(terminal_status(included.request_id, CommandStatus.STATE_SUCCEEDED))
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
    manager._playback_commands = deque([RecordedStep(first), RecordedStep(second)])
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
    manager._playback_commands = deque([RecordedStep(first), RecordedStep(second)])
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
    set_recording_paused = RecordManager.set_recording_paused
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
        self.recording_paused_pub = FakePublisher()
        self.recording_finishing_pub = FakePublisher()

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


def test_paused_commands_are_logged_but_absent_from_saved_recording(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("start", "session")
    before = make_request(CommandID.STAND_UP)
    excluded = make_request(CommandID.WAIT_TIME)
    after = make_request(CommandID.STOW_ARM)
    assert manager.capture_request(command_request_to_message(before))
    manager.control("pause")
    assert manager.recording
    assert manager.recording_paused_pub.messages[-1].data
    message = command_request_to_message(excluded)
    assert not manager.capture_request(message)
    assert any(excluded.request_id in entry for entry in manager.logger.info_messages)
    assert command_request_from_message(message) == excluded
    manager.control("resume")
    # A duplicate accepted event must not include an excluded command later.
    assert not manager.capture_request(message)
    assert manager.capture_request(command_request_to_message(after))
    manager.handle_command_status(terminal_status(before.request_id, CommandStatus.STATE_SUCCEEDED))
    manager.handle_command_status(terminal_status(after.request_id, CommandStatus.STATE_SUCCEEDED))
    manager.control("stop")
    assert manager.repository.load("session") == [before.command, after.command]
    assert not manager.command_submission_pub.messages
    assert not manager.playback_state_pub.messages


def test_pause_is_idempotent_and_resets_between_recordings(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("pause")
    assert not manager.recording_paused
    manager.control("start", "first")
    manager.control("pause")
    manager.control("pause")
    assert manager.recording_paused
    manager.control("stop")
    assert not manager.recording_paused_pub.messages[-1].data
    manager.control("start", "second")
    assert not manager.recording_paused
    assert manager.capture_request(command_request_to_message(make_request(CommandID.STAND_UP)))


def test_failed_save_preserves_paused_recording(tmp_path, monkeypatch):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("start", "session")
    manager.control("pause")

    def fail_save(*args):
        raise OSError("disk full")

    monkeypatch.setattr(manager.repository, "save", fail_save)
    manager.control("stop")
    assert manager.recording and manager.recording_paused
    assert manager.recording_paused_pub.messages[-1].data


def test_stop_waits_for_outcomes_and_keeps_successes_in_accepted_order(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("start", "session")
    requests = [make_request(command) for command in (
        CommandID.STAND_UP, CommandID.WAIT_TIME, CommandID.SIT_DOWN, CommandID.STOW_ARM,
    )]
    for request in requests:
        assert manager.capture_request(command_request_to_message(request))
    manager.control("stop")
    assert manager.recording_finishing_pub.messages[-1].data
    assert not manager.repository.path("session").exists()
    assert not manager.capture_request(command_request_to_message(make_request(CommandID.READY_ARM)))
    manager.control("start", "another")
    assert manager.current_name == "session"
    for index, outcome in (
        (3, CommandStatus.STATE_SUCCEEDED),
        (1, CommandStatus.STATE_FAILED),
        (2, CommandStatus.STATE_CANCELLED),
    ):
        manager.handle_command_status(terminal_status(requests[index].request_id, outcome))
        assert manager.recording
    manager.handle_command_status(terminal_status(requests[0].request_id, CommandStatus.STATE_SUCCEEDED))
    assert not manager.recording
    assert not manager.recording_finishing_pub.messages[-1].data
    assert manager.repository.load("session") == [requests[0].command, requests[3].command]
    assert not manager.command_submission_pub.messages


@pytest.mark.parametrize("outcome", [CommandStatus.STATE_SUCCEEDED, CommandStatus.STATE_FAILED, CommandStatus.STATE_CANCELLED])
def test_outcome_before_acceptance_and_duplicate_status(tmp_path, outcome):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("start", "session")
    request = make_request(CommandID.STAND_UP)
    manager.handle_command_status(terminal_status(request.request_id, outcome))
    manager.capture_request(command_request_to_message(request))
    manager.handle_command_status(terminal_status(request.request_id, CommandStatus.STATE_SUCCEEDED))
    manager.control("stop")
    expected = [request.command] if outcome == CommandStatus.STATE_SUCCEEDED else []
    assert manager.repository.load("session") == expected


def test_pause_is_decided_at_acceptance_not_completion(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("start", "session")
    included = make_request(CommandID.STAND_UP)
    excluded = make_request(CommandID.SIT_DOWN)
    manager.capture_request(command_request_to_message(included))
    manager.control("pause")
    manager.capture_request(command_request_to_message(excluded))
    manager.handle_command_status(terminal_status(included.request_id, CommandStatus.STATE_SUCCEEDED))
    manager.control("resume")
    manager.control("stop")
    assert not manager.recording  # Excluded command does not delay saving.
    manager.handle_command_status(terminal_status(excluded.request_id, CommandStatus.STATE_SUCCEEDED))
    assert manager.repository.load("session") == [included.command]


def test_deferred_save_failure_can_be_retried(tmp_path, monkeypatch):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("start", "session")
    request = make_request(CommandID.STAND_UP)
    manager.capture_request(command_request_to_message(request))
    manager.control("stop")
    def fail_save(*args):
        raise OSError("disk full")
    with monkeypatch.context() as patch:
        patch.setattr(manager.repository, "save", fail_save)
        manager.handle_command_status(terminal_status(request.request_id, CommandStatus.STATE_SUCCEEDED))
    assert manager.recording
    assert not manager.recording_finishing_pub.messages[-1].data
    assert manager.temp_data == [request.command]
    assert manager.logger.error_messages
    manager.control("stop")
    assert manager.repository.load("session") == [request.command]


@pytest.mark.parametrize("state,outcome", [
    (CommandStatus.STATE_SUCCEEDED, "added"),
    (CommandStatus.STATE_FAILED, "failed"),
    (CommandStatus.STATE_CANCELLED, "cancelled"),
])
def test_recording_feedback_reports_confirmed_outcome(tmp_path, state, outcome):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("start", "session")
    request = make_request(CommandID.STAND_UP)
    manager.capture_request(command_request_to_message(request))
    assert json.loads(manager.recording_feedback_pub.messages[-1].data)["outcome"] == "pending"
    manager.handle_command_status(terminal_status(request.request_id, state))
    assert json.loads(manager.recording_feedback_pub.messages[-1].data) == {
        "request_id": request.request_id, "command_id": "stand_up", "outcome": outcome,
    }


def test_latest_command_feedback_survives_older_completion(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("start", "session")
    first = make_request(CommandID.STAND_UP)
    manager.capture_request(command_request_to_message(first))
    manager.control("pause")
    second = make_request(CommandID.STOW_ARM)
    manager.capture_request(command_request_to_message(second))
    feedback = manager.recording_feedback_pub.messages[-1].data
    assert json.loads(feedback)["outcome"] == "paused"
    manager.handle_command_status(terminal_status(first.request_id, CommandStatus.STATE_SUCCEEDED))
    manager.handle_command_status(terminal_status(second.request_id, CommandStatus.STATE_SUCCEEDED))
    assert manager.recording_feedback_pub.messages[-1].data == feedback
    manager.control("stop")
    assert manager.recording_feedback_pub.messages[-1].data == feedback
    manager.control("start", "next")
    assert json.loads(manager.recording_feedback_pub.messages[-1].data)["outcome"] == "idle"


def test_retry_setting_survives_failure_pause_and_out_of_order_success(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("start", "session")
    manager.set_recording_retries(2)
    first, second, third = [make_request(CommandID.MOVE_ARM_TO_TAG) for _ in range(3)]
    manager.capture_request(command_request_to_message(first))
    manager.control("pause")
    manager.capture_request(command_request_to_message(make_request(CommandID.WAIT_TIME)))
    manager.control("resume")
    manager.capture_request(command_request_to_message(second))
    manager.capture_request(command_request_to_message(third))
    manager.handle_command_status(terminal_status(third.request_id, CommandStatus.STATE_SUCCEEDED))
    manager.handle_command_status(terminal_status(second.request_id, CommandStatus.STATE_SUCCEEDED))
    assert manager.recording_retries_pub.messages[-1].data == 2
    manager.handle_command_status(terminal_status(first.request_id, CommandStatus.STATE_FAILED))
    assert manager.recording_retries_pub.messages[-1].data == 0
    manager.control("stop")
    assert [step.playback_retries for step in manager.repository.load_steps("session")] == [2, 0]


def test_retry_setting_does_not_retroactively_change_accepted_commands(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("start", "session")
    first = make_request(CommandID.MOVE_ARM_TO_TAG)
    manager.set_recording_retries(2)
    manager.capture_request(command_request_to_message(first))
    manager.set_recording_retries(4)
    manager.handle_command_status(terminal_status(first.request_id, CommandStatus.STATE_SUCCEEDED))
    assert manager.recording_retries_pub.messages[-1].data == 4
    manager.control("stop")
    assert manager.repository.load_steps("session")[0].playback_retries == 2


def test_playback_retry_uses_new_ids_and_stops_at_budget(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    command = SemanticCommand(command_id=CommandID.MOVE_TO_WAYPOINT)
    manager.repository.save("session", [RecordedStep(command, 2)])
    manager.control("play", "session")
    ids = []
    for attempt in range(3):
        request_id = manager._playback_request_id
        ids.append(request_id)
        failed = terminal_status(request_id, CommandStatus.STATE_FAILED)
        failed.retry_safe = True
        manager.handle_playback_status(failed)
        count = len(manager.command_submission_pub.messages)
        manager.handle_playback_status(failed)  # Duplicate failure cannot retry again.
        assert len(manager.command_submission_pub.messages) == count
    assert len(set(ids)) == 3
    assert len(manager.command_submission_pub.messages) == 3
    assert not manager._playback_request_id


@pytest.mark.parametrize("cancelled, safe, acquisition", [
    (True, True, False), (False, False, False), (False, True, True),
])
def test_playback_retry_requires_safe_failure_without_active_acquisition(tmp_path, cancelled, safe, acquisition):
    manager = RecordingStorageHarness(tmp_path)
    command = SemanticCommand(command_id=CommandID.MOVE_TO_WAYPOINT)
    manager.repository.save("session", [RecordedStep(command, 2)])
    manager.control("play", "session")
    manager._playback_acquisition_active = acquisition
    status = terminal_status(manager._playback_request_id,
                             CommandStatus.STATE_CANCELLED if cancelled else CommandStatus.STATE_FAILED)
    status.retry_safe = safe
    manager.handle_playback_status(status)
    assert len(manager.command_submission_pub.messages) == 1
    assert not manager._playback_request_id


def test_unsupported_retry_is_reported_without_changing_probe_retries(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    manager.control("start", "session")
    manager.set_recording_retries(3)
    request = make_request(CommandID.MOVE_ARM_RELATIVE)
    manager.capture_request(command_request_to_message(request))
    manager.handle_command_status(terminal_status(request.request_id, CommandStatus.STATE_SUCCEEDED))
    assert json.loads(manager.recording_feedback_pub.messages[-1].data)["retry_unsupported"]
    manager.control("stop")
    assert manager.repository.load_steps("session")[0].playback_retries == 0


def test_retry_success_advances_and_new_step_gets_its_own_budget(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    command = SemanticCommand(command_id=CommandID.MOVE_TO_WAYPOINT)
    manager.repository.save("session", [RecordedStep(command, 1), RecordedStep(command, 2)])
    manager.control("play", "session")
    failed = terminal_status(manager._playback_request_id, CommandStatus.STATE_FAILED)
    failed.retry_safe = True
    manager.handle_playback_status(failed)
    assert manager._playback_retries_left == 0
    manager.handle_playback_status(terminal_status(manager._playback_request_id, CommandStatus.STATE_SUCCEEDED))
    assert manager._playback_retries_left == 2
    assert len(manager.command_submission_pub.messages) == 3


def test_emergency_command_stops_retry_playback(tmp_path):
    manager = RecordingStorageHarness(tmp_path)
    command = SemanticCommand(command_id=CommandID.MOVE_TO_WAYPOINT)
    manager.repository.save("session", [RecordedStep(command, 2)])
    manager.control("play", "session")
    original_id = manager._playback_request_id
    manager.capture_request(command_request_to_message(make_request(CommandID.EMERGENCY_CANCEL)))
    failed = terminal_status(original_id, CommandStatus.STATE_FAILED)
    failed.retry_safe = True
    manager.handle_playback_status(failed)
    assert len(manager.command_submission_pub.messages) == 1
    assert not manager._playback_request_id
