"""Tests for transient active-sensor command context."""

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.recording.semantic_command_codec import (
    deserialize_recorded_command,
    serialize_recorded_command,
)
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message,
    semantic_command_to_message,
)


def test_sensor_context_round_trips_over_command_wire():
    command = SemanticCommand(
        command_id=CommandID.MOVE_ARM_TO_TAG,
        motion_sensor_id="bmm150_01",
    )

    message = semantic_command_to_message(command)
    restored = semantic_command_from_message(message)

    assert message.motion_sensor_id == "bmm150_01"
    assert restored.motion_sensor_id == "bmm150_01"


def test_recording_omits_transient_sensor_context():
    command = SemanticCommand(
        command_id=CommandID.MOVE_ARM_TO_TAG,
        motion_sensor_id="bmm150_01",
    )

    data = serialize_recorded_command(command)
    restored = deserialize_recorded_command(data)

    assert "motion_sensor_id" not in data
    assert restored.motion_sensor_id == ""
