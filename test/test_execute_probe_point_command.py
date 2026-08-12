"""Tests for semantic probe-point command conversion."""

import pytest
from builtin_interfaces.msg import Time

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    InspectionSelection,
    SemanticCommand,
)
from fault_detector_spot.inspection.commands import ExecuteProbePointCommand


def test_semantic_command_contains_only_saved_selection_ids():
    command = ExecuteProbePointCommand(
        stamp=Time(sec=7, nanosec=11),
        object_id="motor_a",
        routine_id="magnetic_scan",
        probe_point_id="bearing_1",
    )

    assert command.command_id == CommandID.EXECUTE_PROBE_POINT
    assert command.object_id == "motor_a"
    assert command.routine_id == "magnetic_scan"
    assert command.probe_point_id == "bearing_1"


@pytest.mark.parametrize(
    "object_id,routine_id,probe_point_id",
    [
        ("", "scan", "point_1"),
        ("motor_a", " scan", "point_1"),
        ("motor_a", "scan", "../point_1"),
    ],
)
def test_semantic_command_rejects_invalid_selection_ids(
    object_id,
    routine_id,
    probe_point_id,
):
    with pytest.raises(ValueError):
        ExecuteProbePointCommand(
            stamp=Time(),
            object_id=object_id,
            routine_id=routine_id,
            probe_point_id=probe_point_id,
        )


def test_topic_command_is_rejected_until_action_server_exists():
    command = SemanticCommand(
        command_id=CommandID.EXECUTE_PROBE_POINT,
        inspection=InspectionSelection(
            object_id="motor_a",
            routine_id="magnetic_scan",
            probe_point_id="bearing_1",
        ),
    )
    subscriber = CommandSubscriber()

    with pytest.raises(ValueError, match="action server"):
        subscriber.fire_command_sequence(command)
