"""Tests for semantic probe-point command conversion."""

from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import ComplexCommand

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.commands import (
    execute_probe_point_command,
)
from fault_detector_spot.behaviour_tree.nodes.sensing.command_subscriber import (
    CommandSubscriber,
)


ExecuteProbePointCommand = (
    execute_probe_point_command.ExecuteProbePointCommand
)


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


def test_complex_message_converts_to_one_semantic_command():
    message = ComplexCommand()
    message.command.command_id = CommandID.EXECUTE_PROBE_POINT
    message.inspection.object.object_id = "motor_a"
    message.inspection.routine.routine_id = "magnetic_scan"
    message.inspection.probe_point_id = "bearing_1"
    subscriber = CommandSubscriber()
    subscriber.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: Time(sec=8, nanosec=12)
            )
        )
    )

    commands = subscriber._execute_probe_point(message)

    assert len(commands) == 1
    assert isinstance(commands[0], ExecuteProbePointCommand)
    assert commands[0].object_id == "motor_a"
    assert commands[0].routine_id == "magnetic_scan"
    assert commands[0].probe_point_id == "bearing_1"
