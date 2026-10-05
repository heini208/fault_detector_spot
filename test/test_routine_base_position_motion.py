"""Tests for moving to a routine's saved base position."""

import math
from unittest.mock import Mock

import pytest
from fault_detector_msgs.msg import OperationalIntent, TagElement

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.application_controller import (
    ApplicationController,
)
from fault_detector_spot.application.ros.operational_intent_adapter import (
    operational_intent_to_command,
)
from fault_detector_spot.inspection.execution.routine_base_motion import (
    routine_base_position_command,
)
from fault_detector_spot.inspection.model.models import (
    InspectionObject,
    InspectionRoutine,
    ReferenceTag,
)
from fault_detector_spot.shared.geometry.models import (
    PoseData,
)
from test_application_controller import FakeCommandController


def _definition(with_base_position=True):
    base_position = None
    if with_base_position:
        base_position = PoseData.identity()
        base_position.position.x = -1.2
        base_position.position.y = 0.35
        yaw = math.radians(20.0) * 0.5
        base_position.orientation.z = math.sin(yaw)
        base_position.orientation.w = math.cos(yaw)
    return InspectionObject(
        object_id="motor",
        display_name="Motor",
        routines=[
            InspectionRoutine(
                routine_id="scan",
                display_name="Scan",
                reference_tag=ReferenceTag(
                    tag_id=7,
                    tag_family="36h11",
                ),
                base_position=base_position,
            )
        ],
    )


def _intent():
    intent = OperationalIntent()
    intent.intent = OperationalIntent.INTENT_MOVE_TO_ROUTINE_BASE_POSITION
    intent.object_id = "motor"
    intent.routine_id = "scan"
    intent.walking_profile = "precision"
    return intent


def _tag():
    tag = TagElement()
    tag.id = 7
    tag.pose.header.frame_id = "body"
    tag.pose.header.stamp.sec = 12
    tag.pose.header.stamp.nanosec = 34
    tag.pose.pose.position.x = 0.8
    tag.pose.pose.orientation.w = 1.0
    return tag


def test_saved_routine_base_position_builds_existing_base_to_tag_command():
    definition = _definition()
    repository = Mock()
    repository.load.return_value = definition
    state_source = Mock()
    state_source.reference_tag.return_value = _tag()

    command = routine_base_position_command(
        _intent(),
        repository,
        state_source,
    )

    base_position = definition.get_routine("scan").base_position
    assert command.command_id is CommandID.MOVE_BASE_TO_TAG
    assert command.tag.id == 7
    assert command.tag.pose.frame_id == "body"
    assert command.offset.frame_id == "Tag_7"
    assert command.offset.position.x == base_position.position.x
    assert command.offset.position.y == base_position.position.y
    assert command.offset.position.z == 0.0
    assert command.offset.orientation.x == base_position.orientation.x
    assert command.offset.orientation.y == base_position.orientation.y
    assert command.offset.orientation.z == base_position.orientation.z
    assert command.offset.orientation.w == base_position.orientation.w
    assert command.walking_profile == "precision"
    assert command.inspection.object_id == "motor"
    assert command.inspection.routine_id == "scan"
    state_source.reference_tag.assert_called_once_with(7)


def test_missing_routine_base_position_is_rejected():
    repository = Mock()
    repository.load.return_value = _definition(with_base_position=False)

    with pytest.raises(ValueError, match="no configured base position"):
        routine_base_position_command(
            _intent(),
            repository,
            Mock(),
        )


@pytest.mark.parametrize("field_name", ["object_id", "routine_id"])
def test_adapter_requires_routine_base_selection(field_name):
    intent = _intent()
    operational_intent_to_command(intent)

    setattr(intent, field_name, "")
    with pytest.raises(ValueError):
        operational_intent_to_command(intent)


def test_application_controller_resolves_routine_base_before_submission():
    controller = ApplicationController(FakeCommandController())
    resolved = SemanticCommand(command_id=CommandID.MOVE_BASE_TO_TAG)
    coordinator = Mock()
    coordinator.uses_setup_coordinator.return_value = True
    coordinator.routine_base_position_command.return_value = resolved
    controller.attach_probe_setup(coordinator)

    intent = _intent()
    operation = controller.prepare_operation(intent, "ui")

    coordinator.routine_base_position_command.assert_called_once_with(intent)
    assert operation.request.command is resolved
    assert operation.intent == (
        OperationalIntent.INTENT_MOVE_TO_ROUTINE_BASE_POSITION
    )
