"""Tests for moving to a routine's saved base position."""

import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import OperationalIntent, TagElement
from py_trees.common import Status

from fault_detector_spot.application.behaviour_tree.behaviours.command_manager import (
    CommandManager,
)
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import (
    CommandSubscriber,
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
from fault_detector_spot.application.controllers.application_controller import (
    ApplicationController,
)
from fault_detector_spot.application.ros.operational_intent_adapter import (
    operational_intent_to_command,
)
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message,
    semantic_command_to_message,
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


def _definition(with_base_position=True, body_height_m=0.0):
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
                base_body_height_m=body_height_m,
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


@pytest.mark.parametrize("body_height_m", [0.0, -0.13, 0.2])
def test_saved_routine_base_position_builds_existing_base_to_tag_command(body_height_m):
    definition = _definition(body_height_m=body_height_m)
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
    assert command.body_height_m == pytest.approx(body_height_m)
    assert command.inspection.object_id == "motor"
    assert command.inspection.routine_id == "scan"
    state_source.reference_tag.assert_called_once_with(7)


def _queued_base_motion(body_height_m):
    repository = Mock()
    repository.load.return_value = _definition(body_height_m=body_height_m)
    state_source = Mock()
    state_source.reference_tag.return_value = _tag()
    command = routine_base_position_command(_intent(), repository, state_source)
    command = semantic_command_from_message(semantic_command_to_message(command))
    request = CommandRequest.create(
        command=command,
        client_id="operator_ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )
    node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=12, nanosec=34))
        )
    )
    blackboard = SimpleNamespace(
        command_buffer=[], command_tree_status=Status.SUCCESS, last_command=None
    )
    subscriber = CommandSubscriber()
    subscriber.node = node
    subscriber.blackboard = blackboard
    subscriber.fire_request(request)
    manager = CommandManager()
    manager.node = node
    manager.blackboard = blackboard
    return request, subscriber, manager


@pytest.mark.parametrize("body_height_m", [0.0, -0.13, 0.2])
def test_routine_base_height_is_optional_final_step_in_the_same_request(body_height_m):
    request, _, manager = _queued_base_motion(body_height_m)
    commands = manager.blackboard.command_buffer

    expected_ids = [CommandID.MOVE_BASE_TO_TAG]
    if body_height_m:
        expected_ids.append(CommandID.CHANGE_BODY_HEIGHT)
        assert commands[-1].body_height_m == pytest.approx(body_height_m)
    assert [command.command_id for command in commands] == expected_ids
    assert all(command.request_id == request.request_id for command in commands)


@pytest.mark.parametrize("movement_result", [Status.SUCCESS, Status.FAILURE])
def test_saved_height_only_dispatches_after_successful_base_movement(movement_result):
    _, _, manager = _queued_base_motion(-0.13)
    manager.update()
    movement = manager.blackboard.last_command
    assert movement.command_id is CommandID.MOVE_BASE_TO_TAG

    manager.blackboard.command_tree_status = Status.RUNNING
    manager.update()
    assert manager.blackboard.last_command is movement
    assert len(manager.blackboard.command_buffer) == 1

    manager.blackboard.command_tree_status = movement_result
    manager.update()
    assert manager.blackboard.command_buffer == []
    if movement_result is Status.SUCCESS:
        assert manager.blackboard.last_command.command_id is CommandID.CHANGE_BODY_HEIGHT
    else:
        assert manager.blackboard.last_command is movement


def test_cancellation_drops_saved_height_after_base_movement():
    _, subscriber, manager = _queued_base_motion(-0.13)
    manager.update()
    manager.blackboard.command_tree_status = Status.RUNNING
    cancel = CommandRequest.create(
        command=SemanticCommand(command_id=CommandID.EMERGENCY_CANCEL),
        client_id="operator_ui",
        origin=CommandOrigin.OPERATIONAL,
        recording_policy=RecordingPolicy.INCLUDE_IF_RECORDING_ACTIVE,
    )
    subscriber.trigger_estop(cancel)

    manager.update()

    assert manager.blackboard.command_buffer == []
    assert manager.blackboard.last_command.command_id is CommandID.EMERGENCY_CANCEL
    assert manager.blackboard.last_command.request_id == cancel.request_id


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
