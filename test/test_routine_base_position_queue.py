"""Keep saved routine base moves in the shared operational command queue."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.application_controller import (
    ApplicationController,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    CommandExecutionStatus,
)
from fault_detector_spot.application.coordinators.probe_setup_coordinator import (
    ProbeSetupCoordinator,
)
from fault_detector_spot.inspection.model.models import (
    InspectionObject,
    InspectionRoutine,
    ReferenceTag,
)
from fault_detector_spot.shared.geometry.models import PoseData


@pytest.fixture
def queue_rig():
    base_position = PoseData.identity()
    base_position.position.x = -1.2
    base_position.position.y = 0.35
    repository = Mock()
    repository.list_object_ids.return_value = ["motor"]
    repository.load.return_value = InspectionObject(
        object_id="motor",
        display_name="Motor",
        routines=[InspectionRoutine(
            routine_id="scan",
            display_name="Scan",
            reference_tag=ReferenceTag(tag_id=7, tag_family="36h11"),
            base_position=base_position,
            base_body_height_m=-0.13,
        )],
    )
    dispatched = []
    commands = CommandController(dispatch_request=dispatched.append)
    app = ApplicationController(commands)
    probe = ProbeSetupCoordinator(
        app.setup_coordinator,
        reference_repository=SimpleNamespace(object_repository=repository),
        sensor_attachment_controller=Mock(),
        motion_state_source=None,
    )
    app.attach_probe_setup(probe)
    states = []
    app.add_status_listener(states.append)
    return SimpleNamespace(
        app=app, commands=commands, probe=probe, repository=repository,
        dispatched=dispatched, states=states,
    )


def submit(rig, intent_id):
    intent = OperationalIntent()
    intent.intent = intent_id
    if intent_id == OperationalIntent.INTENT_MOVE_TO_ROUTINE_BASE_POSITION:
        intent.object_id = "motor"
        intent.routine_id = "scan"
        intent.walking_profile = "precision"
    operation = rig.app.prepare_operation(intent, "operator_ui")
    rig.app.submit(operation)
    return operation


def succeed(rig, operation, remaining_steps=0):
    rig.commands.handle_execution_status(CommandExecutionStatus(
        request_id=operation.request_id,
        state=CommandControllerState.SUCCEEDED,
        buffered_command_count=remaining_steps,
    ))


def test_saved_base_move_queues_behind_active_and_pending_commands_without_live_pose(queue_rig):
    rig = queue_rig
    first = submit(rig, OperationalIntent.INTENT_STOW_ARM)
    second = submit(rig, OperationalIntent.INTENT_STAND_UP)
    base = submit(rig, OperationalIntent.INTENT_MOVE_TO_ROUTINE_BASE_POSITION)

    assert rig.dispatched == [first.request]
    assert rig.commands.active_request_id == first.request_id
    assert rig.commands.queued_request_ids == (second.request_id, base.request_id)
    command = base.request.command
    assert command.command_id is CommandID.MOVE_BASE_TO_TAG
    assert command.tag.id == 7
    assert command.offset.position.x == pytest.approx(-1.2)
    assert command.offset.position.y == pytest.approx(0.35)
    assert command.body_height_m == pytest.approx(-0.13)
    assert command.walking_profile == "precision"

    succeed(rig, first)
    assert rig.dispatched == [first.request, second.request]
    assert rig.commands.active_request_id == second.request_id
    assert rig.commands.queued_request_ids == (base.request_id,)

    succeed(rig, second)
    assert rig.dispatched == [first.request, second.request, base.request]
    assert rig.commands.active_request_id == base.request_id
    assert rig.commands.queued_request_ids == ()

    following = submit(rig, OperationalIntent.INTENT_STOW_ARM)
    succeed(rig, base, remaining_steps=1)
    assert rig.dispatched == [first.request, second.request, base.request]
    assert rig.commands.active_request_id == base.request_id
    succeed(rig, base)
    assert rig.dispatched[-1] == following.request
    assert rig.commands.active_request_id == following.request_id


def test_queued_saved_base_move_can_be_cancelled_without_dispatch_or_emergency_stop(queue_rig):
    rig = queue_rig
    first = submit(rig, OperationalIntent.INTENT_STOW_ARM)
    second = submit(rig, OperationalIntent.INTENT_STAND_UP)
    base = submit(rig, OperationalIntent.INTENT_MOVE_TO_ROUTINE_BASE_POSITION)

    assert rig.app.cancel("operator_ui", base.request_id) == base.request_id
    assert rig.commands.queued_request_ids == (second.request_id,)
    assert rig.dispatched == [first.request]
    assert rig.states[-1].operation.request_id == base.request_id
    assert rig.states[-1].state is CommandControllerState.CANCELLED

    succeed(rig, first)
    succeed(rig, second)
    assert rig.dispatched == [first.request, second.request]
    assert rig.commands.active_request_id == ""
    assert rig.commands.queued_request_ids == ()


def test_saving_base_position_still_requires_idle_lane(queue_rig):
    rig = queue_rig
    state = rig.probe.open_context("operator_ui")
    state = rig.probe.select_routine(state.context, "motor", "scan")
    first = submit(rig, OperationalIntent.INTENT_STOW_ARM)

    with pytest.raises(RuntimeError, match="idle before saving a base position"):
        rig.probe.save_base_position(state.context, -0.13)

    rig.repository.set_routine_base_position.assert_not_called()
    assert rig.dispatched == [first.request]
