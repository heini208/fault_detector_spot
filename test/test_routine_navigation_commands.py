"""Routine navigation uses saved targets and the shared operational queue."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import OperationalIntent

from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.controllers.application_controller import ApplicationController
from fault_detector_spot.application.controllers.command_controller import (
    CommandController,
    CommandControllerState,
    CommandExecutionStatus,
)
from fault_detector_spot.application.coordinators.probe_setup_coordinator import ProbeSetupCoordinator
from fault_detector_spot.application.ros.operational_intent_adapter import operational_intent_to_command
from fault_detector_spot.application.ros.semantic_command_adapter import (
    semantic_command_from_message,
    semantic_command_to_message,
)
from fault_detector_spot.inspection.execution.routine_navigation import routine_navigation_command
from fault_detector_spot.inspection.model.models import InspectionObject, InspectionRoutine, ReferenceTag
from fault_detector_spot.mapping.model.models import MapDefinition, Waypoint
from fault_detector_spot.mapping.repository.map_artifact_store import MapArtifactStore
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.shared.geometry.models import PoseData


LAUNCH = OperationalIntent.INTENT_LAUNCH_ROUTINE_MAP
MOVE = OperationalIntent.INTENT_MOVE_TO_ROUTINE_WAYPOINT


def intent(operation):
    result = OperationalIntent()
    result.intent = operation
    result.object_id = "motor"
    result.routine_id = "scan"
    return result


@pytest.fixture
def rig(tmp_path):
    routine = InspectionRoutine(
        routine_id="scan", display_name="Scan",
        reference_tag=ReferenceTag(tag_id=7, tag_family="36h11"),
        map_id="plant", waypoint_id="inspection_start",
    )
    repository = Mock()
    repository.load.return_value = InspectionObject(
        object_id="motor", display_name="Motor", routines=[routine],
    )
    maps = MapRepository(tmp_path)
    maps.save("plant", MapDefinition(
        map_id="plant", display_name="Plant",
        waypoints=[Waypoint(
            waypoint_id="inspection_start", display_name="Inspection start",
            pose_map=PoseData.identity(),
        )],
    ))
    database = tmp_path / "plant.db"
    database.touch()
    artifacts = MapArtifactStore(tmp_path)
    dispatched = []
    controller = CommandController(dispatch_request=dispatched.append)
    app = ApplicationController(controller)
    setup = ProbeSetupCoordinator(
        app.setup_coordinator,
        reference_repository=SimpleNamespace(object_repository=repository),
        sensor_attachment_controller=Mock(), map_repository=maps,
        map_artifacts=artifacts,
    )
    app.attach_probe_setup(setup)
    states = []
    app.add_status_listener(states.append)
    return SimpleNamespace(
        routine=routine, repository=repository, maps=maps, database=database,
        artifacts=artifacts, app=app, controller=controller,
        dispatched=dispatched, states=states,
    )


def resolve(rig, operation):
    return routine_navigation_command(intent(operation), rig.repository, rig.maps, rig.artifacts)


@pytest.mark.parametrize("operation, command_id", [
    (LAUNCH, CommandID.START_LOCALIZATION), (MOVE, CommandID.MOVE_TO_WAYPOINT),
])
def test_saved_navigation_targets_reach_existing_bt_command_builders(rig, operation, command_id):
    request = intent(operation)
    request.map_name = "unsaved_map"
    request.waypoint_name = "unsaved_waypoint"
    command = routine_navigation_command(request, rig.repository, rig.maps, rig.artifacts)
    command = semantic_command_from_message(semantic_command_to_message(command))
    subscriber = CommandSubscriber()
    subscriber.node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10)),
    ))

    [execution] = subscriber.fire_command_sequence(command)

    assert command.command_id is command_id
    assert command.map_name == execution.map_name == "plant"
    assert command.inspection.object_id == "motor"
    assert command.inspection.routine_id == "scan"
    assert command.inspection.probe_point_id == ""
    assert command.waypoint_name == ("" if operation == LAUNCH else "inspection_start")
    if operation == MOVE:
        assert execution.waypoint_name == "inspection_start"


def test_launch_does_not_require_saved_waypoint(rig):
    rig.routine.waypoint_id = ""
    assert resolve(rig, LAUNCH).command_id is CommandID.START_LOCALIZATION
    with pytest.raises(ValueError, match="no saved waypoint"):
        resolve(rig, MOVE)


@pytest.mark.parametrize("operation", [LAUNCH, MOVE])
def test_missing_saved_map_is_rejected(rig, operation):
    rig.routine.map_id = ""
    rig.routine.waypoint_id = ""
    with pytest.raises(ValueError, match="no saved map"):
        resolve(rig, operation)


def test_launch_requires_database_not_only_metadata_or_sidecars(rig):
    rig.database.unlink()
    rig.database.with_name("plant.db-wal").touch()
    rig.database.with_name("plant.db.bak").touch()
    with pytest.raises(FileNotFoundError, match="Database file not found"):
        resolve(rig, LAUNCH)


@pytest.mark.parametrize("operation", [LAUNCH, MOVE])
def test_missing_map_metadata_is_rejected(rig, operation):
    rig.maps.get_map_path("plant").unlink()
    with pytest.raises(FileNotFoundError, match="Map metadata does not exist"):
        resolve(rig, operation)


def test_missing_saved_waypoint_is_rejected(rig):
    rig.routine.waypoint_id = "deleted_waypoint"
    with pytest.raises(ValueError, match="does not exist in map 'plant'"):
        resolve(rig, MOVE)


def test_missing_routine_is_rejected(rig):
    request = intent(LAUNCH)
    request.routine_id = "deleted_routine"
    with pytest.raises(ValueError, match="routine does not exist"):
        routine_navigation_command(request, rig.repository, rig.maps, rig.artifacts)


@pytest.mark.parametrize("missing, detail", [
    ("repository", "Inspection object data is unavailable"),
    ("maps", "Map navigation data is unavailable"),
    ("artifacts", "Map database artifacts are unavailable"),
])
def test_missing_navigation_dependency_has_descriptive_failure(rig, missing, detail):
    setattr(rig, missing, None)
    with pytest.raises(RuntimeError, match=detail):
        resolve(rig, LAUNCH)


@pytest.mark.parametrize("operation", [LAUNCH, MOVE])
@pytest.mark.parametrize("field", ["object_id", "routine_id"])
def test_adapter_requires_only_saved_routine_selection(operation, field):
    request = intent(operation)
    operational_intent_to_command(request)
    setattr(request, field, "")
    with pytest.raises(ValueError, match="must not be empty"):
        operational_intent_to_command(request)


def submit(rig, operation):
    prepared = rig.app.prepare_operation(intent(operation), "operator_ui")
    rig.app.submit(prepared)
    return prepared


def succeed(rig, operation):
    rig.controller.handle_execution_status(CommandExecutionStatus(
        request_id=operation.request_id, state=CommandControllerState.SUCCEEDED,
    ))


def test_launch_and_waypoint_queue_with_no_active_map_or_live_robot_state(rig):
    first = submit(rig, OperationalIntent.INTENT_STOW_ARM)
    launch = submit(rig, LAUNCH)
    move = submit(rig, MOVE)
    assert rig.dispatched == [first.request]
    assert rig.controller.queued_request_ids == (launch.request_id, move.request_id)

    # Editing the routine afterwards must not retarget accepted commands.
    rig.routine.map_id = "another_map"
    rig.routine.waypoint_id = "another_waypoint"
    succeed(rig, first)
    assert rig.dispatched[-1] == launch.request
    succeed(rig, launch)
    assert rig.dispatched[-1] == move.request
    assert move.request.command.map_name == "plant"
    assert move.request.command.waypoint_name == "inspection_start"
    succeed(rig, move)
    assert rig.controller.active_request_id == ""


@pytest.mark.parametrize("operation", [LAUNCH, MOVE])
def test_queued_navigation_cancellation_does_not_dispatch_or_interrupt_active_move(rig, operation):
    first = submit(rig, OperationalIntent.INTENT_STOW_ARM)
    queued = submit(rig, operation)
    rig.app.cancel("operator_ui", queued.request_id)

    assert rig.dispatched == [first.request]
    assert rig.controller.active_request_id == first.request_id
    assert rig.controller.queued_request_ids == ()
    assert rig.states[-1].operation.request_id == queued.request_id
    assert rig.states[-1].state is CommandControllerState.CANCELLED
    succeed(rig, first)
    assert rig.dispatched == [first.request]
