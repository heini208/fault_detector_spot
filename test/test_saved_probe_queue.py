"""Saved probe operations share the serialized operational command queue."""

from dataclasses import replace
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
from fault_detector_spot.application.controllers.sensor_attachment_controller import (
    SensorAttachmentController,
)
from fault_detector_spot.application.coordinators.probe_setup_coordinator import (
    ProbeSetupCoordinator,
)
from fault_detector_spot.inspection.model.models import PreApproachPathPoint
from fault_detector_spot.inspection.repository.object_repository import ObjectRepository
from fault_detector_spot.inspection.repository.sensor_attachment_state_store import (
    SensorAttachmentStateStore,
)
from fault_detector_spot.inspection.repository.sensor_repository import SensorRepository
from test_probe_execution_target import inspection_object, pose
from test_sensor_attachment_controller import sensor_definition


SAVED_OPERATIONS = (
    pytest.param(
        OperationalIntent.INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH,
        "",
        CommandID.MOVE_SAFE_APPROACH,
        id="routine-safe-approach",
    ),
    pytest.param(
        OperationalIntent.INTENT_MOVE_SAVED_PROBE_SAFE_APPROACH,
        "point_1",
        CommandID.MOVE_ARM_TO_TAG,
        id="saved-safe-approach",
    ),
    pytest.param(
        OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH,
        "point_1",
        CommandID.FOLLOW_MOVE_TO_TAG_PATH,
        id="saved-aligned-path",
    ),
    pytest.param(
        OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE,
        "point_1",
        CommandID.MOVE_CLOSE_TO_SURFACE,
        id="saved-surface-approach",
    ),
    pytest.param(
        OperationalIntent.INTENT_MOVE_SAVED_CUSTOM_PROBE_PATH,
        "custom_1",
        CommandID.FOLLOW_MOVE_TO_TAG_PATH,
        id="saved-custom-final-path",
    ),
    pytest.param(
        OperationalIntent.INTENT_EXECUTE_PROBE_POINT,
        "point_1",
        CommandID.EXECUTE_PROBE_POINT,
        id="execute-and-record",
    ),
)


@pytest.fixture
def queue_rig(tmp_path):
    definition = inspection_object()
    routine = definition.get_routine("scan")
    point = routine.get_probe_point("point_1")
    point.pre_approach_path = [
        PreApproachPathPoint("travel", pose(x=0.2), 0.025, 0.6),
    ]
    routine.probe_points.append(replace(
        point,
        probe_point_id="custom_1",
        display_name="Custom point",
        fully_custom=True,
        final_probe_pose_object=pose(x=0.05),
        final_probe_path=[
            PreApproachPathPoint("final travel", pose(x=0.07), 0.015, 0.15),
        ],
    ))
    objects = ObjectRepository(tmp_path / "objects")
    objects.save(definition)

    dispatched = []
    commands = CommandController(dispatch_request=dispatched.append)
    sensors = SensorRepository(tmp_path / "sensors")
    sensors.create(sensor_definition())
    sensors.create(sensor_definition("thermal_01"))
    attachments = SensorAttachmentController(
        sensors,
        SensorAttachmentStateStore(tmp_path / "attachment.yaml"),
        commands,
    )
    pending = attachments.select_sensor("bmm150_01")
    attachments.confirm_sensor("bmm150_01", pending.attachment_revision)
    commands.add_request_preparer(attachments.prepare_request)

    state_source = SimpleNamespace(
        reference_tag=Mock(side_effect=AssertionError(
            "Queued saved motions must not resolve a live reference tag"
        )),
        validate_aligned_probe_distance=Mock(return_value=0.05),
    )
    app = ApplicationController(commands)
    probe = ProbeSetupCoordinator(
        app.setup_coordinator,
        reference_repository=SimpleNamespace(object_repository=objects),
        sensor_attachment_controller=attachments,
        motion_state_source=state_source,
    )
    app.attach_probe_setup(probe)
    states = []
    app.add_status_listener(states.append)
    return SimpleNamespace(
        app=app,
        commands=commands,
        probe=probe,
        objects=objects,
        attachments=attachments,
        state_source=state_source,
        dispatched=dispatched,
        states=states,
    )


def prepare(rig, intent_id, point_id=""):
    intent = OperationalIntent()
    intent.intent = intent_id
    intent.object_id = "motor_a"
    intent.routine_id = "scan"
    intent.probe_point_id = point_id
    if intent_id == OperationalIntent.INTENT_EXECUTE_PROBE_POINT:
        intent.duration_sec = 1.5
    return rig.app.prepare_operation(intent, "operator_ui")


def submit(rig, intent_id, point_id=""):
    operation = prepare(rig, intent_id, point_id)
    rig.app.submit(operation)
    return operation


def succeed(rig, operation, remaining_steps=0, detail=""):
    assert rig.commands.handle_execution_status(CommandExecutionStatus(
        request_id=operation.request_id,
        state=CommandControllerState.SUCCEEDED,
        buffered_command_count=remaining_steps,
        detail=detail,
    ))


def dispatched_ids(rig):
    return [request.request_id for request in rig.dispatched]


def test_saved_operations_queue_behind_active_and_pending_work_and_each_other(queue_rig):
    rig = queue_rig
    first = submit(rig, OperationalIntent.INTENT_STOW_ARM)
    second = submit(rig, OperationalIntent.INTENT_STAND_UP)
    saved = [
        submit(rig, intent_id, point_id)
        for intent_id, point_id, _ in (case.values for case in SAVED_OPERATIONS)
    ]
    operations = [first, second, *saved]

    assert dispatched_ids(rig) == [first.request_id]
    assert rig.commands.queued_request_ids == tuple(
        operation.request_id for operation in operations[1:]
    )
    assert [operation.request.command.command_id for operation in saved] == [
        case.values[2] for case in SAVED_OPERATIONS
    ]
    assert len(saved[2].request.command.pre_approach_offsets) == 1
    assert len(saved[4].request.command.pre_approach_offsets) == 1
    rig.state_source.reference_tag.assert_not_called()
    rig.state_source.validate_aligned_probe_distance.assert_called_once_with(
        "bmm150_01", 0.10,
    )

    for index, operation in enumerate(operations):
        assert rig.commands.active_request_id == operation.request_id
        assert dispatched_ids(rig) == [
            item.request_id for item in operations[:index + 1]
        ]
        succeed(rig, operation)

    assert rig.commands.active_request_id == ""
    assert rig.commands.queued_request_ids == ()
    assert all(
        request.command.motion_sensor_id == "bmm150_01"
        for request in rig.dispatched[2:]
    )


@pytest.mark.parametrize("intent_id,point_id,command_id", SAVED_OPERATIONS)
def test_queued_saved_operation_cancels_without_dispatch(
    queue_rig, intent_id, point_id, command_id,
):
    rig = queue_rig
    active = submit(rig, OperationalIntent.INTENT_STOW_ARM)
    pending = submit(rig, OperationalIntent.INTENT_STAND_UP)
    saved = submit(rig, intent_id, point_id)

    assert saved.request.command.command_id is command_id
    assert rig.app.cancel("operator_ui", saved.request_id) == saved.request_id
    assert rig.commands.queued_request_ids == (pending.request_id,)
    assert rig.states[-1].operation.request_id == saved.request_id
    assert rig.states[-1].state is CommandControllerState.CANCELLED
    succeed(rig, active)
    succeed(rig, pending)

    assert dispatched_ids(rig) == [active.request_id, pending.request_id]
    assert rig.commands.active_request_id == ""


@pytest.mark.parametrize(
    "intent_id,point_id",
    [
        (OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH, "point_1"),
        (OperationalIntent.INTENT_MOVE_SAVED_CUSTOM_PROBE_PATH, "custom_1"),
        (OperationalIntent.INTENT_EXECUTE_PROBE_POINT, "point_1"),
        (OperationalIntent.INTENT_EXECUTE_PROBE_POINT, "custom_1"),
    ],
)
def test_multistep_saved_operation_retains_lane_until_all_steps_complete(
    queue_rig, intent_id, point_id,
):
    rig = queue_rig
    operation = submit(rig, intent_id, point_id)
    following = submit(rig, OperationalIntent.INTENT_STOW_ARM)

    for remaining_steps in (3, 2, 1):
        succeed(rig, operation, remaining_steps)
        assert rig.commands.active_request_id == operation.request_id
        assert rig.commands.queued_request_ids == (following.request_id,)
        assert dispatched_ids(rig) == [operation.request_id]
        assert rig.states[-1].operation.request_id == operation.request_id
        assert rig.states[-1].state is CommandControllerState.RUNNING

    succeed(rig, operation)
    assert dispatched_ids(rig) == [operation.request_id, following.request_id]
    assert rig.commands.active_request_id == following.request_id


def test_execute_and_record_holds_following_work_until_execution_completes(queue_rig):
    rig = queue_rig
    operation = submit(rig, OperationalIntent.INTENT_EXECUTE_PROBE_POINT, "point_1")
    following = submit(rig, OperationalIntent.INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH)

    for detail in ("Moving to probe", "Recording measurement", "Retracting probe"):
        assert rig.commands.handle_execution_status(CommandExecutionStatus(
            operation.request_id,
            CommandControllerState.RUNNING,
            detail,
        ))
        assert dispatched_ids(rig) == [operation.request_id]
        assert rig.commands.queued_request_ids == (following.request_id,)

    succeed(rig, operation)
    assert dispatched_ids(rig) == [operation.request_id, following.request_id]


@pytest.mark.parametrize("intent_id,point_id,command_id", SAVED_OPERATIONS)
def test_queued_saved_operations_still_require_confirmed_attachment(
    queue_rig, intent_id, point_id, command_id,
):
    rig = queue_rig
    rig.attachments.select_sensor("thermal_01")
    active = submit(rig, OperationalIntent.INTENT_STOW_ARM)

    with pytest.raises(RuntimeError, match="confirmation is pending"):
        submit(rig, intent_id, point_id)

    assert rig.commands.queued_request_ids == ()
    assert dispatched_ids(rig) == [active.request_id]


def test_saved_queue_keeps_attachment_binding_and_blocks_sensor_changes(queue_rig):
    rig = queue_rig
    prepared = prepare(rig, OperationalIntent.INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH)
    pending = rig.attachments.select_sensor("thermal_01")
    rig.attachments.confirm_sensor("thermal_01", pending.attachment_revision)
    active = submit(rig, OperationalIntent.INTENT_STOW_ARM)

    with pytest.raises(RuntimeError, match="prepared for attachment"):
        rig.app.submit(prepared)

    saved = submit(rig, OperationalIntent.INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH)
    with pytest.raises(RuntimeError, match="active or queued"):
        rig.attachments.select_sensor("bmm150_01")

    assert rig.commands.queued_request_ids == (saved.request_id,)
    succeed(rig, active)
    assert rig.dispatched[-1].command.motion_sensor_id == "thermal_01"


def test_aligned_path_queue_admission_preserves_sensor_clearance_validation(queue_rig):
    rig = queue_rig
    active = submit(rig, OperationalIntent.INTENT_STOW_ARM)
    rig.state_source.validate_aligned_probe_distance.side_effect = ValueError(
        "Aligned pre-approach is too close for registered hand depth"
    )

    with pytest.raises(ValueError, match="too close"):
        submit(rig, OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH, "point_1")

    rig.state_source.validate_aligned_probe_distance.assert_called_once_with(
        "bmm150_01", 0.10,
    )
    assert rig.commands.queued_request_ids == ()
    assert dispatched_ids(rig) == [active.request_id]
