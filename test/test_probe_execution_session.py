"""Tests for frozen probe execution configuration and state."""

from dataclasses import FrozenInstanceError, replace

import pytest
from builtin_interfaces.msg import Time

from fault_detector_spot.inspection.commands import (
    ExecuteProbePointCommand,
)
from fault_detector_spot.inspection.model.models import (
    InspectionObject,
    InspectionRoutine,
    PoseData,
    ProbePoint,
    ReferenceTag,
    ReferenceView,
    Vector3Data,
)
from fault_detector_spot.inspection.repository.object_repository import (
    ObjectRepository,
)
from fault_detector_spot.inspection.execution.probe_execution_session import (
    ProbeExecutionSession,
    ProbeExecutionStage,
)
from fault_detector_spot.inspection.model.sensor_models import SensorDefinition


def pose(x=0.0, y=0.0, z=0.0):
    value = PoseData.identity()
    value.position = Vector3Data(x=x, y=y, z=z)
    return value


def probe_point(safe_x=0.30):
    return ProbePoint(
        probe_point_id="point_1",
        display_name="Point 1",
        safe_approach_pose_object=pose(x=safe_x),
        probe_pose_object=pose(x=0.03),
        target_surface_distance_m=0.03,
        position_tolerance_m=0.005,
        orientation_tolerance_rad=0.05,
        aligned_preapproach_distance_m=0.10,
        reference_view_id="slot1_hand",
        measurement_duration_sec=1.0,
    )


def inspection_object(point=None):
    view = ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
        reference_dataset_path="datasets/slot1_hand",
        view_id="slot1_hand",
        camera_id="hand",
        slot_index=0,
    )
    routine = InspectionRoutine(
        routine_id="scan",
        display_name="Scan",
        sensor_id="bmm150_01",
        reference_views=[view],
        probe_points=[point or probe_point()],
    )
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(tag_id=2, tag_family="36h11"),
        routines=[routine],
    )


def repository_and_sensor(tmp_path, sensor_id="bmm150_01"):
    objects = ObjectRepository(tmp_path / "objects")
    objects.create(inspection_object())
    sensor = SensorDefinition(
        sensor_id=sensor_id,
        display_name="Active sensor",
        hand_to_probe=pose(x=0.20),
    )
    return objects, sensor


def command():
    return ExecuteProbePointCommand(
        Time(),
        "motor_a",
        "scan",
        "point_1",
    )


def load_session(tmp_path, sensor_id="bmm150_01"):
    objects, sensor = repository_and_sensor(tmp_path, sensor_id)
    selection = command()
    session = ProbeExecutionSession.load(
        selection.object_id,
        selection.routine_id,
        selection.probe_point_id,
        objects,
        sensor,
    )
    return session, objects


def test_loaded_configuration_is_not_changed_by_later_repository_edits(
    tmp_path,
):
    session, objects = load_session(tmp_path)
    changed = replace(probe_point(), safe_approach_pose_object=pose(x=0.80))
    objects.replace_probe_point("motor_a", "scan", changed)

    target = session.configuration.resolve_target(PoseData.identity())

    assert (
        target.safe_approach_probe_pose_execution.position.x
        == pytest.approx(0.30)
    )


def test_loaded_configuration_freezes_active_sensor_not_routine_sensor(
    tmp_path,
):
    session, _ = load_session(tmp_path, sensor_id="active_sensor")

    assert session.configuration.sensor_id == "active_sensor"
    assert session.configuration.hand_to_probe.position == (
        0.20,
        0.0,
        0.0,
    )



def test_loaded_configuration_can_freeze_bare_hand_identity_geometry(tmp_path):
    objects, _ = repository_and_sensor(tmp_path)

    session = ProbeExecutionSession.load(
        "motor_a",
        "scan",
        "point_1",
        objects,
        None,
    )
    target = session.configuration.resolve_target(PoseData.identity())

    assert session.configuration.sensor_id == ""
    assert session.configuration.hand_to_probe.position == (0.0, 0.0, 0.0)
    assert session.configuration.hand_to_probe.orientation == (
        0.0,
        0.0,
        0.0,
        1.0,
    )
    assert target.probe_frame == "hand"
    assert target.nominal_probe_pose_execution == (
        target.nominal_hand_pose_execution
    )


def test_loaded_configuration_has_no_mutable_definition_graph(tmp_path):
    session, _ = load_session(tmp_path)
    configuration = session.configuration

    assert not hasattr(configuration, "inspection_object")
    assert configuration.reference_tag_id == 2
    with pytest.raises(FrozenInstanceError):
        configuration.object_id = "changed"
    with pytest.raises(TypeError):
        configuration.safe_approach_pose_object.position[0] = 0.8


def test_normal_execution_order_is_explicit(tmp_path):
    session, _ = load_session(tmp_path)
    expected = [
        ProbeExecutionStage.WAITING_FOR_OBJECT,
        ProbeExecutionStage.MOVING_SAFE_APPROACH,
        ProbeExecutionStage.MOVING_ALIGNED_PREAPPROACH,
        ProbeExecutionStage.CONVERGING_SURFACE_DISTANCE,
        ProbeExecutionStage.MEASURING,
        ProbeExecutionStage.RETRACTING_ALIGNED_PREAPPROACH,
        ProbeExecutionStage.RETRACTING_SAFE_APPROACH,
        ProbeExecutionStage.SUCCEEDED,
    ]

    for stage in expected:
        session.advance(stage)

    assert session.is_terminal is True
    assert session.stage == ProbeExecutionStage.SUCCEEDED


def test_execution_cannot_skip_a_stage(tmp_path):
    session, _ = load_session(tmp_path)

    with pytest.raises(RuntimeError, match="Expected stage"):
        session.advance(ProbeExecutionStage.MOVING_SAFE_APPROACH)


def test_failure_reports_only_after_required_recovery(tmp_path):
    session, _ = load_session(tmp_path)
    session.advance(ProbeExecutionStage.WAITING_FOR_OBJECT)
    session.advance(ProbeExecutionStage.MOVING_SAFE_APPROACH)
    session.advance(ProbeExecutionStage.MOVING_ALIGNED_PREAPPROACH)
    session.advance(ProbeExecutionStage.CONVERGING_SURFACE_DISTANCE)

    session.fail("arm motion failed")

    assert session.recovery_required is True
    assert session.is_terminal is False
    assert session.detail == "arm motion failed"
    assert session.stopped_stage == (
        ProbeExecutionStage.CONVERGING_SURFACE_DISTANCE
    )
    assert session.stage == (
        ProbeExecutionStage.RECOVERING_ALIGNED_PREAPPROACH
    )

    session.complete_recovery_stage()
    assert session.stage == ProbeExecutionStage.RECOVERING_SAFE_APPROACH
    session.complete_recovery_stage()

    assert session.stage == ProbeExecutionStage.FAILED
    assert session.is_terminal is True


def test_cancel_after_contact_preserves_cancelled_outcome(tmp_path):
    session, _ = load_session(tmp_path)
    session.advance(ProbeExecutionStage.WAITING_FOR_OBJECT)
    session.advance(ProbeExecutionStage.MOVING_SAFE_APPROACH)
    session.advance(ProbeExecutionStage.MOVING_ALIGNED_PREAPPROACH)
    session.advance(ProbeExecutionStage.CONVERGING_SURFACE_DISTANCE)

    session.cancel("operator cancelled")
    assert session.requested_outcome == ProbeExecutionStage.CANCELLED
    session.complete_recovery_stage()
    assert session.stage == ProbeExecutionStage.RECOVERING_SAFE_APPROACH
    assert session.requested_outcome == ProbeExecutionStage.CANCELLED
    session.complete_recovery_stage()

    assert session.stage == ProbeExecutionStage.CANCELLED
    assert session.detail == "operator cancelled"


def test_waiting_failure_does_not_command_recovery(tmp_path):
    session, _ = load_session(tmp_path)
    session.advance(ProbeExecutionStage.WAITING_FOR_OBJECT)

    session.fail("object pose unavailable")

    assert session.stage == ProbeExecutionStage.FAILED
    assert not session.recovery_required


def test_caller_cannot_skip_stage_derived_recovery(tmp_path):
    session, _ = load_session(tmp_path)
    for stage in (
        ProbeExecutionStage.WAITING_FOR_OBJECT,
        ProbeExecutionStage.MOVING_SAFE_APPROACH,
        ProbeExecutionStage.MOVING_ALIGNED_PREAPPROACH,
        ProbeExecutionStage.CONVERGING_SURFACE_DISTANCE,
        ProbeExecutionStage.MEASURING,
    ):
        session.advance(stage)

    session.cancel("operator cancelled")

    with pytest.raises(RuntimeError, match="Cannot advance"):
        session.advance(ProbeExecutionStage.CANCELLED)
    assert session.stage == (
        ProbeExecutionStage.RECOVERING_ALIGNED_PREAPPROACH
    )


def test_recovery_failure_preserves_original_cancel_reason(tmp_path):
    session, _ = load_session(tmp_path)
    for stage in (
        ProbeExecutionStage.WAITING_FOR_OBJECT,
        ProbeExecutionStage.MOVING_SAFE_APPROACH,
        ProbeExecutionStage.MOVING_ALIGNED_PREAPPROACH,
        ProbeExecutionStage.CONVERGING_SURFACE_DISTANCE,
    ):
        session.advance(stage)
    session.cancel("operator cancelled")

    session.recovery_failed("arm unavailable")

    assert session.stage == ProbeExecutionStage.FAILED
    assert session.requested_outcome == ProbeExecutionStage.CANCELLED
    assert session.detail == (
        "operator cancelled; recovery failed: arm unavailable"
    )
