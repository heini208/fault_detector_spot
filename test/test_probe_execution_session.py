"""Tests for frozen probe execution configuration and state."""

from dataclasses import replace

import pytest
from builtin_interfaces.msg import Time

from fault_detector_spot.behaviour_tree.commands import (
    execute_probe_point_command,
)
from fault_detector_spot.inspection.models import (
    InspectionObject,
    InspectionRoutine,
    PoseData,
    ProbePoint,
    ReferenceTag,
    ReferenceView,
    Vector3Data,
)
from fault_detector_spot.inspection.object_repository import ObjectRepository
from fault_detector_spot.inspection.probe_execution_session import (
    ProbeExecutionSession,
    ProbeExecutionStage,
)
from fault_detector_spot.inspection.sensor_models import SensorDefinition
from fault_detector_spot.inspection.sensor_repository import SensorRepository


ExecuteProbePointCommand = (
    execute_probe_point_command.ExecuteProbePointCommand
)


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


def repositories(tmp_path):
    objects = ObjectRepository(tmp_path / "objects")
    sensors = SensorRepository(tmp_path / "sensors")
    objects.create(inspection_object())
    sensors.create(
        SensorDefinition(
            sensor_id="bmm150_01",
            display_name="BMM150",
            hand_to_probe=pose(x=0.20),
        )
    )
    return objects, sensors


def command():
    return ExecuteProbePointCommand(
        Time(),
        "motor_a",
        "scan",
        "point_1",
    )


def load_session(tmp_path):
    objects, sensors = repositories(tmp_path)
    selection = command()
    session = ProbeExecutionSession.load(
        selection.object_id,
        selection.routine_id,
        selection.probe_point_id,
        objects,
        sensors,
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

    session.fail("arm motion failed", recovery_required=True)

    assert session.recovery_required is True
    assert session.is_terminal is False
    assert session.detail == "arm motion failed"
    assert session.stopped_stage == (
        ProbeExecutionStage.MOVING_SAFE_APPROACH
    )

    session.complete_recovery()

    assert session.stage == ProbeExecutionStage.FAILED
    assert session.is_terminal is True


def test_cancel_after_contact_preserves_cancelled_outcome(tmp_path):
    session, _ = load_session(tmp_path)
    session.advance(ProbeExecutionStage.WAITING_FOR_OBJECT)
    session.advance(ProbeExecutionStage.MOVING_SAFE_APPROACH)
    session.advance(ProbeExecutionStage.MOVING_ALIGNED_PREAPPROACH)
    session.advance(ProbeExecutionStage.CONVERGING_SURFACE_DISTANCE)

    session.cancel("operator cancelled", recovery_required=True)
    session.complete_recovery()

    assert session.stage == ProbeExecutionStage.CANCELLED
    assert session.detail == "operator cancelled"
