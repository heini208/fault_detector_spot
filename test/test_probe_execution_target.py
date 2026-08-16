"""Tests for resolving saved probe targets without commanding motion."""

import math

import pytest

from fault_detector_spot.inspection.model.models import (
    InspectionObject,
    InspectionRoutine,
    PoseData,
    ProbePoint,
    QuaternionData,
    ReferenceTag,
    ReferenceView,
    Vector3Data,
)
from fault_detector_spot.inspection.execution.probe_execution_target import (
    resolve_probe_execution_target,
)
from fault_detector_spot.inspection.model.sensor_models import SensorDefinition


def pose(x=0.0, y=0.0, z=0.0, orientation=None):
    return PoseData(
        position=Vector3Data(x=x, y=y, z=z),
        orientation=orientation or QuaternionData.identity(),
    )


def yaw_quaternion(degrees):
    radians = math.radians(degrees) * 0.5
    return QuaternionData(
        x=0.0,
        y=0.0,
        z=math.sin(radians),
        w=math.cos(radians),
    )


def inspection_object(sensor_id="bmm150_01"):
    view = ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
        reference_dataset_path=(
            "reference_datasets/scan/set_1/slot1_hand"
        ),
        view_id="slot1_hand",
        camera_id="hand",
        slot_index=0,
    )
    point = ProbePoint(
        probe_point_id="point_1",
        display_name="Point 1",
        safe_approach_pose_object=pose(x=0.30),
        probe_pose_object=pose(
            x=0.03,
            orientation=yaw_quaternion(180.0),
        ),
        target_surface_distance_m=0.03,
        position_tolerance_m=0.005,
        orientation_tolerance_rad=0.05,
        measurement_duration_sec=1.5,
        aligned_preapproach_distance_m=0.10,
        reference_view_id="slot1_hand",
    )
    routine = InspectionRoutine(
        routine_id="scan",
        display_name="Scan",
        sensor_id=sensor_id,
        reference_views=[view],
        probe_points=[point],
    )
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(
            tag_id=2,
            tag_family="36h11",
        ),
        routines=[routine],
    )


def sensor(sensor_id="bmm150_01", hand_to_probe=None):
    return SensorDefinition(
        sensor_id=sensor_id,
        display_name="BMM150 01",
        hand_to_probe=hand_to_probe or pose(x=0.20),
    )


def test_resolves_probe_and_hand_targets_in_execution_frame():
    target = resolve_probe_execution_target(
        inspection_object(),
        routine_id="scan",
        probe_point_id="point_1",
        sensor_definition=sensor(),
        object_pose_execution=pose(x=1.0, y=2.0, z=0.5),
    )

    assert target.object_id == "motor_a"
    assert target.routine_id == "scan"
    assert target.probe_point_id == "point_1"
    assert target.sensor_id == "bmm150_01"
    assert target.probe_frame == "bmm150_01_probe"
    assert target.execution_frame == "odom"
    assert target.safe_approach_probe_pose_execution.position.x == (
        pytest.approx(1.30)
    )
    assert target.safe_approach_hand_pose_execution.position.x == (
        pytest.approx(1.10)
    )
    assert (
        target.aligned_preapproach_probe_pose_execution.position.x
        == pytest.approx(1.10)
    )
    assert (
        target.aligned_preapproach_hand_pose_execution.position.x
        == pytest.approx(1.30)
    )
    assert target.nominal_probe_pose_execution.position.x == pytest.approx(
        1.03
    )
    assert target.nominal_hand_pose_execution.position.x == pytest.approx(
        1.23
    )
    assert target.inward_direction_execution.x == pytest.approx(-1.0)
    assert target.inward_direction_execution.y == pytest.approx(0.0)
    assert target.inward_direction_execution.z == pytest.approx(0.0)
    assert target.target_surface_distance_m == pytest.approx(0.03)
    assert target.aligned_preapproach_distance_m == pytest.approx(0.10)
    assert target.measurement_duration_sec == pytest.approx(1.5)


def test_live_object_rotation_rotates_positions_and_approach_axis():
    target = resolve_probe_execution_target(
        inspection_object(),
        routine_id="scan",
        probe_point_id="point_1",
        sensor_definition=sensor(),
        object_pose_execution=pose(
            x=1.0,
            y=2.0,
            orientation=yaw_quaternion(90.0),
        ),
    )

    safe_probe = target.safe_approach_probe_pose_execution.position
    safe_hand = target.safe_approach_hand_pose_execution.position
    aligned_probe = (
        target.aligned_preapproach_probe_pose_execution.position
    )
    inward = target.inward_direction_execution
    assert safe_probe.x == pytest.approx(1.0)
    assert safe_probe.y == pytest.approx(2.30)
    assert safe_hand.x == pytest.approx(1.0)
    assert safe_hand.y == pytest.approx(2.10)
    assert aligned_probe.x == pytest.approx(1.0)
    assert aligned_probe.y == pytest.approx(2.10)
    assert inward.x == pytest.approx(0.0, abs=1e-12)
    assert inward.y == pytest.approx(-1.0)


def test_rotated_sensor_calibration_changes_required_hand_pose():
    calibration = pose(
        x=0.10,
        y=0.0,
        orientation=yaw_quaternion(90.0),
    )

    target = resolve_probe_execution_target(
        inspection_object(),
        routine_id="scan",
        probe_point_id="point_1",
        sensor_definition=sensor(hand_to_probe=calibration),
        object_pose_execution=PoseData.identity(),
    )

    hand = target.nominal_hand_pose_execution
    assert hand.position.x == pytest.approx(0.03)
    assert hand.position.y == pytest.approx(-0.10)
    assert hand.orientation.z == pytest.approx(math.sqrt(0.5))
    assert hand.orientation.w == pytest.approx(math.sqrt(0.5))


def test_active_sensor_calibration_may_differ_from_saved_routine_sensor():
    target = resolve_probe_execution_target(
        inspection_object(sensor_id="saved_sensor"),
        routine_id="scan",
        probe_point_id="point_1",
        sensor_definition=sensor("active_sensor"),
        object_pose_execution=PoseData.identity(),
    )

    assert target.sensor_id == "active_sensor"
    assert target.probe_frame == "active_sensor_probe"


@pytest.mark.parametrize(
    "routine_id,probe_point_id,message",
    [
        ("missing", "point_1", "Unknown routine missing"),
        ("scan", "missing", "Unknown probe point missing"),
    ],
)
def test_rejects_unknown_selection(routine_id, probe_point_id, message):
    with pytest.raises(ValueError, match=message):
        resolve_probe_execution_target(
            inspection_object(),
            routine_id=routine_id,
            probe_point_id=probe_point_id,
            sensor_definition=sensor(),
            object_pose_execution=PoseData.identity(),
        )


@pytest.mark.parametrize("execution_frame", ["", " odom", "odom "])
def test_rejects_invalid_execution_frame(execution_frame):
    with pytest.raises(ValueError, match="Execution frame"):
        resolve_probe_execution_target(
            inspection_object(),
            routine_id="scan",
            probe_point_id="point_1",
            sensor_definition=sensor(),
            object_pose_execution=PoseData.identity(),
            execution_frame=execution_frame,
        )
