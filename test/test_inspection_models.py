"""Tests for strict inspection and map models."""

import pytest

from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    InspectionObject,
    InspectionRoutine,
    PoseData,
    ProbePoint,
    ReferenceTag,
    ReferenceView,
)
from fault_detector_spot.mapping.model.models import (
    MapDefinition,
    ObjectApproach,
    Waypoint,
)


def make_probe(probe_point_id="point_a") -> ProbePoint:
    return ProbePoint(
        probe_point_id=probe_point_id,
        display_name=probe_point_id,
        safe_approach_pose_object=PoseData.identity(),
        aligned_preapproach_pose_object=PoseData.identity(),
        target_surface_distance_m=0.01,
        position_tolerance_m=0.005,
        orientation_tolerance_rad=0.05,
        measurement_duration_sec=1.0,
        aligned_preapproach_distance_m=0.08,
        reference_pixel=ImagePoint(u=100, v=200),
        reference_view_id="slot1_hand",
    )


def make_routine(routine_id="magnetic_scan") -> InspectionRoutine:
    return InspectionRoutine(
        routine_id=routine_id,
        display_name=routine_id,
        reference_views=[ReferenceView(
            controlled_frame_pose_object=PoseData.identity(),
            controlled_frame="hand_color_image_sensor",
            reference_dataset_path="reference/magnetic_scan",
            view_id="slot1_hand",
            camera_id="hand",
            slot_index=0,
        )],
        probe_points=[make_probe()],
    )


def make_object() -> InspectionObject:
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(tag_id=23, tag_family="36h11"),
        routines=[
            make_routine("magnetic_scan"),
            make_routine("temperature_scan"),
        ],
    )


def test_inspection_object_round_trip_preserves_routine_order():
    original = make_object()
    restored = InspectionObject.from_dict(original.to_dict())
    restored.validate()
    assert restored == original
    assert [routine.routine_id for routine in restored.routines] == [
        "magnetic_scan",
        "temperature_scan",
    ]
    assert "sensor_id" not in restored.get_routine(
        "temperature_scan"
    ).to_dict()


def test_uncaptured_routine_round_trip_preserves_empty_reference_views():
    routine = InspectionRoutine(
        routine_id="magnetic_scan",
        display_name="Magnetic scan",
    )
    restored = InspectionRoutine.from_dict(routine.to_dict())
    restored.validate()
    assert restored == routine
    assert restored.to_dict()["reference_views"] == []


def test_probe_points_require_a_captured_reference_view():
    routine = InspectionRoutine(
        routine_id="magnetic_scan",
        display_name="Magnetic scan",
        probe_points=[make_probe()],
    )
    with pytest.raises(ValueError, match="requires a reference view"):
        routine.validate()


def test_probe_point_round_trip_preserves_execution_geometry():
    original = make_probe()
    original.safe_approach_pose_object.position.x = 0.30
    original.aligned_preapproach_pose_object.position.x = 0.08
    restored = ProbePoint.from_dict(original.to_dict())
    restored.validate()
    assert restored == original
    assert restored.safe_approach_pose_object.position.x == 0.30
    assert restored.aligned_preapproach_pose_object.position.x == 0.08
    assert restored.target_surface_distance_m == 0.01
    assert restored.aligned_preapproach_distance_m == 0.08
    serialized = restored.to_dict()
    assert "probe_pose_object" not in serialized
    assert "aligned_preapproach_pose_object" in serialized


@pytest.mark.parametrize(
    "distance",
    [0.0, -0.01, 0.059, float("inf"), float("nan")],
)
def test_probe_point_requires_safe_absolute_preapproach_distance(distance):
    probe_point = make_probe()
    probe_point.aligned_preapproach_distance_m = distance
    with pytest.raises(ValueError):
        probe_point.validate()


def test_probe_point_accepts_exact_minimum_distance_separation():
    probe_point = make_probe()
    probe_point.target_surface_distance_m = 0.10
    probe_point.aligned_preapproach_distance_m = 0.15
    probe_point.validate()


def test_probe_point_rejects_invalid_approved_pose():
    probe_point = make_probe()
    probe_point.aligned_preapproach_pose_object.orientation.w = 2.0
    with pytest.raises(ValueError, match="Quaternion must be normalized"):
        probe_point.validate()


@pytest.mark.parametrize(
    "field_name",
    ["aligned_preapproach_pose_object", "aligned_preapproach_distance_m"],
)
def test_incomplete_probe_point_format_is_rejected(field_name):
    serialized = make_probe().to_dict()
    serialized.pop(field_name)
    with pytest.raises(KeyError):
        ProbePoint.from_dict(serialized)


def test_old_nominal_probe_pose_format_is_rejected():
    serialized = make_probe().to_dict()
    serialized["probe_pose_object"] = PoseData.identity().to_dict()
    serialized.pop("aligned_preapproach_pose_object")

    with pytest.raises(KeyError):
        ProbePoint.from_dict(serialized)


def test_routine_reference_view_requires_persisted_dataset():
    routine = make_routine()
    routine.reference_views[0].reference_dataset_path = None
    with pytest.raises(ValueError, match="requires a dataset path"):
        routine.validate()


def test_duplicate_routine_id_is_rejected():
    inspection_object = make_object()
    inspection_object.routines[1].routine_id = "magnetic_scan"
    with pytest.raises(ValueError, match="Duplicate routine ID"):
        inspection_object.validate()


def test_duplicate_probe_point_id_is_rejected():
    routine = make_routine()
    routine.probe_points.append(make_probe())
    with pytest.raises(ValueError, match="Duplicate probe point ID"):
        routine.validate()


def test_non_normalized_quaternion_is_rejected():
    routine = make_routine()
    routine.reference_views[0].controlled_frame_pose_object = PoseData.identity()
    routine.reference_views[0].controlled_frame_pose_object.orientation.w = 2.0
    with pytest.raises(ValueError, match="Quaternion must be normalized"):
        routine.validate()


def test_old_object_format_is_rejected():
    with pytest.raises(KeyError):
        InspectionObject.from_dict({"id": "motor_a", "tag_id": 23})


def test_map_validates_internal_and_external_references():
    definition = MapDefinition(
        map_id="laboratory",
        display_name="Laboratory",
        waypoints=[Waypoint(
            waypoint_id="motor_front",
            display_name="Motor front",
            pose_map=PoseData.identity(),
        )],
        object_approaches=[ObjectApproach(
            approach_id="motor_a_front",
            display_name="Motor A front",
            object_id="motor_a",
            waypoint_id="motor_front",
        )],
    )
    definition.validate()
    definition.validate_object_references({"motor_a"})
    with pytest.raises(ValueError, match="Unknown inspection object"):
        definition.validate_object_references(set())
