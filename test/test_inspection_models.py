"""Tests for the strict inspection and map models."""

import pytest

from fault_detector_spot.inspection.models import (
    ImagePoint,
    InspectionObject,
    InspectionRoutine,
    MapDefinition,
    ObjectApproach,
    PoseData,
    ProbePoint,
    ReferenceTag,
    ReferenceView,
    Waypoint,
)


def make_probe(probe_point_id="point_a") -> ProbePoint:
    """Create a valid probe point."""
    return ProbePoint(
        probe_point_id=probe_point_id,
        display_name=probe_point_id,
        safe_approach_pose_object=PoseData.identity(),
        target_surface_distance_m=0.01,
        position_tolerance_m=0.005,
        orientation_tolerance_rad=0.05,
        measurement_duration_sec=1.0,
        reference_pixel=ImagePoint(u=100, v=200),
        reference_view_id="slot1_hand",
    )


def make_routine(
    routine_id="magnetic_scan",
) -> InspectionRoutine:
    """Create a valid inspection routine."""
    return InspectionRoutine(
        routine_id=routine_id,
        display_name=routine_id,
        sensor_id="bmm150",
        probe_frame="sensor_tip",
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
    """Create an object with two independent routines."""
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(
            tag_id=23,
            tag_family="36h11",
        ),
        routines=[
            make_routine("magnetic_scan"),
            make_routine("temperature_scan"),
        ],
    )


def test_inspection_object_round_trip_preserves_routine_order():
    """Serialized routines and probe order remain deterministic."""
    original = make_object()
    restored = InspectionObject.from_dict(original.to_dict())

    restored.validate()

    assert restored == original
    assert [
        routine.routine_id for routine in restored.routines
    ] == ["magnetic_scan", "temperature_scan"]
    assert (
        restored.get_routine("temperature_scan").sensor_id
        == "bmm150"
    )


def test_uncaptured_routine_round_trip_preserves_empty_reference_views():
    """A new routine is valid before its first reference capture."""
    routine = InspectionRoutine(
        routine_id="magnetic_scan",
        display_name="Magnetic scan",
        sensor_id="bmm150",
        probe_frame="sensor_tip",
    )

    restored = InspectionRoutine.from_dict(routine.to_dict())

    restored.validate()
    assert restored == routine
    assert restored.to_dict()["reference_views"] == []


def test_probe_points_require_a_captured_reference_view():
    """Image-derived probe points cannot exist before capture."""
    routine = InspectionRoutine(
        routine_id="magnetic_scan",
        display_name="Magnetic scan",
        sensor_id="bmm150",
        probe_frame="sensor_tip",
        probe_points=[make_probe()],
    )

    with pytest.raises(ValueError, match="requires a reference view"):
        routine.validate()


def test_routine_reference_view_requires_persisted_dataset():
    """Authoritative routine views cannot be metadata-only entries."""
    routine = make_routine()
    routine.reference_views[0].reference_dataset_path = None

    with pytest.raises(ValueError, match="requires a dataset path"):
        routine.validate()


def test_duplicate_routine_id_is_rejected():
    """Routine IDs are unique within an object."""
    inspection_object = make_object()
    inspection_object.routines[1].routine_id = "magnetic_scan"

    with pytest.raises(ValueError, match="Duplicate routine ID"):
        inspection_object.validate()


def test_duplicate_probe_point_id_is_rejected():
    """Probe IDs are unique within one routine."""
    routine = make_routine()
    routine.probe_points.append(make_probe())

    with pytest.raises(
        ValueError,
        match="Duplicate probe point ID",
    ):
        routine.validate()


def test_non_normalized_quaternion_is_rejected():
    """Stored transforms require normalized quaternions."""
    routine = make_routine()
    routine.reference_views[0].controlled_frame_pose_object = (
        PoseData.identity()
    )
    pose = routine.reference_views[0].controlled_frame_pose_object
    pose.orientation.w = 2.0

    with pytest.raises(
        ValueError,
        match="Quaternion must be normalized",
    ):
        routine.validate()


def test_old_object_format_is_rejected():
    """Obsolete object files do not receive implicit migration."""
    with pytest.raises(KeyError):
        InspectionObject.from_dict({
            "id": "motor_a",
            "tag_id": 23,
        })


def test_map_validates_internal_and_external_references():
    """Object approaches reference existing map and object IDs."""
    definition = MapDefinition(
        map_id="laboratory",
        display_name="Laboratory",
        waypoints=[
            Waypoint(
                waypoint_id="motor_front",
                display_name="Motor front",
                pose_map=PoseData.identity(),
            )
        ],
        object_approaches=[
            ObjectApproach(
                approach_id="motor_a_front",
                display_name="Motor A front",
                object_id="motor_a",
                waypoint_id="motor_front",
            )
        ],
    )

    definition.validate()
    definition.validate_object_references({"motor_a"})

    with pytest.raises(
        ValueError,
        match="Unknown inspection object",
    ):
        definition.validate_object_references(set())
