"""Tests for map and inspection repositories."""

import json

import pytest

from fault_detector_spot.inspection.inspection_repository import (
    InspectionRepository,
)
from fault_detector_spot.inspection.map_repository import (
    MapRepository,
)
from fault_detector_spot.inspection.models import (
    InspectionDefinition,
    InspectionObject,
    LandmarkDefinition,
    MapDefinition,
    PoseData,
    ProbePoint,
    Vector3Data,
    WaypointDefinition,
    WaypointReference,
)


def create_valid_map() -> MapDefinition:
    """Create valid map metadata for repository tests."""
    landmark = LandmarkDefinition(
        name="Tag_23",
        tag_id=23,
        pose=PoseData(),
    )

    waypoint = WaypointDefinition(
        name="motor_front",
        pose=PoseData(),
        reference_type=WaypointReference.OBJECT,
        object_id="motor_a",
    )

    inspection_object = InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        tag_id=23,
        landmark_name="Tag_23",
        approach_waypoints=["motor_front"],
        inspections=["motor_a_standard"],
    )

    return MapDefinition(
        landmarks=[landmark],
        waypoints=[waypoint],
        objects=[inspection_object],
    )


def create_valid_inspection() -> InspectionDefinition:
    """Create a valid inspection definition."""
    probe_point = ProbePoint(
        probe_point_id="bearing_front",
        surface_point_object=Vector3Data(
            x=0.2,
            y=0.0,
            z=0.3,
        ),
        surface_normal_object=Vector3Data(
            x=-1.0,
            y=0.0,
            z=0.0,
        ),
        standoff_m=0.01,
        approach_distance_m=0.10,
        position_tolerance_m=0.01,
        orientation_tolerance_rad=0.087,
        measurement_duration_sec=2.0,
        approach_waypoint="motor_front",
    )

    return InspectionDefinition(
        inspection_id="motor_a_standard",
        map_name="laboratory",
        object_id="motor_a",
        probe_points=[probe_point],
        reference_image="reference.png",
        default_approach_waypoint="motor_front",
    )


def test_map_repository_loads_legacy_map(tmp_path):
    """Legacy map metadata loads without migration."""
    repository = MapRepository(tmp_path)
    path = repository.get_map_path("legacy")

    path.write_text(
        json.dumps({
            "waypoints": [
                {
                    "name": "start",
                    "pose": {},
                }
            ],
            "landmarks": [],
        }),
        encoding="utf-8",
    )

    map_definition = repository.load("legacy")

    assert map_definition.objects == []
    assert (
        map_definition.waypoints[0].reference_type
        == WaypointReference.MAP
    )


def test_map_repository_round_trip(tmp_path):
    """Extended map metadata survives saving and loading."""
    repository = MapRepository(tmp_path)
    original = create_valid_map()

    repository.save("laboratory", original)
    restored = repository.load("laboratory")

    assert restored == original


def test_map_repository_preserves_unknown_fields(tmp_path):
    """Unknown legacy fields survive a round trip."""
    repository = MapRepository(tmp_path)
    path = repository.get_map_path("laboratory")

    path.write_text(
        json.dumps({
            "waypoints": [],
            "landmarks": [],
            "objects": [],
            "custom_metadata": {
                "value": 42,
            },
        }),
        encoding="utf-8",
    )

    map_definition = repository.load("laboratory")
    repository.save("laboratory", map_definition)

    restored_data = json.loads(
        path.read_text(encoding="utf-8")
    )

    assert restored_data["custom_metadata"]["value"] == 42


def test_invalid_map_is_not_written(tmp_path):
    """Invalid map references prevent writing."""
    repository = MapRepository(tmp_path)

    invalid_map = MapDefinition(
        waypoints=[
            WaypointDefinition(
                name="invalid",
                pose=PoseData(),
                reference_type=WaypointReference.OBJECT,
                object_id="missing_object",
            )
        ]
    )

    with pytest.raises(ValueError):
        repository.save("invalid", invalid_map)

    assert not repository.get_map_path("invalid").exists()


def test_inspection_repository_round_trip(tmp_path):
    """Inspection YAML survives saving and loading."""
    repository = InspectionRepository(tmp_path)
    original = create_valid_inspection()

    saved_path = repository.save(original)
    restored = repository.load(
        "laboratory",
        "motor_a_standard",
    )

    assert saved_path.is_file()
    assert restored == original


def test_inspection_repository_lists_inspections(tmp_path):
    """Only valid inspection directories are listed."""
    repository = InspectionRepository(tmp_path)

    repository.save(create_valid_inspection())

    invalid_directory = (
        tmp_path
        / "laboratory"
        / "not_an_inspection"
    )
    invalid_directory.mkdir(parents=True)

    inspection_ids = repository.list_inspection_ids(
        "laboratory"
    )

    assert inspection_ids == ["motor_a_standard"]


def test_inspection_repository_rejects_path_traversal(
    tmp_path,
):
    """Storage identifiers cannot escape the repository."""
    repository = InspectionRepository(tmp_path)

    with pytest.raises(ValueError):
        repository.get_inspection_path(
            "../other_map",
            "inspection",
        )