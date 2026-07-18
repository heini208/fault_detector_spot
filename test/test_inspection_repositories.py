"""Tests for map and inspection repositories."""

import json

import pytest
import yaml

from fault_detector_spot.inspection.inspection_repository import (
    InspectionRepository,
)
from fault_detector_spot.inspection.map_repository import (
    MapRepository,
)
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)
from fault_detector_spot.inspection.models import (
    InspectionDefinition,
    InspectionObject,
    LandmarkDefinition,
    MapDefinition,
    ObjectDefinition,
    PoseData,
    QuaternionData,
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
        object_id="motor_a",
        object_calibration_revision=1,
        map_name="laboratory",
        display_name="Motor A standard",
        preferred_execution_frame="odom",
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


def create_valid_object() -> ObjectDefinition:
    """Create a portable object definition."""
    return ObjectDefinition(
        object_id="motor_a",
        display_name="Motor A",
        tag_id=23,
        tag_family="36h11",
        calibration_revision=1,
        marker_to_object=PoseData(
            orientation=QuaternionData(w=1.0),
        ),
    )


def test_object_repository_round_trip(tmp_path):
    """Portable object YAML survives saving and loading."""
    repository = ObjectRepository(tmp_path)
    original = create_valid_object()

    saved_path = repository.save(original)
    restored = repository.load("motor_a")

    assert saved_path.is_file()
    assert restored == original
    assert restored.calibration_revision == 1
    assert repository.list_object_ids() == ["motor_a"]


def test_object_repository_rejects_path_traversal(tmp_path):
    """Object IDs cannot escape the repository."""
    repository = ObjectRepository(tmp_path)

    with pytest.raises(ValueError):
        repository.get_object_path("../other")


def test_object_repository_rejects_zero_quaternion(tmp_path):
    """Object transforms require a valid quaternion."""
    repository = ObjectRepository(tmp_path)
    definition = create_valid_object()
    definition.marker_to_object.orientation.w = 0.0

    with pytest.raises(ValueError):
        repository.save(definition)


def test_object_repository_defaults_missing_transform_to_identity(
    tmp_path,
):
    """A legacy object without a transform uses tag-frame identity."""
    path = ObjectRepository(tmp_path).get_object_path("motor_a")
    path.parent.mkdir(parents=True)
    path.write_text(
        yaml.safe_dump({
            "id": "motor_a",
            "display_name": "Motor A",
            "tag_id": 23,
        }),
        encoding="utf-8",
    )

    restored = ObjectRepository(tmp_path).load("motor_a")

    assert restored.marker_to_object == PoseData()
    assert restored.calibration_revision == 1

def test_inspection_repository_round_trip(tmp_path):
    """Inspection YAML uses object-scoped storage."""
    repository = InspectionRepository(tmp_path)
    original = create_valid_inspection()

    saved_path = repository.save(original)
    restored = repository.load(
        "motor_a",
        "motor_a_standard",
    )

    assert saved_path.is_file()
    assert saved_path == (
        tmp_path
        / "motor_a"
        / "motor_a_standard"
        / "inspection.yaml"
    )
    assert restored == original
    assert restored.object_calibration_revision == 1


def test_probe_point_schema_round_trip(tmp_path):
    """Probe points persist their independent schema version."""
    repository = InspectionRepository(tmp_path)
    original = create_valid_inspection()

    repository.save(original)
    restored = repository.load(
        "motor_a",
        "motor_a_standard",
    )

def test_inspection_repository_lists_inspections(tmp_path):
    """Only canonical object inspections are listed."""
    repository = InspectionRepository(tmp_path)
    repository.save(create_valid_inspection())

    invalid_directory = (
        tmp_path
        / "motor_a"
        / "not_an_inspection"
    )
    invalid_directory.mkdir(parents=True)

    assert repository.list_inspection_ids(
        "motor_a"
    ) == ["motor_a_standard"]


def test_inspection_repository_loads_legacy_without_moving(
    tmp_path,
):
    """Legacy map data loads only through explicit fallback."""
    repository = InspectionRepository(tmp_path)
    legacy_path = repository.get_legacy_inspection_path(
        "laboratory",
        "motor_a_standard",
    )
    legacy_path.parent.mkdir(parents=True)
    legacy_data = create_valid_inspection().to_dict()
    legacy_data.pop("preferred_execution_frame")
    legacy_path.write_text(
        yaml.safe_dump(legacy_data, sort_keys=False),
        encoding="utf-8",
    )

    restored = repository.load(
        "motor_a",
        "motor_a_standard",
        legacy_map_name="laboratory",
    )

    assert restored.preferred_execution_frame == "odom"
    assert legacy_path.is_file()
    assert not repository.get_inspection_path(
        "motor_a",
        "motor_a_standard",
    ).exists()


def test_saving_loaded_legacy_creates_v3_without_deleting_old(
    tmp_path,
):
    """Explicit save migrates a copy and preserves legacy data."""
    repository = InspectionRepository(tmp_path)
    legacy_path = repository.get_legacy_inspection_path(
        "laboratory",
        "motor_a_standard",
    )
    legacy_path.parent.mkdir(parents=True)
    legacy_data = create_valid_inspection().to_dict()
    legacy_path.write_text(
        yaml.safe_dump(legacy_data, sort_keys=False),
        encoding="utf-8",
    )

    restored = repository.load(
        "motor_a",
        "motor_a_standard",
        legacy_map_name="laboratory",
    )
    current_path = repository.save(restored)
    current = repository.load(
        "motor_a",
        "motor_a_standard",
    )

    assert current_path.is_file()
    assert legacy_path.is_file()


def test_inspection_repository_rejects_path_traversal(
    tmp_path,
):
    """Storage identifiers cannot escape the repository."""
    repository = InspectionRepository(tmp_path)

    with pytest.raises(ValueError):
        repository.get_inspection_path(
            "../other_object",
            "inspection",
        )