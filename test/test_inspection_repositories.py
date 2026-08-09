"""Tests for strict object and map repositories."""

import json

import pytest

from fault_detector_spot.inspection.map_repository import (
    MapRepository,
)
from fault_detector_spot.inspection.models import (
    ImagePoint,
    InspectionObject,
    InspectionRoutine,
    LocalizationLandmark,
    MapDefinition,
    ObjectApproach,
    PoseData,
    ProbePoint,
    ReferenceTag,
    ReferenceView,
    Waypoint,
)
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)


def make_object() -> InspectionObject:
    """Create a valid complete inspection object."""
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(
            tag_id=23,
            tag_family="36h11",
        ),
        routines=[InspectionRoutine(
            routine_id="magnetic_scan",
            display_name="Magnetic scan",
            sensor_id="bmm150",
        )],
    )


def make_map() -> MapDefinition:
    """Create valid map-specific metadata."""
    return MapDefinition(
        map_id="laboratory",
        display_name="Laboratory",
        waypoints=[
            Waypoint(
                waypoint_id="motor_front",
                display_name="Motor front",
                pose_map=PoseData.identity(),
            )
        ],
        localization_landmarks=[
            LocalizationLandmark(
                landmark_id="Tag_23",
                display_name="Tag 23",
                reference_tag=ReferenceTag(
                    tag_id=23,
                    tag_family="36h11",
                ),
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


def make_probe_point(probe_point_id: str) -> ProbePoint:
    """Create one valid image-taught probe point."""
    return ProbePoint(
        probe_point_id=probe_point_id,
        display_name=probe_point_id,
        safe_approach_pose_object=PoseData.identity(),
        probe_pose_object=PoseData.identity(),
        target_surface_distance_m=0.03,
        position_tolerance_m=0.01,
        orientation_tolerance_rad=0.087,
        measurement_duration_sec=1.0,
        aligned_preapproach_distance_m=0.10,
        reference_pixel=ImagePoint(u=20, v=30),
        reference_view_id="slot1_hand",
    )


def test_object_repository_round_trip(tmp_path):
    """One object file contains every inspection routine."""
    repository = ObjectRepository(tmp_path)
    original = make_object()

    path = repository.save(original)
    restored = repository.load("motor_a")

    assert path.is_file()
    assert restored == original
    assert repository.list_object_ids() == ["motor_a"]


def test_object_repository_creates_new_object_without_overwrite(
    tmp_path,
):
    """Explicit creation persists one empty object only once."""
    repository = ObjectRepository(tmp_path)
    definition = InspectionObject(
        object_id="motor_b",
        display_name="Motor B",
        reference_tag=ReferenceTag(
            tag_id=24,
            tag_family="36h11",
        ),
    )

    created = repository.create(definition)

    assert created == definition
    assert repository.load("motor_b") == definition
    with pytest.raises(FileExistsError, match="already exists"):
        repository.create(definition)


def test_object_repository_adds_uncaptured_routine_once(tmp_path):
    """Routine creation updates its existing object aggregate."""
    repository = ObjectRepository(tmp_path)
    definition = InspectionObject(
        object_id="motor_b",
        display_name="Motor B",
        reference_tag=ReferenceTag(
            tag_id=24,
            tag_family="36h11",
        ),
    )
    repository.create(definition)
    routine = InspectionRoutine(
        routine_id="magnetic_scan",
        display_name="Magnetic scan",
        sensor_id="bmm150",
    )

    stored = repository.add_routine("motor_b", routine)

    assert stored.get_routine("magnetic_scan") == routine
    assert repository.load("motor_b") == stored
    assert stored.get_routine("magnetic_scan").reference_view is None
    with pytest.raises(FileExistsError, match="already exists"):
        repository.add_routine("motor_b", routine)


def test_object_repository_requires_object_before_routine(tmp_path):
    """Routine creation never creates a missing parent object."""
    routine = InspectionRoutine(
        routine_id="magnetic_scan",
        display_name="Magnetic scan",
        sensor_id="bmm150",
    )

    with pytest.raises(FileNotFoundError):
        ObjectRepository(tmp_path).add_routine("missing", routine)


def test_object_repository_appends_probe_points_without_overwrite(
    tmp_path,
):
    """Probe insertion preserves order and rejects replacement."""
    repository = ObjectRepository(tmp_path)
    definition = make_object()
    routine = definition.get_routine("magnetic_scan")
    routine.reference_views = [ReferenceView(
        controlled_frame_pose_object=PoseData.identity(),
        controlled_frame="hand_color_image_sensor",
        reference_dataset_path=(
            "reference_datasets/magnetic_scan/set_1/slot1_hand"
        ),
        view_id="slot1_hand",
        camera_id="hand",
        slot_index=0,
    )]
    repository.save(definition)

    repository.add_probe_point(
        "motor_a",
        "magnetic_scan",
        make_probe_point("point_b"),
    )
    stored = repository.add_probe_point(
        "motor_a",
        "magnetic_scan",
        make_probe_point("point_a"),
    )

    assert [
        point.probe_point_id
        for point in stored.get_routine("magnetic_scan").probe_points
    ] == ["point_b", "point_a"]
    assert repository.load("motor_a") == stored

    before_duplicate = repository.get_object_path(
        "motor_a"
    ).read_text(encoding="utf-8")
    with pytest.raises(FileExistsError, match="already exists"):
        repository.add_probe_point(
            "motor_a",
            "magnetic_scan",
            make_probe_point("point_a"),
        )
    assert repository.get_object_path("motor_a").read_text(
        encoding="utf-8"
    ) == before_duplicate


def test_object_repository_deletes_routine_and_owned_datasets(tmp_path):
    """Routine deletion preserves its object and removes owned data."""
    repository = ObjectRepository(tmp_path)
    repository.save(make_object())
    routine_dataset_dir = (
        tmp_path
        / "motor_a"
        / "reference_datasets"
        / "magnetic_scan"
    )
    routine_dataset_dir.mkdir(parents=True)
    (routine_dataset_dir / "marker").write_text("", encoding="utf-8")

    result = repository.delete_routine(
        "motor_a",
        "magnetic_scan",
    )

    assert result.routines == []
    assert repository.load("motor_a").routines == []
    assert not routine_dataset_dir.exists()


def test_object_repository_rejects_missing_routine_deletion(tmp_path):
    """Routine deletion cannot silently accept an unknown target."""
    repository = ObjectRepository(tmp_path)
    repository.save(make_object())

    with pytest.raises(KeyError, match="does not exist"):
        repository.delete_routine("motor_a", "missing")

    assert repository.load("motor_a") == make_object()


def test_object_repository_deletes_complete_object_tree(tmp_path):
    """Object deletion removes its definition and all owned datasets."""
    repository = ObjectRepository(tmp_path)
    repository.save(make_object())
    dataset_path = (
        repository.get_object_dir("motor_a")
        / "reference_datasets"
        / "magnetic_scan"
        / "set"
    )
    dataset_path.mkdir(parents=True)
    object_dir = repository.get_object_dir("motor_a")

    assert repository.delete_object("motor_a") is True
    assert not object_dir.exists()
    assert repository.delete_object("motor_a") is False


def test_object_repository_rejects_old_format(tmp_path):
    """Old object files fail instead of being migrated."""
    path = ObjectRepository(tmp_path).get_object_path("motor_a")
    path.parent.mkdir(parents=True)
    path.write_text(
        "id: motor_a\ntag_id: 23\n",
        encoding="utf-8",
    )

    with pytest.raises(KeyError):
        ObjectRepository(tmp_path).load("motor_a")


def test_object_repository_rejects_path_traversal(tmp_path):
    """Object IDs cannot escape repository storage."""
    repository = ObjectRepository(tmp_path)

    with pytest.raises(ValueError):
        repository.get_object_path("../other")


def test_map_repository_round_trip(tmp_path):
    """Strict map metadata survives saving and loading."""
    repository = MapRepository(tmp_path)
    original = make_map()

    path = repository.save("laboratory", original)
    restored = repository.load("laboratory")

    assert path.is_file()
    assert restored == original
    assert repository.get_waypoint(
        "laboratory",
        "motor_front",
    ) == original.waypoints[0]
    assert repository.get_landmark(
        "laboratory",
        "Tag_23",
    ) == original.localization_landmarks[0]


def test_map_repository_creates_complete_empty_map(tmp_path):
    """New maps immediately use the strict format."""
    repository = MapRepository(tmp_path)

    definition = repository.create_empty("laboratory")
    serialized = json.loads(
        repository.get_map_path("laboratory").read_text(
            encoding="utf-8"
        )
    )

    assert definition.map_id == "laboratory"
    assert serialized == {
        "map_id": "laboratory",
        "display_name": "laboratory",
        "waypoints": [],
        "localization_landmarks": [],
        "object_approaches": [],
    }


def test_map_repository_rejects_old_format(tmp_path):
    """Legacy map JSON is intentionally invalid."""
    path = MapRepository(tmp_path).get_map_path("legacy")
    path.write_text(
        json.dumps({
            "waypoints": [],
            "landmarks": [],
        }),
        encoding="utf-8",
    )

    with pytest.raises(KeyError):
        MapRepository(tmp_path).load("legacy")


def test_map_repository_rejects_id_mismatch(tmp_path):
    """The map ID must match its storage filename."""
    repository = MapRepository(tmp_path)

    with pytest.raises(ValueError, match="does not match"):
        repository.save("other", make_map())
