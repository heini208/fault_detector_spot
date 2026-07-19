"""Tests for strict object and map repositories."""

import json

import pytest

from fault_detector_spot.inspection.map_repository import (
    MapRepository,
)
from fault_detector_spot.inspection.models import (
    InspectionObject,
    InspectionRoutine,
    LocalizationLandmark,
    MapDefinition,
    ObjectApproach,
    PoseData,
    ReferenceTag,
    ReferenceView,
    Waypoint,
)
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)


def make_object() -> InspectionObject:
    """Create a valid complete inspection object."""
    routine = InspectionRoutine(
        routine_id="magnetic_scan",
        display_name="Magnetic scan",
        sensor_id="bmm150",
        probe_frame="sensor_tip",
        reference_view=ReferenceView(
            controlled_frame_pose_object=PoseData.identity(),
            controlled_frame="hand_color_image_sensor",
        ),
    )
    return InspectionObject(
        object_id="motor_a",
        display_name="Motor A",
        reference_tag=ReferenceTag(
            tag_id=23,
            tag_family="36h11",
        ),
        routines=[routine],
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


def test_object_repository_round_trip(tmp_path):
    """One object file contains every inspection routine."""
    repository = ObjectRepository(tmp_path)
    original = make_object()

    path = repository.save(original)
    restored = repository.load("motor_a")

    assert path.is_file()
    assert restored == original
    assert repository.list_object_ids() == ["motor_a"]


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
