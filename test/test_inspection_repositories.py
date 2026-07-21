"""Tests for strict object and map repositories."""

import json
from array import array
from dataclasses import replace

import pytest
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection import object_repository as object_module
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
    Vector3Data,
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


def make_image(encoding, step, data, nanosec) -> Image:
    """Create one captured image."""
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = nanosec
    image.height = 2
    image.width = 2
    image.encoding = encoding
    image.step = step
    image.data = array("B", data)
    return image


def make_reference_inputs():
    """Create RGB, registered depth, and CameraInfo."""
    rgb = make_image("rgb8", 6, range(12), 200_000_000)
    depth = make_image(
        "16UC1",
        4,
        [232, 3, 208, 7, 184, 11, 160, 15],
        210_000_000,
    )
    info = CameraInfo()
    info.header.frame_id = "hand_color_image_sensor"
    info.header.stamp.sec = 5
    info.height = 2
    info.width = 2
    info.k = [100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0]
    return rgb, depth, info


def serialized(message) -> bytes:
    """Return exact CDR bytes for comparison."""
    return bytes(serialize_message(message))


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
        probe_frame="sensor_tip",
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
        probe_frame="sensor_tip",
    )

    with pytest.raises(FileNotFoundError):
        ObjectRepository(tmp_path).add_routine("missing", routine)


def test_object_repository_deletes_routine_and_owned_datasets(tmp_path):
    """Routine deletion preserves its object and removes owned data."""
    repository = ObjectRepository(tmp_path)
    original = make_object()
    repository.save(original)
    stored = repository.save_reference_dataset(
        "motor_a",
        "magnetic_scan",
        original.routines[0].reference_view,
        *make_reference_inputs(),
    )
    dataset_path = (
        tmp_path
        / "motor_a"
        / stored.routines[0].reference_view.reference_dataset_path
    )

    result = repository.delete_routine(
        "motor_a",
        "magnetic_scan",
    )

    assert result.routines == []
    assert repository.load("motor_a").routines == []
    assert not dataset_path.exists()
    assert not dataset_path.parent.exists()


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
    original = make_object()
    repository.save(original)
    repository.save_reference_dataset(
        "motor_a",
        "magnetic_scan",
        original.routines[0].reference_view,
        *make_reference_inputs(),
    )
    object_dir = repository.get_object_dir("motor_a")

    assert repository.delete_object("motor_a") is True
    assert not object_dir.exists()
    assert repository.delete_object("motor_a") is False


def test_reference_dataset_load_requires_captured_view(tmp_path):
    """An uncaptured routine has no loadable camera dataset."""
    repository = ObjectRepository(tmp_path)
    definition = InspectionObject(
        object_id="motor_b",
        display_name="Motor B",
        reference_tag=ReferenceTag(
            tag_id=24,
            tag_family="36h11",
        ),
        routines=[
            InspectionRoutine(
                routine_id="magnetic_scan",
                display_name="Magnetic scan",
                sensor_id="bmm150",
                probe_frame="sensor_tip",
            )
        ],
    )
    repository.create(definition)

    with pytest.raises(ValueError, match="captured reference view"):
        repository.load_reference_dataset(
            "motor_b",
            "magnetic_scan",
        )


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


def test_object_repository_saves_reference_dataset_with_object(
    tmp_path,
):
    """Dataset persistence updates the owning object definition."""
    repository = ObjectRepository(tmp_path)
    original = make_object()
    repository.save(original)
    inputs = make_reference_inputs()
    resolved_view = replace(
        original.routines[0].reference_view,
        controlled_frame_pose_object=replace(
            PoseData.identity(),
            position=Vector3Data(x=1.0, y=2.0, z=3.0),
        ),
    )

    stored = repository.save_reference_dataset(
        "motor_a",
        "magnetic_scan",
        resolved_view,
        *inputs,
    )
    restored_object = repository.load("motor_a")
    restored_inputs = repository.load_reference_dataset(
        "motor_a",
        "magnetic_scan",
    )

    assert (
        original.routines[0].reference_view.reference_dataset_path
        is None
    )
    assert stored == restored_object
    assert stored.routines[0].reference_view.reference_dataset_path == (
        "reference_datasets/magnetic_scan/10_200000000"
    )
    assert (
        stored.routines[0].reference_view.controlled_frame_pose_object
        == resolved_view.controlled_frame_pose_object
    )
    assert [serialized(value) for value in restored_inputs] == [
        serialized(value) for value in inputs
    ]


def test_reference_dataset_write_failure_preserves_object(
    tmp_path,
    monkeypatch,
):
    """A failed dataset write cannot change the object aggregate."""
    repository = ObjectRepository(tmp_path)
    original = make_object()
    repository.save(original)
    calls = 0

    def fail_second_write(path, content):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError("write failed")
        path.write_bytes(content)

    monkeypatch.setattr(
        object_module,
        "_write_bytes",
        fail_second_write,
    )

    with pytest.raises(OSError, match="write failed"):
        repository.save_reference_dataset(
            "motor_a",
            "magnetic_scan",
            original.routines[0].reference_view,
            *make_reference_inputs(),
        )

    assert repository.load("motor_a") == original
    routine_dir = (
        tmp_path
        / "motor_a"
        / "reference_datasets"
        / "magnetic_scan"
    )
    assert list(routine_dir.iterdir()) == []


def test_object_save_failure_rolls_back_published_dataset(
    tmp_path,
    monkeypatch,
):
    """A failed object update removes its unpublished dataset."""
    repository = ObjectRepository(tmp_path)
    original = make_object()
    repository.save(original)

    def fail_save(definition, validate=True):
        raise OSError("object write failed")

    monkeypatch.setattr(repository, "save", fail_save)

    with pytest.raises(OSError, match="object write failed"):
        repository.save_reference_dataset(
            "motor_a",
            "magnetic_scan",
            original.routines[0].reference_view,
            *make_reference_inputs(),
        )

    restored = ObjectRepository(tmp_path).load("motor_a")
    assert restored == original
    routine_dir = (
        tmp_path
        / "motor_a"
        / "reference_datasets"
        / "magnetic_scan"
    )
    assert list(routine_dir.iterdir()) == []


def test_new_reference_dataset_replaces_owned_previous_dataset(
    tmp_path,
):
    """Recapture updates the view and removes the prior dataset."""
    repository = ObjectRepository(tmp_path)
    original = make_object()
    repository.save(original)
    first = repository.save_reference_dataset(
        "motor_a",
        "magnetic_scan",
        original.routines[0].reference_view,
        *make_reference_inputs(),
    )
    old_relative_path = (
        first.routines[0].reference_view.reference_dataset_path
    )
    old_dataset_path = tmp_path / "motor_a" / old_relative_path
    rgb, depth, info = make_reference_inputs()
    rgb.header.stamp.nanosec = 300_000_000
    new_view = replace(
        first.routines[0].reference_view,
        reference_dataset_path=None,
    )

    second = repository.save_reference_dataset(
        "motor_a",
        "magnetic_scan",
        new_view,
        rgb,
        depth,
        info,
    )

    assert second.routines[0].reference_view.reference_dataset_path == (
        "reference_datasets/magnetic_scan/10_300000000"
    )
    assert not old_dataset_path.exists()
    assert repository.load("motor_a") == second


def test_reference_dataset_rejects_unowned_path(tmp_path):
    """Dataset paths cannot escape the selected routine."""
    repository = ObjectRepository(tmp_path)
    original = make_object()
    bad_view = replace(
        original.routines[0].reference_view,
        reference_dataset_path="../outside",
    )
    bad_routine = replace(
        original.routines[0],
        reference_view=bad_view,
    )
    invalid = replace(original, routines=[bad_routine])
    repository.save(invalid)

    with pytest.raises(ValueError, match="outside"):
        repository.load_reference_dataset(
            "motor_a",
            "magnetic_scan",
        )


def test_reference_dataset_rejects_missing_routine(tmp_path):
    """Dataset operations require a routine owned by the object."""
    repository = ObjectRepository(tmp_path)
    original = make_object()
    repository.save(original)

    with pytest.raises(KeyError, match="does not exist"):
        repository.save_reference_dataset(
            "motor_a",
            "missing",
            original.routines[0].reference_view,
            *make_reference_inputs(),
        )

    with pytest.raises(KeyError, match="does not exist"):
        repository.load_reference_dataset("motor_a", "missing")


def test_reference_dataset_rejects_zero_rgb_timestamp(tmp_path):
    """A dataset cannot use latest-time fallback semantics."""
    rgb, depth, info = make_reference_inputs()
    rgb.header.stamp.sec = 0
    rgb.header.stamp.nanosec = 0
    repository = ObjectRepository(tmp_path)
    original = make_object()
    repository.save(original)

    with pytest.raises(ValueError, match="must not be zero"):
        repository.save_reference_dataset(
            "motor_a",
            "magnetic_scan",
            original.routines[0].reference_view,
            rgb,
            depth,
            info,
        )


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
