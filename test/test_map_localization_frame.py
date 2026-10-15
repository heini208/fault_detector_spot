"""Reference-frame compatibility checks without ROS nodes or child processes."""

import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_spot.mapping.model.models import MapDefinition
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.mapping.runtime.rtabmap_runtime_manager import RtabmapRuntimeManager


def legacy_metadata():
    return {
        "map_id": "plant",
        "display_name": "Plant",
        "waypoints": [],
        "localization_landmarks": [],
        "object_approaches": [],
    }


def test_new_maps_record_floor_reference_and_round_trip(tmp_path):
    repository = MapRepository(tmp_path)
    definition = repository.create_empty("plant")
    assert definition.localization_frame == "feet_center"
    assert repository.load("plant").localization_frame == "feet_center"
    assert json.loads(repository.get_map_path("plant").read_text())["localization_frame"] == "feet_center"


def test_legacy_metadata_remains_body_referenced_when_saved(tmp_path):
    repository = MapRepository(tmp_path)
    path = repository.get_map_path("plant")
    original = json.dumps(legacy_metadata())
    path.write_text(original)
    definition = repository.load("plant")
    assert definition.localization_frame == "base_link"
    assert path.read_text() == original  # Reading must not migrate the file.
    definition.display_name = "Renamed plant"
    repository.save("plant", definition)
    assert repository.load("plant").localization_frame == "base_link"


def test_creating_metadata_for_existing_database_preserves_body_reference(tmp_path):
    (tmp_path / "plant.db").write_bytes(b"legacy database")
    definition = MapRepository(tmp_path).create_empty("plant")
    assert definition.localization_frame == "base_link"
    assert (tmp_path / "plant.db").read_bytes() == b"legacy database"


@pytest.mark.parametrize("frame", ["base_link", "feet_center"])
def test_empty_metadata_overwrite_preserves_recorded_frame(tmp_path, frame):
    repository = MapRepository(tmp_path)
    repository.save("plant", MapDefinition("plant", "Plant", localization_frame=frame))
    assert repository.create_empty("plant", overwrite=True).localization_frame == frame


def test_repository_rejects_changing_reference_frame_in_place(tmp_path):
    repository = MapRepository(tmp_path)
    definition = repository.create_empty("plant")
    original = repository.get_map_path("plant").read_bytes()
    definition.localization_frame = "base_link"
    with pytest.raises(ValueError, match="Cannot change"):
        repository.save("plant", definition)
    assert repository.get_map_path("plant").read_bytes() == original


@pytest.mark.parametrize("frame", [None, "", "odom", "body", "feet_center ", [], 4])
def test_unknown_or_malformed_frame_is_rejected(frame):
    data = legacy_metadata()
    data["localization_frame"] = frame
    with pytest.raises(ValueError, match="localization frame"):
        MapDefinition.from_dict(data).validate()


def runtime(tmp_path):
    manager = object.__new__(RtabmapRuntimeManager)
    manager.maps_dir = str(tmp_path)
    manager.bb = SimpleNamespace(active_map_name=None)
    manager.launch_file = "lidar_rtab_mapping_launch.py"
    manager.raw_lidar_topic = None
    manager.node = Mock()
    manager.stop = Mock(return_value=True)
    manager._launch = Mock(return_value=object())
    manager._publish_active_map = Mock()
    return manager


@pytest.mark.parametrize("frame,ground_height", [("feet_center", "0.05"), ("base_link", "0.12")])
@pytest.mark.parametrize("mapping", [False, True])
def test_runtime_passes_recorded_frame_without_rewriting_metadata(tmp_path, frame, ground_height, mapping):
    repository = MapRepository(tmp_path)
    repository.save("plant", MapDefinition("plant", "Plant", localization_frame=frame))
    path = repository.get_map_path("plant")
    original = path.read_bytes()
    (tmp_path / "plant.db").touch()
    manager = runtime(tmp_path)
    manager._start_rtabmap("plant", extend_map=mapping)
    _, arguments = manager._launch.call_args.args
    assert f"frame_id:={frame}" in arguments
    assert f"ground_max_height:={ground_height}" in arguments
    assert manager.localization_frame == frame
    assert path.read_bytes() == original


@pytest.mark.parametrize("database_exists", [False, True])
def test_metadata_absent_runtime_calls_stay_legacy_without_creating_metadata(tmp_path, database_exists):
    if database_exists:
        (tmp_path / "plant.db").touch()
    manager = runtime(tmp_path)
    manager._start_rtabmap("plant", extend_map=True)
    _, arguments = manager._launch.call_args.args
    assert "frame_id:=base_link" in arguments
    assert "ground_max_height:=0.12" in arguments
    assert not (tmp_path / "plant.json").exists()


def test_invalid_metadata_fails_before_stopping_current_runtime(tmp_path):
    data = legacy_metadata()
    data["localization_frame"] = "odom"
    (tmp_path / "plant.json").write_text(json.dumps(data))
    manager = runtime(tmp_path)
    with pytest.raises(ValueError, match="localization frame"):
        manager._start_rtabmap("plant")
    manager.stop.assert_not_called()
    manager._launch.assert_not_called()
