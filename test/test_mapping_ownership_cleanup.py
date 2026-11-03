"""Lock mapping model and runtime ownership boundaries."""

import inspect
from types import SimpleNamespace

import pytest
from fault_detector_spot.mapping.model.models import (
    LocalizationLandmark,
    MapDefinition,
    ObjectApproach,
    Waypoint,
)
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.navigation.behaviours.set_waypoint_as_goal import (
    SetWaypointAsGoal,
)
from fault_detector_spot.shared.persistence.runtime_paths import default_map_root


def test_map_repository_uses_mapping_models():
    source = inspect.getsource(MapRepository)
    assert "inspection.model.models" not in source
    assert "mapping.model.models" in inspect.getsource(
        __import__(
            "fault_detector_spot.mapping.repository.map_repository",
            fromlist=["MapRepository"],
        )
    )
    assert all(
        value.__module__ == "fault_detector_spot.mapping.model.models"
        for value in (Waypoint, LocalizationLandmark, ObjectApproach, MapDefinition)
    )


@pytest.mark.parametrize("configuration", ["explicit", "blank", "absent"])
def test_waypoint_goal_uses_configured_persistent_map_root(tmp_path, configuration):
    configured = tmp_path / "custom_maps"
    parameters = {}
    if configuration != "absent":
        parameters["navigation.map_root"] = (
            f" {configured} " if configuration == "explicit" else " "
        )
    node = SimpleNamespace(
        has_parameter=lambda name: name in parameters,
        declare_parameter=lambda name, value: parameters.setdefault(name, value),
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
    )
    behavior = SetWaypointAsGoal()
    try:
        behavior.setup(node=node)
        expected = configured if configuration == "explicit" else default_map_root()
        assert behavior.repository.maps_dir == expected
    finally:
        behavior.blackboard.unregister_all_keys()
