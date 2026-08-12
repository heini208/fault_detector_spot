"""Lock mapping model and runtime ownership boundaries."""

import inspect
from pathlib import Path

from fault_detector_spot.application.behaviour_tree.behaviours.helper_initializer import (
    HelperInitializer,
)
from fault_detector_spot.inspection.model import models as inspection_models
from fault_detector_spot.mapping.model.models import (
    LocalizationLandmark,
    MapDefinition,
    ObjectApproach,
    Waypoint,
)
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper
from fault_detector_spot.navigation.behaviours.set_waypoint_as_goal import (
    SetWaypointAsGoal,
)
from fault_detector_spot.navigation.runtime.nav2_helper import Nav2Helper


def test_map_models_are_not_owned_by_inspection_module():
    for name in (
        "Waypoint",
        "LocalizationLandmark",
        "ObjectApproach",
        "MapDefinition",
    ):
        assert not hasattr(inspection_models, name)


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


def test_waypoint_goal_uses_configured_persistent_map_root():
    source = inspect.getsource(SetWaypointAsGoal.setup)
    assert '"navigation.map_root"' in source
    assert "default_map_root" in source
    assert "get_package_share_directory" not in source


def test_current_runtime_defaults_are_lidar_launches():
    rtab_signature = inspect.signature(RTABHelper.__init__)
    nav2_signature = inspect.signature(Nav2Helper.__init__)
    assert rtab_signature.parameters["launch_file"].default == (
        "lidar_rtab_mapping_launch.py"
    )
    assert rtab_signature.parameters["nav2_launch_file"].default == (
        "nav2_lidar_launch.py"
    )
    assert rtab_signature.parameters["nav2_params_file"].default == (
        "nav2_lidar_params.yaml"
    )
    assert nav2_signature.parameters["launch_file"].default == (
        "nav2_lidar_launch.py"
    )


def test_helper_initializer_has_no_unused_simulation_switch():
    assert "use_simulation" not in inspect.signature(
        HelperInitializer.__init__
    ).parameters
    assert "use_simulation" not in inspect.getsource(HelperInitializer)


def test_old_rgbd_launch_is_removed_from_repository():
    path = Path(__file__).parents[1] / "launch" / "rtab_mapping_launch.py"
    assert not path.exists()
