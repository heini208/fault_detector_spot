"""Saved observation coordinates reach geometry and retain the original target."""

from copy import deepcopy

import numpy as np
import pytest

from fault_detector_spot.manipulation.measure_sensor_tip_distance_execution import (
    MeasureSensorTipDistanceConfig, MeasureSensorTipDistanceOutcome,
)
from fault_detector_spot.manipulation.move_close_to_surface_execution import MoveCloseToSurfaceConfig
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.persistence import basic_settings
from test_arm_motion_parameters import FakeNode
from test_measure_sensor_tip_distance_execution import harness, finish
import test_sensor_tip_observation as observation_fixtures
from test_sensor_tip_observation_hint import set_image
from test_sensor_tip_automatic_discovery import wall_image


source = observation_fixtures.source
CONFIGS = (MeasureSensorTipDistanceConfig, MoveCloseToSurfaceConfig)


@pytest.mark.parametrize("config_type", CONFIGS)
def test_saved_reference_reaches_each_owner_and_changes_only_after_restart(config_type):
    repo = basic_settings.BasicSettingsRepository()
    repo.save({
        "close_surface.observation_custom_region_enabled": True,
        "close_surface.observation_reference_u": 1.0,
        "close_surface.observation_reference_v": 0.0,
    })
    config = config_type.from_node(FakeNode())
    assert config.observation_reference_position == (1.0, 0.0)
    assert config.observation_custom_region_enabled is True

    repo.save({"close_surface.observation_custom_region_enabled": False})
    assert config_type.from_node(FakeNode()).observation_reference_position == (1.0, 0.0)
    basic_settings._startup_settings.cache_clear()
    assert config_type.from_node(FakeNode()).observation_reference_position is None


@pytest.mark.parametrize("config_type", CONFIGS)
@pytest.mark.parametrize("name,value", [
    ("observation_custom_region_enabled", 1),
    ("observation_custom_region_enabled", "false"),
    ("observation_reference_u", True),
    ("observation_reference_u", -0.01),
    ("observation_reference_v", 1.01),
    ("observation_reference_v", float("nan")),
])
def test_invalid_ros_reference_settings_fail_before_movement(config_type, name, value):
    with pytest.raises(ValueError, match=name):
        config_type.from_node(FakeNode({f"close_surface.{name}": value}))


@pytest.mark.parametrize("custom", [False, True])
def test_execution_passes_frozen_reference_through_observation_and_return(custom):
    case = harness(
        visible=False, observation_custom_region_enabled=custom,
        observation_reference_u=1.0, observation_reference_v=1.0,
    )
    original = deepcopy(case.executor.probe_motion_planner.hand)
    assert finish(case) is MeasureSensorTipDistanceOutcome.SUCCESS
    expected = (1.0, 1.0) if custom else None
    assert case.source.view_reference_positions == [expected, expected]
    assert len(case.executor.calls) == 2
    assert case.executor.probe_motion_planner.hand == original


def test_bottom_right_reference_guides_empty_view_then_measures_original_axis(source):
    camera = observation_fixtures.camera_pose(0.08)
    reference = PoseData.identity()
    image, info = wall_image(camera, 0.218)
    image.data = np.zeros((480, 640), dtype="<f4").tobytes()
    set_image(source, image, info, camera)
    view = source.source.observation_view(reference, reference_position=(1.0, 1.0))
    assert not view.visible
    assert view.target_plane_probe is None
    assert source.fits == 0
    delta = view.hand_translation_odom
    assert delta.x == pytest.approx(0.0, abs=1e-12)
    assert delta.y > 0.0  # Camera moves left to bring the saved ray image-right.
    assert -0.08 < delta.z < 0.0

    camera.position = Vector3Data(*(
        getattr(camera.position, axis) + getattr(delta, axis) for axis in ("x", "y", "z")
    ))
    image, info = wall_image(camera, 0.218)
    set_image(source, image, info, camera)
    measured = source.source.observation_view(
        reference, allow_translation=False, reference_position=(1.0, 1.0),
    )
    assert measured.visible
    assert measured.hand_translation_odom == Vector3Data.zero()
    assert measured.target_plane_probe.signed_distance(Vector3Data.zero()) == pytest.approx(0.218, abs=1e-6)
