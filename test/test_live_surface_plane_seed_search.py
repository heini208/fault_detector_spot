"""A missing seed window may use measured depth within the existing fit radius."""

from copy import deepcopy

import numpy as np
import pytest

from fault_detector_spot.inspection.sensing.live_surface_plane import (
    SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX,
    estimate_live_surface_normal,
)
from fault_detector_spot.inspection.sensing.sensor_tip_distance import (
    SensorTipTargetUnavailable,
    measure_sensor_tip_distance,
)
from fault_detector_spot.shared.geometry.models import PoseData
from test_sensor_tip_automatic_discovery import wall_image
import test_sensor_tip_observation as observation_fixtures
from test_surface_normal import make_32fc1, make_camera_info, plane_depth_values


source = observation_fixtures.source


def seed_scene(normal=(0.0, 0.0, -1.0)):
    width = height = 161
    info = make_camera_info(width, height, 300.0)
    values = np.asarray(
        plane_depth_values(width, height, info, normal, 0.4), dtype="<f4",
    ).reshape(height, width)
    rows, columns = np.indices(values.shape)
    radius_squared = (columns-80)**2 + (rows-120)**2
    values[radius_squared < 18**2] = 0.0
    return values, info, radius_squared


@pytest.mark.parametrize("normal", [(0.0, 0.0, -1.0), (0.18, -0.12, -1.0)])
def test_empty_seed_window_finds_wall_without_expanding_the_plane_fit(normal):
    values, info, _ = seed_scene(normal)
    image = make_32fc1(values.ravel(), 161, 161)

    estimate = estimate_live_surface_normal(image, info)

    sampled = estimate.projected_point.sampled_pixel
    assert 18.0 <= np.hypot(sampled.u-80, sampled.v-120) <= 19.0
    assert estimate.neighborhood_radius_px == SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX
    assert estimate.sample_count >= 30
    assert estimate.surface_plane_camera.normal_array() == pytest.approx(
        np.asarray(normal)/np.linalg.norm(normal), abs=1e-6,
    )


def test_missing_seed_does_not_affect_axis_observation_or_replace_target_depth(source):
    camera = observation_fixtures.camera_pose(0.08)
    image, info = wall_image(camera, 0.218)
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    rows, columns = np.indices(values.shape)
    values[(columns-320)**2 + (rows-360)**2 < 18**2] = 0.0
    values[376:383, 317:324] = 0.0
    image.data = values.tobytes()
    source.source._receive_hand_depth_camera_info(info)
    source.source._receive_hand_depth(image)
    source.source._lookup_pose = lambda *args, **kwargs: deepcopy(camera)
    reference = PoseData.identity()

    view = source.source.observation_view(reference)

    assert not view.visible
    assert "calibrated TF" in view.detail
    assert view.target_plane_probe is None
    delta = view.hand_translation_odom
    assert delta.x == pytest.approx(0.0, abs=1e-9)
    assert 0.001 < np.linalg.norm([delta.x, delta.y, delta.z]) < 0.20
    with pytest.raises(SensorTipTargetUnavailable, match="insufficient unoccluded"):
        measure_sensor_tip_distance(image, info, reference, camera)
    with pytest.raises(ValueError, match="insufficient unoccluded"):
        source.source.sensor_tip_distance_samples(reference)


@pytest.mark.parametrize("outside_wall", [False, True])
def test_no_seed_within_existing_fit_radius_remains_unavailable(outside_wall):
    values, info, radius_squared = seed_scene()
    if outside_wall:
        values[radius_squared <= SURFACE_ORIENTATION_MAXIMUM_FIT_RADIUS_PX**2] = 0.0
    else:
        values[:] = 0.0
    image = make_32fc1(values.ravel(), 161, 161)

    with pytest.raises(ValueError, match="No valid registered hand depth"):
        estimate_live_surface_normal(image, info)


def test_valid_near_original_seed_is_not_replaced_by_farther_expanded_depth():
    values, info, radius_squared = seed_scene()
    values[radius_squared <= 16**2] = 0.170
    image = make_32fc1(values.ravel(), 161, 161)

    with pytest.raises(ValueError, match=r"camera distance 0\.170 m, minimum 0\.210 m"):
        estimate_live_surface_normal(image, info)


def test_expanded_seed_preserves_camera_minimum_distance():
    values, info, radius_squared = seed_scene()
    values[(radius_squared >= 18**2) & (radius_squared <= 32**2)] = 0.170
    image = make_32fc1(values.ravel(), 161, 161)

    with pytest.raises(ValueError, match=r"camera distance 0\.170 m, minimum 0\.210 m"):
        estimate_live_surface_normal(image, info)


def test_ambiguous_original_seed_is_not_replaced_by_expanded_dominant_wall():
    values, info, radius_squared = seed_scene()
    _, columns = np.indices(values.shape)
    values[(radius_squared <= 16**2) & (columns < 80)] = 0.300
    values[(radius_squared <= 16**2) & (columns >= 80)] = 0.450
    image = make_32fc1(values.ravel(), 161, 161)

    with pytest.raises(ValueError, match="Ambiguous lower-middle depth"):
        estimate_live_surface_normal(image, info)
