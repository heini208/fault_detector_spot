"""Discover the probe-facing surface without supplying its distance."""

from copy import deepcopy

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry.sensor_tip_observation import observation_translation
from fault_detector_spot.inspection.sensing.sensor_tip_distance import (
    SensorTipTargetOutsideView, measure_sensor_tip_distance,
)
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.geometry.rotation import quaternion_from_matrix, rotation_from_quaternion
from test_live_surface_distance import planar_depth
from test_sensor_tip_observation import camera_pose, source


def mounted_camera():
    return PoseData(
        Vector3Data(-0.114589969, 0.0182147651, 0.130271103),
        quaternion_from_matrix(np.array([
            [0.000004199, 0.136085467, 0.990697101],
            [-1.0, -0.0000006196, 0.0000043235],
            [0.0000012022, -0.990697101, 0.136085467],
        ])),
    )


def wall_image(camera, distance, normal=(-1.0, 0.0, 0.0)):
    """Render a known wall in the original probe frame into camera depth."""
    image, info = planar_depth(width=640, height=480, depth_m=0.4)
    info.k[0] = info.k[4] = 552.0
    rows, columns = np.indices((480, 640))
    rays = np.stack(((columns - 320) / 552, (rows - 240) / 552, np.ones((480, 640))), axis=-1)
    rotation = rotation_from_quaternion(camera.orientation).as_matrix()
    rays_probe = rays @ rotation.T
    origin = np.array([camera.position.x, camera.position.y, camera.position.z])
    normal = np.asarray(normal, dtype=float)
    values = (np.dot([distance, 0, 0] - origin, normal) / (rays_probe @ normal)).astype("<f4")
    image.data = values.tobytes()
    return image, info


def set_view(state, camera, distance):
    image, info = wall_image(camera, distance)
    state.source._receive_hand_depth_camera_info(info)
    state.source._receive_hand_depth(image)
    state.source._lookup_pose = lambda *args, **kwargs: deepcopy(camera)


@pytest.mark.parametrize("distance", [0.379, 0.381])
def test_visible_wall_needs_neither_estimate_nor_unnecessary_observation_move(source, distance):
    set_view(source, mounted_camera(), distance)
    view = source.source.observation_view(PoseData.identity())
    assert view.visible
    assert view.hand_translation_odom == Vector3Data.zero()
    samples = source.source.sensor_tip_distance_samples(PoseData.identity())
    assert samples[-1].distance_m == pytest.approx(distance, abs=1e-6)


@pytest.mark.parametrize("distance", [0.20, 0.25])
def test_hidden_target_move_uses_depth_and_measures_original_tip_after_excursion(source, distance):
    camera = mounted_camera()
    set_view(source, camera, distance)
    view = source.source.observation_view(PoseData.identity())
    assert not view.visible
    expected_delta = observation_translation(
        PoseData.identity(), camera, distance, (552, 552, 320, 240), (640, 480), 36,
    )
    delta = view.hand_translation_odom
    assert delta.x == pytest.approx(0.0, abs=1e-8)
    assert delta.y == pytest.approx(expected_delta.y, abs=1e-6)
    assert delta.z == pytest.approx(expected_delta.z, abs=1e-6)
    assert -0.10 < delta.z < 0.0
    for axis in ("x", "y", "z"):
        setattr(camera.position, axis, getattr(camera.position, axis) + getattr(delta, axis))
    image, info = wall_image(camera, distance)
    # The camera has moved, but the measured reference remains the original tip.
    sample = measure_sensor_tip_distance(image, info, PoseData.identity(), camera)
    assert sample.distance_m == pytest.approx(distance, abs=1e-6)
    assert sample.surface_plane_probe.normal_array() == pytest.approx([-1, 0, 0], abs=1e-6)


def test_saved_distance_does_not_choose_observation_geometry(source):
    set_view(source, mounted_camera(), 0.25)
    discovered = source.source.observation_view(PoseData.identity())
    checked = source.source.observation_view(PoseData.identity(), 0.32)
    assert not discovered.visible and not checked.visible
    assert discovered.hand_translation_odom == checked.hand_translation_odom


def test_measurement_discovers_normal_and_distance_for_an_angled_wall():
    camera = camera_pose(0.02)
    normal = np.array([-1.0, 0.25, 0.10])
    normal /= np.linalg.norm(normal)
    image, info = wall_image(camera, 0.38, normal)
    sample = measure_sensor_tip_distance(image, info, PoseData.identity(), camera)
    assert sample.surface_plane_probe.normal_array() == pytest.approx(normal, abs=1e-6)
    assert sample.distance_m == pytest.approx(-normal[0] * 0.38, abs=1e-6)
    assert -sample.distance_m / sample.surface_plane_probe.normal.x == pytest.approx(0.38, abs=1e-6)


def test_farther_wall_cannot_hide_nearer_surface_along_probe_path():
    camera = camera_pose(0.15)
    image, info = wall_image(camera, 0.38)
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    # A nearer wall is visible near the bottom edge. Its probe-axis contact
    # is just outside the image; the far wall has depth exactly on the ray.
    values[450:, :] = 0.32
    image.data = values.tobytes()
    with pytest.raises(SensorTipTargetOutsideView) as error:
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera)
    assert error.value.axis_distance_m == pytest.approx(0.22, abs=1e-6)


def test_no_nearby_supported_surface_cannot_trigger_a_guessed_camera_move(source):
    set_view(source, camera_pose(0.40), 0.25)
    view = source.source.observation_view(PoseData.identity())
    assert not view.visible
    assert view.hand_translation_odom == Vector3Data.zero()
    assert "No measured depth near forward probe" in view.detail
