"""Discover the probe-facing surface without supplying its distance."""

from copy import deepcopy

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry.sensor_tip_observation import (
    probe_axis_in_camera, project_camera_point, target_camera_point,
)
from fault_detector_spot.inspection.sensing.sensor_tip_distance import (
    SensorTipTargetOutsideView, measure_sensor_tip_distance,
)
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.geometry.rotation import quaternion_from_matrix, rotation_from_quaternion
from test_live_surface_distance import planar_depth
import test_sensor_tip_observation as observation_fixtures
from test_sensor_tip_observation import camera_pose


source = observation_fixtures.source


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


@pytest.mark.parametrize("distance", [0.20, 0.25, 0.379, 0.381])
def test_mount_geometry_aligns_original_axis_before_target_measurement(source, distance):
    camera = mounted_camera()
    set_view(source, camera, distance)
    view = source.source.observation_view(PoseData.identity())
    assert not view.visible
    delta = view.hand_translation_odom
    assert view.target_plane_probe is None
    assert source.fits == 0
    assert delta.x == pytest.approx(0.0, abs=1e-8)
    assert delta.y == pytest.approx(-camera.position.y, abs=1e-8)
    assert delta.z == pytest.approx(-camera.position.z, abs=1e-8)
    for axis in ("x", "y", "z"):
        setattr(camera.position, axis, getattr(camera.position, axis) + getattr(delta, axis))
    origin, direction = probe_axis_in_camera(PoseData.identity(), camera)
    assert np.linalg.norm(np.cross(origin, direction)) < 1e-8
    # Every distance along the original axis now projects to the same pixel.
    for probe_distance in (0.05, 0.25, 0.45):
        pixel = project_camera_point(
            target_camera_point(PoseData.identity(), camera, probe_distance),
            (552, 552, 320, 240),
        )
        assert pixel == pytest.approx(project_camera_point(direction, (552, 552, 320, 240)), abs=1e-5)
    set_view(source, camera, distance)
    observed = source.source.observation_view(PoseData.identity(), allow_translation=False)
    assert observed.visible
    assert observed.hand_translation_odom == Vector3Data.zero()
    # The camera has moved, but measurement still refers to the original tip.
    sample = source.source.sensor_tip_distance_samples(PoseData.identity())[-1]
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


def test_object_on_original_axis_is_measured_instead_of_initial_background(source):
    camera = mounted_camera()
    set_view(source, camera, 0.45)
    initial = source.source.observation_view(PoseData.identity())
    assert initial.target_plane_probe is None
    for axis in ("x", "y", "z"):
        setattr(camera.position, axis, getattr(camera.position, axis) + getattr(initial.hand_translation_odom, axis))

    background, info = wall_image(camera, 0.45)
    object_image, _ = wall_image(camera, 0.20)
    values = np.frombuffer(bytes(background.data), dtype="<f4").reshape(480, 640).copy()
    foreground = np.frombuffer(bytes(object_image.data), dtype="<f4").reshape(480, 640)
    u, v = np.rint(project_camera_point(
        target_camera_point(PoseData.identity(), camera, 0.20), (552, 552, 320, 240),
    )).astype(int)
    values[v-10:v+11, u-10:u+11] = foreground[v-10:v+11, u-10:u+11]
    background.data = values.tobytes()
    source.source._receive_hand_depth_camera_info(info)
    source.source._receive_hand_depth(background)
    source.source._lookup_pose = lambda *args, **kwargs: deepcopy(camera)

    observed = source.source.observation_view(PoseData.identity(), allow_translation=False)

    assert observed.visible
    assert observed.hand_translation_odom == Vector3Data.zero()
    assert observed.target_plane_probe.signed_distance(Vector3Data.zero()) == pytest.approx(0.20, abs=1e-6)


def test_postmove_target_uses_actual_camera_tf_despite_lateral_position_error(source):
    # The intended aligned camera pixel is (320, 240). The actual camera
    # stops 20 mm higher, so the original target appears around (320, 275).
    camera = camera_pose(0.020)
    background, info = wall_image(camera, 0.40)
    object_image, _ = wall_image(camera, 0.218)
    values = np.frombuffer(bytes(background.data), dtype="<f4").reshape(480, 640).copy()
    foreground = np.frombuffer(bytes(object_image.data), dtype="<f4").reshape(480, 640)
    u, v = np.rint(project_camera_point(
        target_camera_point(PoseData.identity(), camera, 0.218), (552, 552, 320, 240),
    )).astype(int)
    assert v - 240 > 30
    values[v-20:v+21, u-20:u+21] = foreground[v-20:v+21, u-20:u+21]
    background.data = values.tobytes()
    source.source._receive_hand_depth_camera_info(info)
    source.source._receive_hand_depth(background)
    source.source._lookup_pose = lambda *args, **kwargs: deepcopy(camera)

    observed = source.source.observation_view(PoseData.identity(), allow_translation=False)

    assert observed.visible
    assert observed.hand_translation_odom == Vector3Data.zero()
    assert observed.target_plane_probe.signed_distance(Vector3Data.zero()) == pytest.approx(0.218, abs=1e-6)
