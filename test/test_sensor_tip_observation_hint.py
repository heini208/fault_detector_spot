"""A neighboring wall may guide observation, never replace target depth."""

from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry.sensor_tip_observation import (
    observation_translation, project_camera_point, target_camera_point,
)
from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.sensing.sensor_tip_distance import (
    SensorTipTargetUnavailable, measure_sensor_tip_distance,
)
from fault_detector_spot.inspection.sensing import sensor_tip_distance
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from test_sensor_tip_automatic_discovery import wall_image
from test_sensor_tip_observation import camera_pose, source


def set_image(source, image, info, camera):
    source.source._receive_hand_depth_camera_info(info)
    source.source._receive_hand_depth(image)
    source.source._lookup_pose = lambda *args, **kwargs: deepcopy(camera)


def unresolved_scene():
    camera = camera_pose(0.08)
    image, info = wall_image(camera, 0.218)
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    values[376:383, 317:324] = 0.0
    # A near return can lie inside the discovery corridor while its actual
    # probe-axis location is below the image. The wall target is in-frame.
    values[473, 371] = 0.170
    image.data = values.tobytes()
    return image, info, camera


def test_unresolved_in_frame_target_proposes_only_a_lateral_observation(source):
    image, info, camera = unresolved_scene()
    reference = PoseData.identity()
    set_image(source, image, info, camera)

    view = source.source.observation_view(reference)

    assert not view.visible
    assert "Target requires another view" in view.detail
    assert view.target_plane_probe.signed_distance(Vector3Data.zero()) == pytest.approx(0.218, abs=1e-6)
    delta = view.hand_translation_odom
    assert delta.x == pytest.approx(0.0, abs=1e-9)
    assert 0.001 < np.linalg.norm([delta.x, delta.y, delta.z]) < 0.20
    moved_camera = deepcopy(camera)
    for axis in ("x", "y", "z"):
        setattr(moved_camera.position, axis, getattr(camera.position, axis) + getattr(delta, axis))
    pixel = project_camera_point(
        target_camera_point(reference, moved_camera, 0.218), (552, 552, 320, 240),
    )
    centered_delta = observation_translation(
        reference, camera, 0.218, (552, 552, 320, 240), (640, 480), 36,
        desired_pixel=ImagePoint(320, 360),
    )
    assert np.linalg.norm([delta.x, delta.y, delta.z]) < np.linalg.norm(
        [centered_delta.x, centered_delta.y, centered_delta.z],
    )
    original_pixel = project_camera_point(
        target_camera_point(reference, camera, 0.218), (552, 552, 320, 240),
    )
    assert np.linalg.norm(np.asarray(pixel)-original_pixel) < 19
    # The hint never enters the measurement cache or becomes a distance.
    with pytest.raises(SensorTipTargetUnavailable, match="insufficient unoccluded"):
        measure_sensor_tip_distance(image, info, reference, camera)
    with pytest.raises(ValueError, match="insufficient unoccluded"):
        source.source.sensor_tip_distance_samples(reference)


def test_observation_still_requires_actual_original_target_after_camera_move(source):
    image, info, camera = unresolved_scene()
    reference = PoseData.identity()
    set_image(source, image, info, camera)
    delta = source.source.observation_view(reference).hand_translation_odom
    for axis in ("x", "y", "z"):
        setattr(camera.position, axis, getattr(camera.position, axis) + getattr(delta, axis))
    image, info = wall_image(camera, 0.218)
    # Attached near returns remain at their camera pixels after the shift.
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    values[473, 371] = 0.170
    image.data = values.tobytes()
    sample = measure_sensor_tip_distance(image, info, reference, camera)
    assert sample.distance_m == pytest.approx(0.218, abs=1e-6)

    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    u, v = np.rint(project_camera_point(
        target_camera_point(reference, camera, 0.218), (552, 552, 320, 240),
    )).astype(int)
    values[v-3:v+4, u-3:u+4] = 0.0
    image.data = values.tobytes()
    with pytest.raises(SensorTipTargetUnavailable, match="insufficient unoccluded"):
        measure_sensor_tip_distance(image, info, reference, camera)

    near_image, near_info = wall_image(camera, 0.10)
    with pytest.raises(SensorTipTargetUnavailable, match="too close.*minimum 0.210 m"):
        measure_sensor_tip_distance(near_image, near_info, reference, camera)


@pytest.mark.parametrize("foreground_depth", [0.170, 0.217])
def test_observation_ignores_unfitted_camera_attached_foreground(source, foreground_depth):
    image, info, camera = unresolved_scene()
    reference = PoseData.identity()
    fixed_delta = observation_translation(
        reference, camera, 0.218, (552, 552, 320, 240), (640, 480), 36,
        desired_pixel=ImagePoint(320, 360),
    )
    fixed_camera = deepcopy(camera)
    fixed_camera.position.z += fixed_delta.z
    # This unsupported return lies 20 mm beside the frozen probe ray. Its
    # camera pixel and missing near-ray footprint stay fixed as the hand
    # moves; neither may replace valid depth at the actual wall contact.
    u = round(320 + 552 * 0.020 / foreground_depth)
    v = round(240 + 552 * fixed_camera.position.z / foreground_depth)
    fixed_image, fixed_info = wall_image(fixed_camera, 0.218)
    fixed_values = np.frombuffer(bytes(fixed_image.data), dtype="<f4").reshape(480, 640).copy()
    fixed_values[v-3:v+4, 317:324] = 0.0
    fixed_values[v, u] = foreground_depth
    fixed_image.data = fixed_values.tobytes()
    sample = measure_sensor_tip_distance(fixed_image, fixed_info, reference, fixed_camera)
    assert sample.distance_m == pytest.approx(0.218, abs=1e-6)

    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    values[473, 371] = 0.0
    values[v-3:v+4, 317:324] = 0.0
    values[v, u] = foreground_depth
    image.data = values.tobytes()
    set_image(source, image, info, camera)

    view = source.source.observation_view(reference)
    assert not view.visible
    assert view.hand_translation_odom != Vector3Data.zero()
    for axis in ("x", "y", "z"):
        setattr(camera.position, axis, getattr(camera.position, axis) + getattr(view.hand_translation_odom, axis))
    moved_image, moved_info = wall_image(camera, 0.218)
    moved_values = np.frombuffer(bytes(moved_image.data), dtype="<f4").reshape(480, 640).copy()
    moved_values[v-3:v+4, 317:324] = 0.0
    moved_values[v, u] = foreground_depth
    moved_image.data = moved_values.tobytes()

    sample = measure_sensor_tip_distance(moved_image, moved_info, reference, camera)
    assert sample.distance_m == pytest.approx(0.218, abs=1e-6)


def test_no_positioning_margin_rejects_observation_with_bounded_patch_checks(monkeypatch):
    camera = camera_pose(0.03)
    image, info = wall_image(camera, 0.30)
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    values[::12, ::12] = 0.200
    image.data = values.tobytes()
    checks = []
    original = sensor_tip_distance._target_patch

    def counted(*args, **kwargs):
        checks.append(1)
        return original(*args, **kwargs)

    monkeypatch.setattr(sensor_tip_distance, "_target_patch", counted)

    with pytest.raises(ValueError, match="positioning margin"):
        sensor_tip_distance.sensor_tip_observation_hint(
            image, info, PoseData.identity(), camera,
        )
    assert len(checks) <= sensor_tip_distance.MAXIMUM_SEED_FITS


def test_observation_rejects_a_locally_planar_patch_with_a_different_normal(monkeypatch):
    image, info, camera = unresolved_scene()
    original = sensor_tip_distance.estimate_surface_normal
    calls = []

    def tilted(*args, **kwargs):
        estimate = original(*args, **kwargs)
        calls.append(1)
        plane = replace(
            estimate.surface_plane_camera,
            normal=Vector3Data(0.0, np.sin(np.radians(40)), -np.cos(np.radians(40))),
        )
        return replace(estimate, surface_plane_camera=plane)

    monkeypatch.setattr(sensor_tip_distance, "estimate_surface_normal", tilted)

    with pytest.raises(ValueError, match="No nearby observation patch"):
        sensor_tip_distance.sensor_tip_observation_hint(
            image, info, PoseData.identity(), camera,
        )
    assert len(calls) == sensor_tip_distance.MAXIMUM_SEED_FITS


def test_observation_patch_validation_preserves_the_original_target(monkeypatch):
    image, info, camera = unresolved_scene()
    original = sensor_tip_distance.estimate_surface_normal

    def offset(*args, **kwargs):
        estimate = original(*args, **kwargs)
        point = deepcopy(estimate.surface_plane_camera.point)
        point.z += 0.004
        return replace(estimate, surface_plane_camera=replace(estimate.surface_plane_camera, point=point))

    monkeypatch.setattr(sensor_tip_distance, "estimate_surface_normal", offset)

    plane, _ = sensor_tip_distance.sensor_tip_observation_hint(
        image, info, PoseData.identity(), camera,
    )

    assert plane.signed_distance(Vector3Data.zero()) == pytest.approx(0.218, abs=1e-6)
    assert plane.normal_array() == pytest.approx([-1.0, 0.0, 0.0], abs=1e-6)


def test_tilted_wall_margin_uses_target_depth_after_camera_translation(monkeypatch):
    camera = camera_pose(0.10)
    image, info = wall_image(camera, 0.118, normal=(-1.0, 0.0, 0.8))
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    values[425:, :] = 0.0
    image.data = values.tobytes()
    margins = []
    original = sensor_tip_distance._target_patch

    def checked(*args, **kwargs):
        result = original(*args, **kwargs)
        margins.append(kwargs["minimum_support_margin_px"])
        return result

    monkeypatch.setattr(sensor_tip_distance, "_target_patch", checked)

    reference = PoseData.identity()
    plane, pixel = sensor_tip_distance.sensor_tip_observation_hint(image, info, reference, camera)
    distance = -plane.signed_distance(Vector3Data.zero())/plane.normal.x
    delta = observation_translation(
        reference, camera, distance, (552, 552, 320, 240), (640, 480), 36, desired_pixel=pixel,
    )
    moved_camera = deepcopy(camera)
    for axis in ("x", "y", "z"):
        setattr(moved_camera.position, axis, getattr(camera.position, axis) + getattr(delta, axis))
    target_depth = target_camera_point(reference, moved_camera, distance)[2]
    candidate_depth = values[pixel.v, pixel.u]
    assert target_depth < candidate_depth
    required_margin = 552*sensor_tip_distance.OBSERVATION_TARGET_MARGIN_M/target_depth
    assert margins[-1] >= required_margin

    cloud = sensor_tip_distance.create_organized_depth_point_cloud(image, info)
    plane = sensor_tip_distance.estimate_live_surface_normal(image, info).surface_plane_camera
    support = sensor_tip_distance._observation_support_mask(cloud, plane)
    radius = int(np.ceil(required_margin))
    assert support[pixel.v-radius:pixel.v+radius+1, pixel.u-radius:pixel.u+radius+1].all()


def test_a_true_near_wall_cannot_supply_an_observation_hint(source):
    camera = camera_pose()
    image, info = wall_image(camera, 0.10)
    set_image(source, image, info, camera)

    view = source.source.observation_view(PoseData.identity())

    assert not view.visible
    assert view.hand_translation_odom == Vector3Data.zero()
    assert "too close" in view.detail


@pytest.mark.parametrize("expected", [0.04, 0.45])
def test_unresolved_target_cannot_use_a_hint_for_a_different_saved_distance(source, expected):
    image, info, camera = unresolved_scene()
    set_image(source, image, info, camera)

    view = source.source.observation_view(PoseData.identity(), expected)

    assert not view.visible
    assert view.hand_translation_odom == Vector3Data.zero()


def test_unknown_target_with_no_reliable_neighbor_cannot_propose_movement(source):
    image, info, camera = unresolved_scene()
    values = np.zeros((480, 640), dtype="<f4")
    values[473, 371] = 0.170
    image.data = values.tobytes()
    set_image(source, image, info, camera)

    view = source.source.observation_view(PoseData.identity())

    assert not view.visible
    assert view.hand_translation_odom == Vector3Data.zero()


def test_valid_target_still_needs_no_observation_move(source):
    camera = camera_pose(0.08)
    image, info = wall_image(camera, 0.218)
    set_image(source, image, info, camera)

    view = source.source.observation_view(PoseData.identity())

    assert view.visible
    assert view.hand_translation_odom == Vector3Data.zero()
