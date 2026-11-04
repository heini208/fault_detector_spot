"""A neighboring wall may guide observation, never replace target depth."""

from copy import deepcopy

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry.sensor_tip_observation import (
    project_camera_point, target_camera_point,
)
from fault_detector_spot.inspection.sensing.sensor_tip_distance import (
    SensorTipTargetUnavailable, measure_sensor_tip_distance,
)
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
    delta = view.hand_translation_odom
    assert delta.x == pytest.approx(0.0, abs=1e-9)
    assert 0.001 < np.linalg.norm([delta.x, delta.y, delta.z]) < 0.20
    moved_camera = deepcopy(camera)
    for axis in ("x", "y", "z"):
        setattr(moved_camera.position, axis, getattr(camera.position, axis) + getattr(delta, axis))
    pixel = project_camera_point(
        target_camera_point(reference, moved_camera, 0.218), (552, 552, 320, 240),
    )
    assert pixel == pytest.approx((320, 360), abs=1e-5)
    # The hint never enters the measurement cache or becomes a distance.
    with pytest.raises(SensorTipTargetUnavailable, match="too close"):
        measure_sensor_tip_distance(image, info, reference, camera)
    with pytest.raises(ValueError, match="too close"):
        source.source.sensor_tip_distance_samples(reference)


def test_observation_still_requires_actual_original_target_after_camera_move(source):
    image, info, camera = unresolved_scene()
    reference = PoseData.identity()
    set_image(source, image, info, camera)
    delta = source.source.observation_view(reference).hand_translation_odom
    for axis in ("x", "y", "z"):
        setattr(camera.position, axis, getattr(camera.position, axis) + getattr(delta, axis))
    image, info = wall_image(camera, 0.218)
    sample = measure_sensor_tip_distance(image, info, reference, camera)
    assert sample.distance_m == pytest.approx(0.218, abs=1e-6)

    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    values[357:364, 317:324] = 0.0
    image.data = values.tobytes()
    with pytest.raises(SensorTipTargetUnavailable, match="insufficient unoccluded"):
        measure_sensor_tip_distance(image, info, reference, camera)

    near_image, near_info = wall_image(camera, 0.10)
    with pytest.raises(SensorTipTargetUnavailable, match="too close.*minimum 0.210 m"):
        measure_sensor_tip_distance(near_image, near_info, reference, camera)


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
