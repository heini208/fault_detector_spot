"""Depth-independent observation of the saved sensor-tip forward axis."""

import copy

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry.pose import compose_poses
from fault_detector_spot.inspection.geometry.sensor_tip_observation import (
    probe_axis_observation_translation,
    project_camera_point,
    target_camera_point,
)
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.geometry.rotation import (
    quaternion_from_euler,
    quaternion_from_matrix,
    rotation_from_quaternion,
)


INTRINSICS = (400.0, 410.0, 320.0, 240.0)
IMAGE_SIZE = (640, 480)
MARGIN = 36


def probe_pose(origin, direction=(0.0, 0.0, 1.0)):
    axis = np.asarray(direction, dtype=float)
    axis /= np.linalg.norm(axis)
    side = np.cross([0.0, 1.0, 0.0], axis)
    side /= np.linalg.norm(side)
    rotation = np.column_stack((axis, side, np.cross(axis, side)))
    return PoseData(Vector3Data(*origin), quaternion_from_matrix(rotation))


def vector_array(vector):
    return np.array([vector.x, vector.y, vector.z])


def observation_delta(reference, camera=None, **overrides):
    arguments = dict(
        intrinsics=INTRINSICS, image_size=IMAGE_SIZE, patch_margin_px=MARGIN,
    )
    arguments.update(overrides)
    return probe_axis_observation_translation(
        reference, camera or PoseData.identity(), **arguments,
    )


@pytest.mark.parametrize("origin,direction", [
    ((0.02, 0.08, 0.09), (0.0, 0.0, 1.0)),
    ((-0.06, 0.04, 0.12), (0.1, 0.14, 1.0)),
    ((0.07, -0.03, 0.18), (-0.2, -0.1, 1.0)),
])
def test_all_forward_ranges_share_one_pixel_after_perpendicular_translation(origin, direction):
    reference = probe_pose(origin, direction)
    camera = PoseData.identity()
    saved_reference, saved_camera = copy.deepcopy((reference, camera))
    delta = observation_delta(reference, camera)
    axis = rotation_from_quaternion(reference.orientation).apply([1.0, 0.0, 0.0])
    assert np.dot(vector_array(delta), axis) == pytest.approx(0.0, abs=1e-12)
    moved_camera = PoseData(delta, camera.orientation)
    expected_pixel = project_camera_point(axis, INTRINSICS)
    for distance in (0.001, 0.03, 0.25, 3.0):
        target = target_camera_point(reference, moved_camera, distance)
        assert target[2] > 0.0
        assert project_camera_point(target, INTRINSICS) == pytest.approx(expected_pixel)
    assert reference == saved_reference
    assert camera == saved_camera


def test_observation_delta_rotates_with_reference_without_changing_projection():
    reference = probe_pose((0.02, 0.08, 0.09), (0.0, 0.14, 1.0))
    camera = PoseData.identity()
    anchor = PoseData(
        Vector3Data(1.5, -2.0, 0.4),
        quaternion_from_euler("xyz", (0.4, -0.7, 1.2)),
    )
    delta = observation_delta(reference, camera)
    rotated_reference = compose_poses(anchor, reference)
    rotated_camera = compose_poses(anchor, camera)
    rotated_delta = observation_delta(rotated_reference, rotated_camera)
    assert vector_array(rotated_delta) == pytest.approx(
        rotation_from_quaternion(anchor.orientation).apply(vector_array(delta)),
    )
    rotated_camera.position = Vector3Data(*(
        vector_array(rotated_camera.position) + vector_array(rotated_delta)
    ))
    assert project_camera_point(
        target_camera_point(rotated_reference, rotated_camera, 0.15), INTRINSICS,
    ) == pytest.approx((320.0, 297.4))


@pytest.mark.parametrize("axial_offset", [0.0, 0.10])
def test_already_collinear_camera_needs_no_translation(axial_offset):
    reference = probe_pose((0.0, 0.0, axial_offset))
    assert vector_array(observation_delta(reference)) == pytest.approx([0.0, 0.0, 0.0])


@pytest.mark.parametrize("direction", [(0.0, 0.0, -1.0), (1.0, 0.0, 0.0)])
def test_axis_must_face_optically_forward(direction):
    with pytest.raises(ValueError, match="does not face forward"):
        observation_delta(probe_pose((0.0, 0.0, 0.1), direction))


@pytest.mark.parametrize("direction", [(0.8, 0.0, 1.0), (0.0, 0.6, 1.0)])
def test_axis_pixel_must_preserve_full_plane_fit_margin(direction):
    with pytest.raises(ValueError, match="full observation patch margin"):
        observation_delta(probe_pose((0.0, 0.0, 0.1), direction))


def test_camera_ahead_of_tip_cannot_assume_unknown_forward_targets_remain_visible():
    with pytest.raises(ValueError, match="ahead of the original sensor tip"):
        observation_delta(probe_pose((0.02, 0.08, -0.01)))


@pytest.mark.parametrize("overrides,error", [
    ({"intrinsics": (0.0, 410.0, 320.0, 240.0)}, "valid camera intrinsics"),
    ({"intrinsics": (400.0, 410.0, float("nan"), 240.0)}, "valid camera intrinsics"),
    ({"image_size": (72, 480)}, "too small"),
    ({"image_size": (640, float("inf"))}, "too small"),
    ({"patch_margin_px": -1}, "patch margin"),
    ({"patch_margin_px": float("nan")}, "patch margin"),
])
def test_invalid_calibration_or_patch_bounds_are_rejected(overrides, error):
    with pytest.raises(ValueError, match=error):
        observation_delta(probe_pose((0.02, 0.08, 0.09)), **overrides)


@pytest.mark.parametrize("invalid_pose", ["reference", "camera"])
def test_invalid_pose_is_rejected_before_planning(invalid_pose):
    reference = probe_pose((0.02, 0.08, 0.09))
    camera = PoseData.identity()
    (reference if invalid_pose == "reference" else camera).position.x = float("nan")
    with pytest.raises(ValueError, match="finite"):
        observation_delta(reference, camera)
