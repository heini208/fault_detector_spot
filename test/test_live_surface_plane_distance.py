"""Use a nearby visible plane when the mounted probe hides axial depth."""

import math

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry.depth_point_cloud import (
    create_organized_depth_point_cloud,
)
from fault_detector_spot.inspection.sensing.live_surface_distance import (
    measure_probe_surface_distance,
)
from fault_detector_spot.inspection.sensing.live_surface_plane import (
    estimate_live_surface_normal,
)
from fault_detector_spot.shared.geometry.models import QuaternionData
from fault_detector_spot.shared.geometry.rotation import rotation_from_quaternion
from test_live_surface_distance import probe_to_camera_pose
from test_surface_normal import make_camera_info, make_32fc1, plane_depth_values


def visible_patch(normal=(0.0, 0.0, -1.0)):
    camera = make_camera_info(161, 161, 300.0)
    values = np.array(plane_depth_values(161, 161, camera, normal, 0.6)).reshape(161, 161)
    mask = np.ones(values.shape, dtype=bool)
    mask[105:137, 70:92] = False
    values[mask] = np.nan
    image = make_32fc1(values.ravel(), 161, 161)
    image.header.stamp.sec = 12
    image.header.stamp.nanosec = 345
    return image, camera


@pytest.mark.parametrize("normal", [
    (0.0, 0.0, -1.0), (0.025, -0.035, -math.sqrt(1 - 0.025**2 - 0.035**2)),
])
def test_orientation_and_distance_share_plane_despite_missing_axial_pixels(normal):
    image, camera = visible_patch(normal)
    transform = probe_to_camera_pose(offset_m=0.2)
    cloud = create_organized_depth_point_cloud(image, camera)
    points = cloud.points_camera[cloud.valid_mask]
    probe_points = rotation_from_quaternion(transform.orientation).apply(points)
    probe_points += [transform.position.x, transform.position.y, transform.position.z]
    assert np.min(np.linalg.norm(probe_points[:, 1:], axis=1)) > 0.025

    orientation = estimate_live_surface_normal(image, camera)
    distance = measure_probe_surface_distance(image, camera, transform)

    assert distance.distance_m == pytest.approx(-0.4 * normal[2], abs=1e-6)
    assert distance.stamp_seconds == pytest.approx(12.000000345)
    assert distance.frame_id == orientation.projected_point.frame_id
    assert distance.sample_count == orientation.surface_plane_camera.inlier_count
    assert distance.surface_plane_probe.normal.x == pytest.approx(normal[2], abs=1e-6)
    assert distance.source_region.width > 0


@pytest.mark.parametrize("patch_gap_m", [0.103, 0.149])
def test_lower_mounted_probe_can_use_the_visible_plane(patch_gap_m):
    image, camera = visible_patch()
    transform = probe_to_camera_pose(offset_m=0.2)
    # The visible patch centroid is 81 mm below the camera optical axis.
    # Put the probe still lower, as with an attached sensor mount.
    transform.position.y = -0.081 - patch_gap_m

    sample = measure_probe_surface_distance(image, camera, transform)

    assert sample.distance_m == pytest.approx(0.4, abs=1e-6)
    assert sample.surface_plane_probe.point.y == pytest.approx(-patch_gap_m, abs=1e-6)
    assert sample.surface_plane_probe.normal.x == pytest.approx(-1.0, abs=1e-6)


def test_lower_mount_does_not_expand_the_plane_extrapolation_limit():
    image, camera = visible_patch()
    transform = probe_to_camera_pose(offset_m=0.2)
    transform.position.y = -0.081 - 0.151

    with pytest.raises(
        ValueError, match=r"patch is 0\.151 m.*maximum plane extrapolation is 0\.150 m",
    ):
        measure_probe_surface_distance(image, camera, transform)


@pytest.mark.parametrize("case, expected", [
    ("behind", "does not intersect forward"),
    ("far", "required range"),
    ("parallel", "does not intersect forward"),
    ("remote_patch", "maximum plane extrapolation"),
])
def test_visible_plane_cannot_guide_a_distant_or_backward_probe_axis(case, expected):
    image, camera = visible_patch()
    transform = probe_to_camera_pose(offset_m=0.2)
    if case == "behind":
        transform.position.x = -0.8
    elif case == "far":
        transform.position.x = 0.0
    elif case == "parallel":
        transform.orientation = QuaternionData.identity()
    else:
        transform.position.y = -0.25

    with pytest.raises(ValueError, match=expected):
        measure_probe_surface_distance(image, camera, transform)


def test_plane_fallback_preserves_minimum_fitted_support():
    image, camera = visible_patch()
    with pytest.raises(ValueError, match="insufficient fitted support"):
        measure_probe_surface_distance(
            image, camera, probe_to_camera_pose(offset_m=0.2), minimum_samples=1000,
        )


@pytest.mark.parametrize("axial_depth_m", [0.202, 0.24, 0.8])
def test_sparse_axial_evidence_cannot_be_replaced_by_a_neighboring_plane(axial_depth_m):
    image, camera = visible_patch()
    values = np.frombuffer(image.data, dtype='<f4').copy().reshape(161, 161)
    values[79:81, 78:82] = axial_depth_m
    image.data = values.tobytes()

    with pytest.raises(ValueError, match="insufficient support along probe local"):
        measure_probe_surface_distance(image, camera, probe_to_camera_pose(offset_m=0.2))


def test_no_visible_plane_does_not_substitute_a_distance():
    image, camera = visible_patch()
    values = np.full((161, 161), np.nan)
    values[5:15, 5:15] = 0.6
    image.data = values.astype('<f4').tobytes()
    with pytest.raises(ValueError, match="nearby surface-plane estimate rejected"):
        measure_probe_surface_distance(image, camera, probe_to_camera_pose(offset_m=0.2))
