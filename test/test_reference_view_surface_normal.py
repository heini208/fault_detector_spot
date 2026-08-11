"""Tests for local surface-normal estimation from registered depth."""

import math
import struct

import pytest
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.data.models import ImagePoint
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    project_reference_pixel,
)
from fault_detector_spot.inspection.setup.reference_view_surface_normal import (
    estimate_reference_surface_normal,
)


def make_camera_info(width=11, height=11, focal_length=100.0):
    """Create a centered registered-depth camera model."""
    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.width = width
    camera_info.height = height
    camera_info.k = [
        focal_length,
        0.0,
        (width - 1) / 2.0,
        0.0,
        focal_length,
        (height - 1) / 2.0,
        0.0,
        0.0,
        1.0,
    ]
    return camera_info


def make_32fc1(values, width=11, height=11):
    """Create one floating-point depth image in metres."""
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.width = width
    image.height = height
    image.encoding = "32FC1"
    image.step = width * 4
    image.data = b"".join(struct.pack("<f", value) for value in values)
    return image


def plane_depth_values(
    width,
    height,
    camera_info,
    normal,
    center_depth_m,
):
    """Render ideal ray-plane intersections into a depth image."""
    nx, ny, nz = normal
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]
    values = []
    for v in range(height):
        for u in range(width):
            ray_x = (u - cx) / fx
            ray_y = (v - cy) / fy
            denominator = nx * ray_x + ny * ray_y + nz
            values.append(nz * center_depth_m / denominator)
    return values


def estimate(depth_image, camera_info, pixel=None, **kwargs):
    """Project the selected point and estimate its local normal."""
    pixel = pixel or ImagePoint(u=5, v=5)
    projected = project_reference_pixel(pixel, depth_image, camera_info)
    return estimate_reference_surface_normal(
        projected,
        depth_image,
        camera_info,
        **kwargs,
    )


def test_front_facing_plane_normal_points_toward_camera():
    """A flat frontal surface produces a camera-facing negative-Z normal."""
    camera_info = make_camera_info()
    depth = make_32fc1([1.0] * 121)

    result = estimate(depth, camera_info)

    assert result.normal_camera.x == pytest.approx(0.0, abs=1e-6)
    assert result.normal_camera.y == pytest.approx(0.0, abs=1e-6)
    assert result.normal_camera.z == pytest.approx(-1.0, abs=1e-6)
    assert result.sample_count >= 40
    assert result.plane_rmse_m == pytest.approx(0.0, abs=1e-7)


def test_tilted_plane_normal_is_recovered_and_normalized():
    """Plane fitting recovers a tilted camera-facing surface direction."""
    camera_info = make_camera_info()
    desired = (0.2, -0.1, -math.sqrt(0.95))
    values = plane_depth_values(
        11,
        11,
        camera_info,
        desired,
        1.0,
    )
    depth = make_32fc1(values)

    result = estimate(depth, camera_info)
    normal = result.normal_camera

    assert normal.x == pytest.approx(desired[0], abs=1e-5)
    assert normal.y == pytest.approx(desired[1], abs=1e-5)
    assert normal.z == pytest.approx(desired[2], abs=1e-5)
    norm = math.sqrt(normal.x ** 2 + normal.y ** 2 + normal.z ** 2)
    assert norm == pytest.approx(1.0, abs=1e-6)


def test_background_depth_is_excluded_near_surface_edge():
    """A distant background cannot tilt the selected foreground normal."""
    width = 11
    height = 11
    camera_info = make_camera_info(width, height)
    values = []
    for _v in range(height):
        for u in range(width):
            values.append(1.0 if u <= 5 else 2.0)
    depth = make_32fc1(values, width, height)

    result = estimate(
        depth,
        camera_info,
        pixel=ImagePoint(u=4, v=5),
        maximum_depth_delta_m=0.05,
    )

    assert result.normal_camera.z == pytest.approx(-1.0, abs=1e-6)
    assert result.sample_count >= 20


def test_too_few_valid_samples_are_rejected():
    """A sparse depth neighborhood cannot define a surface plane."""
    camera_info = make_camera_info()
    values = [float("nan")] * 121
    for u, v in ((5, 5), (4, 5), (6, 5), (5, 4), (5, 6)):
        values[v * 11 + u] = 1.0
    depth = make_32fc1(values)

    with pytest.raises(ValueError, match="Too few consistent depth"):
        estimate(depth, camera_info)


def test_nonplanar_neighborhood_is_rejected_by_fit_error():
    """Depth variation that does not describe a plane remains invalid."""
    camera_info = make_camera_info()
    values = []
    for v in range(11):
        for u in range(11):
            values.append(0.98 if (u + v) % 2 == 0 else 1.02)
    depth = make_32fc1(values)

    with pytest.raises(ValueError, match="not planar enough"):
        estimate(
            depth,
            camera_info,
            maximum_depth_delta_m=0.05,
            maximum_plane_rmse_m=0.005,
        )


def test_invalid_configuration_is_rejected():
    """Estimator thresholds must be positive and meaningful."""
    camera_info = make_camera_info()
    depth = make_32fc1([1.0] * 121)
    projected = project_reference_pixel(
        ImagePoint(u=5, v=5),
        depth,
        camera_info,
    )

    with pytest.raises(ValueError, match="positive integer"):
        estimate_reference_surface_normal(
            projected,
            depth,
            camera_info,
            minimum_sample_count=0,
        )
