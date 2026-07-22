"""Tests for RGB-to-depth mapping in reference geometry."""

import struct

import pytest
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.models import ImagePoint
from fault_detector_spot.inspection.reference_view_depth_projection import (
    map_rgb_pixel_to_depth,
    project_reference_pixel,
)
from fault_detector_spot.inspection.reference_view_surface_normal import (
    estimate_reference_surface_normal,
)


def make_depth(values, width, height):
    """Create one floating-point registered depth image."""
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.width = width
    image.height = height
    image.encoding = "32FC1"
    image.step = width * 4
    image.data = b"".join(struct.pack("<f", value) for value in values)
    return image


def make_camera_info(width, height, focal_length=100.0):
    """Create calibration for the registered depth resolution."""
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


def test_maps_pixel_centres_between_registered_resolutions():
    """RGB coordinates are scaled into the depth raster."""
    result = map_rgb_pixel_to_depth(
        ImagePoint(u=6, v=4),
        rgb_size=(8, 6),
        depth_size=(4, 3),
    )

    assert result == ImagePoint(u=3, v=2)


def test_projection_uses_mapped_and_sampled_depth_coordinates():
    """Back-projection uses the actual depth sample, not the RGB pixel."""
    values = [0.0] * 12
    values[2 * 4 + 3] = 2.0
    result = project_reference_pixel(
        ImagePoint(u=6, v=4),
        make_depth(values, 4, 3),
        make_camera_info(4, 3),
        rgb_size=(8, 6),
        search_radius_px=0,
    )

    assert result.requested_pixel == ImagePoint(u=6, v=4)
    assert result.mapped_pixel == ImagePoint(u=3, v=2)
    assert result.sampled_pixel == ImagePoint(u=3, v=2)
    assert result.point_camera.x == pytest.approx(0.03)
    assert result.point_camera.y == pytest.approx(0.02)


def test_nearby_depth_hole_uses_actual_sample_ray():
    """A local hole search changes both depth and projection ray."""
    values = [0.0] * 12
    values[1 * 4 + 3] = 1.5
    result = project_reference_pixel(
        ImagePoint(u=4, v=2),
        make_depth(values, 4, 3),
        make_camera_info(4, 3),
        rgb_size=(8, 6),
        search_radius_px=1,
    )

    assert result.mapped_pixel == ImagePoint(u=2, v=1)
    assert result.sampled_pixel == ImagePoint(u=3, v=1)
    assert result.point_camera.x == pytest.approx(0.0225)


def test_normal_fit_uses_mapped_depth_neighborhood():
    """A flat plane works when RGB and depth resolutions differ."""
    depth = make_depth([1.0] * 121, 11, 11)
    camera_info = make_camera_info(11, 11)
    projected = project_reference_pixel(
        ImagePoint(u=10, v=10),
        depth,
        camera_info,
        rgb_size=(22, 22),
    )

    result = estimate_reference_surface_normal(
        projected,
        depth,
        camera_info,
    )

    assert projected.mapped_pixel == ImagePoint(u=5, v=5)
    assert result.normal_camera.x == pytest.approx(0.0, abs=1e-6)
    assert result.normal_camera.y == pytest.approx(0.0, abs=1e-6)
    assert result.normal_camera.z == pytest.approx(-1.0, abs=1e-6)
    assert result.sample_count >= 40


def test_rgb_bounds_are_checked_before_depth_mapping():
    """Selections outside the displayed RGB image are rejected."""
    with pytest.raises(ValueError, match="RGB reference image"):
        project_reference_pixel(
            ImagePoint(u=8, v=0),
            make_depth([1.0] * 12, 4, 3),
            make_camera_info(4, 3),
            rgb_size=(8, 6),
        )
