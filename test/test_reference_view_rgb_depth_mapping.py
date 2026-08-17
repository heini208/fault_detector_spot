"""Tests for ROS image-geometry RGB-to-depth mapping."""

import struct

import pytest
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    map_rgb_pixel_to_depth,
    project_reference_pixel,
    rgb_depth_overlap_region,
    rgb_depth_selectable_region,
)
from fault_detector_spot.inspection.setup.reference_view_surface_normal import (
    estimate_reference_surface_normal,
)


def make_depth(values, width, height):
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.width = width
    image.height = height
    image.encoding = "32FC1"
    image.step = width * 4
    image.data = b"".join(struct.pack("<f", value) for value in values)
    return image


def make_camera_info(width, height, focal_length=100.0):
    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.width = width
    camera_info.height = height
    cx = (width - 1) / 2.0
    cy = (height - 1) / 2.0
    camera_info.k = [
        focal_length,
        0.0,
        cx,
        0.0,
        focal_length,
        cy,
        0.0,
        0.0,
        1.0,
    ]
    camera_info.p = [
        focal_length,
        0.0,
        cx,
        0.0,
        0.0,
        focal_length,
        cy,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]
    return camera_info


def test_equal_registered_rasters_need_no_calibration_mapping():
    result = map_rgb_pixel_to_depth(
        ImagePoint(u=3, v=2),
        rgb_size=(4, 3),
        depth_size=(4, 3),
    )

    assert result == ImagePoint(u=3, v=2)


def test_mismatched_rasters_require_camera_geometry():
    with pytest.raises(ValueError, match="require both CameraInfo"):
        map_rgb_pixel_to_depth(
            ImagePoint(u=6, v=4),
            rgb_size=(8, 6),
            depth_size=(4, 3),
        )


def test_ros_camera_geometry_maps_registered_resolution():
    result = map_rgb_pixel_to_depth(
        ImagePoint(u=6, v=4),
        rgb_size=(8, 6),
        depth_size=(4, 3),
        rgb_camera_info=make_camera_info(8, 6, focal_length=200.0),
        depth_camera_info=make_camera_info(4, 3, focal_length=100.0),
    )

    assert result == ImagePoint(u=3, v=2)


def test_half_pixel_projection_rounds_consistently_outward():
    with pytest.raises(ValueError, match="field of view"):
        map_rgb_pixel_to_depth(
            ImagePoint(u=4, v=4),
            rgb_size=(8, 6),
            depth_size=(4, 3),
            rgb_camera_info=make_camera_info(8, 6, focal_length=100.0),
            depth_camera_info=make_camera_info(4, 3, focal_length=100.0),
        )


def test_projection_uses_mapped_and_sampled_depth_coordinates():
    values = [0.0] * 12
    values[2 * 4 + 3] = 2.0
    result = project_reference_pixel(
        ImagePoint(u=6, v=4),
        make_depth(values, 4, 3),
        make_camera_info(4, 3, focal_length=100.0),
        rgb_size=(8, 6),
        rgb_camera_info=make_camera_info(8, 6, focal_length=200.0),
        search_radius_px=0,
    )

    assert result.requested_pixel == ImagePoint(u=6, v=4)
    assert result.mapped_pixel == ImagePoint(u=3, v=2)
    assert result.sampled_pixel == ImagePoint(u=3, v=2)
    assert result.point_camera.x == pytest.approx(0.03)
    assert result.point_camera.y == pytest.approx(0.02)


def test_nearby_depth_hole_uses_actual_sample_ray():
    values = [0.0] * 12
    values[1 * 4 + 3] = 1.5
    result = project_reference_pixel(
        ImagePoint(u=4, v=2),
        make_depth(values, 4, 3),
        make_camera_info(4, 3, focal_length=100.0),
        rgb_size=(8, 6),
        rgb_camera_info=make_camera_info(8, 6, focal_length=200.0),
        search_radius_px=1,
    )

    assert result.mapped_pixel == ImagePoint(u=2, v=1)
    assert result.sampled_pixel == ImagePoint(u=3, v=1)
    assert result.point_camera.x == pytest.approx(0.0225)


def test_normal_fit_uses_registered_depth_neighborhood():
    depth = make_depth([1.0] * 121, 11, 11)
    depth_info = make_camera_info(11, 11, focal_length=100.0)
    projected = project_reference_pixel(
        ImagePoint(u=10, v=10),
        depth,
        depth_info,
        rgb_size=(22, 22),
        rgb_camera_info=make_camera_info(22, 22, focal_length=200.0),
    )

    result = estimate_reference_surface_normal(
        projected,
        depth,
        depth_info,
    )

    assert projected.mapped_pixel == ImagePoint(u=5, v=5)
    assert result.normal_camera.x == pytest.approx(0.0, abs=1e-6)
    assert result.normal_camera.y == pytest.approx(0.0, abs=1e-6)
    assert result.normal_camera.z == pytest.approx(-1.0, abs=1e-6)
    assert result.sample_count >= 40


def test_rgb_bounds_are_checked_before_depth_mapping():
    with pytest.raises(ValueError, match="RGB reference image"):
        project_reference_pixel(
            ImagePoint(u=8, v=0),
            make_depth([1.0] * 12, 4, 3),
            make_camera_info(4, 3),
            rgb_size=(8, 6),
            rgb_camera_info=make_camera_info(8, 6),
        )


def test_camera_geometry_distinguishes_crop_from_lower_resolution():
    rgb_info = make_camera_info(8, 6, focal_length=100.0)
    depth_info = make_camera_info(4, 3, focal_length=100.0)

    region = rgb_depth_overlap_region(
        (8, 6),
        (4, 3),
        rgb_info,
        depth_info,
    )

    assert region.x == 2
    assert region.width == 4
    assert region.y == 1
    assert region.height == 3


def test_camera_geometry_rejects_rgb_border_outside_depth_crop():
    with pytest.raises(ValueError, match="field of view"):
        map_rgb_pixel_to_depth(
            ImagePoint(u=0, v=2),
            rgb_size=(8, 6),
            depth_size=(4, 3),
            rgb_camera_info=make_camera_info(8, 6, focal_length=100.0),
            depth_camera_info=make_camera_info(4, 3, focal_length=100.0),
        )


def test_selectable_region_excludes_zero_depth_border():
    values = []
    for _ in range(4):
        values.extend([0.0, 0.0, 1.0, 1.0, 1.0, 1.0])
    depth = make_depth(values, 6, 4)

    region = rgb_depth_selectable_region(
        (6, 4),
        depth,
        make_camera_info(6, 4),
        make_camera_info(6, 4),
    )

    assert region.x == 2
    assert region.y == 0
    assert region.width == 4
    assert region.height == 4


def test_selectable_region_rejects_empty_registered_depth():
    with pytest.raises(ValueError, match="valid depth support"):
        rgb_depth_selectable_region(
            (4, 3),
            make_depth([0.0] * 12, 4, 3),
            make_camera_info(4, 3),
            make_camera_info(4, 3),
        )


def test_overlap_excludes_exact_upper_half_pixel_boundary():
    rgb_info = make_camera_info(8, 6, focal_length=100.0)
    depth_info = make_camera_info(4, 3, focal_length=100.0)

    region = rgb_depth_overlap_region(
        (8, 6),
        (4, 3),
        rgb_info,
        depth_info,
    )

    assert region.y == 1
    assert region.height == 3
