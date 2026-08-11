"""Tests for registered-depth projection of selected reference pixels."""

import struct

import pytest
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.data.models import ImagePoint
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    project_reference_pixel,
)


def make_camera_info(width=3, height=2):
    """Create one calibrated registered-depth camera model."""
    camera_info = CameraInfo()
    camera_info.header.frame_id = "hand_color_image_sensor"
    camera_info.width = width
    camera_info.height = height
    camera_info.k = [
        100.0,
        0.0,
        1.0,
        0.0,
        200.0,
        0.5,
        0.0,
        0.0,
        1.0,
    ]
    return camera_info


def make_16uc1(values, width=3, height=2, bigendian=False, step=None):
    """Create a depth image from millimetre samples."""
    step = step or width * 2
    rows = []
    byteorder = "big" if bigendian else "little"
    for row_index in range(height):
        row = bytearray()
        for value in values[
            row_index * width:(row_index + 1) * width
        ]:
            row.extend(int(value).to_bytes(2, byteorder=byteorder))
        row.extend(bytes(step - width * 2))
        rows.append(bytes(row))
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.width = width
    image.height = height
    image.encoding = "16UC1"
    image.is_bigendian = bigendian
    image.step = step
    image.data = b"".join(rows)
    return image


def make_32fc1(values, width=2, height=1, bigendian=False):
    """Create a floating-point depth image in metres."""
    prefix = ">" if bigendian else "<"
    image = Image()
    image.header.frame_id = "hand_color_image_sensor"
    image.width = width
    image.height = height
    image.encoding = "32FC1"
    image.is_bigendian = bigendian
    image.step = width * 4
    image.data = b"".join(
        struct.pack(f"{prefix}f", value) for value in values
    )
    return image


def test_projects_exact_16uc1_pixel_into_camera_frame():
    """Millimetre depth is back-projected with the camera matrix."""
    result = project_reference_pixel(
        ImagePoint(u=2, v=1),
        make_16uc1([0, 0, 0, 0, 0, 2000]),
        make_camera_info(),
        search_radius_px=0,
    )

    assert result.depth_m == pytest.approx(2.0)
    assert result.point_camera.x == pytest.approx(0.02)
    assert result.point_camera.y == pytest.approx(0.005)
    assert result.point_camera.z == pytest.approx(2.0)
    assert result.sampled_pixel == ImagePoint(u=2, v=1)
    assert result.frame_id == "hand_color_image_sensor"


def test_nearest_valid_depth_fills_a_small_depth_hole():
    """The closest valid local sample can replace a zero-depth pixel."""
    result = project_reference_pixel(
        ImagePoint(u=1, v=1),
        make_16uc1([0, 0, 0, 0, 0, 1500]),
        make_camera_info(),
        search_radius_px=1,
    )

    assert result.requested_pixel == ImagePoint(u=1, v=1)
    assert result.sampled_pixel == ImagePoint(u=2, v=1)
    assert result.depth_m == pytest.approx(1.5)


def test_reads_big_endian_depth_with_padded_rows():
    """Depth extraction respects endianness and row stride."""
    result = project_reference_pixel(
        ImagePoint(u=1, v=0),
        make_16uc1(
            [0, 2500, 0, 0, 0, 0],
            bigendian=True,
            step=8,
        ),
        make_camera_info(),
        search_radius_px=0,
    )

    assert result.depth_m == pytest.approx(2.5)


def test_projects_32fc1_depth_in_metres():
    """Floating-point registered depth is used without unit conversion."""
    camera_info = make_camera_info(width=2, height=1)
    camera_info.k = [
        100.0,
        0.0,
        0.0,
        0.0,
        100.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    result = project_reference_pixel(
        ImagePoint(u=1, v=0),
        make_32fc1([0.0, 1.25], bigendian=True),
        camera_info,
        search_radius_px=0,
    )

    assert result.depth_m == pytest.approx(1.25)
    assert result.point_camera.x == pytest.approx(0.0125)


def test_missing_local_depth_is_rejected():
    """A point cannot be accepted without a nearby valid measurement."""
    with pytest.raises(ValueError, match="No valid depth"):
        project_reference_pixel(
            ImagePoint(u=1, v=1),
            make_16uc1([0, 0, 0, 0, 0, 0]),
            make_camera_info(),
            search_radius_px=1,
        )


def test_out_of_bounds_pixel_is_rejected():
    """Selected pixels must belong to the registered depth image."""
    with pytest.raises(ValueError, match="outside"):
        project_reference_pixel(
            ImagePoint(u=3, v=1),
            make_16uc1([1000] * 6),
            make_camera_info(),
        )


def test_mismatched_camera_dimensions_are_rejected():
    """Projection cannot mix calibration from another image size."""
    with pytest.raises(ValueError, match="width does not match"):
        project_reference_pixel(
            ImagePoint(u=1, v=1),
            make_16uc1([1000] * 6),
            make_camera_info(width=4, height=2),
        )
