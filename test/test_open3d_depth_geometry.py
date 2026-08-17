"""Tests for the Open3D registered-depth geometry adapter."""

import struct

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image

from fault_detector_spot.inspection.geometry.open3d_depth import (
    create_organized_depth_point_cloud,
)


def make_camera_info(width=3, height=2):
    camera_info = CameraInfo()
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
    image.width = width
    image.height = height
    image.encoding = "16UC1"
    image.is_bigendian = bigendian
    image.step = step
    image.data = b"".join(rows)
    return image


def make_32fc1(values, width=2, height=1, bigendian=False):
    prefix = ">" if bigendian else "<"
    image = Image()
    image.width = width
    image.height = height
    image.encoding = "32FC1"
    image.is_bigendian = bigendian
    image.step = width * 4
    image.data = b"".join(
        struct.pack(f"{prefix}f", value) for value in values
    )
    return image


def test_open3d_cloud_preserves_depth_pixel_layout():
    cloud = create_organized_depth_point_cloud(
        make_16uc1([0, 0, 0, 0, 0, 2000]),
        make_camera_info(),
    )

    assert cloud.points_camera.shape == (2, 3, 3)
    assert cloud.valid_mask.tolist() == [
        [False, False, False],
        [False, False, True],
    ]
    assert cloud.depth_m[1, 2] == pytest.approx(2.0)
    assert cloud.point_camera(2, 1) == pytest.approx(
        np.array([0.02, 0.005, 2.0])
    )


def test_open3d_cloud_handles_big_endian_padded_16uc1():
    cloud = create_organized_depth_point_cloud(
        make_16uc1(
            [0, 2500, 0, 0, 0, 0],
            bigendian=True,
            step=8,
        ),
        make_camera_info(),
    )

    assert cloud.depth_m[0, 1] == pytest.approx(2.5)
    assert cloud.point_camera(1, 0)[2] == pytest.approx(2.5)


def test_open3d_cloud_handles_big_endian_32fc1_and_invalid_values():
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
    cloud = create_organized_depth_point_cloud(
        make_32fc1([float("nan"), 1.25], bigendian=True),
        camera_info,
    )

    assert not cloud.valid_mask[0, 0]
    assert np.isnan(cloud.points_camera[0, 0]).all()
    assert cloud.depth_m[0, 1] == pytest.approx(1.25)
    assert cloud.point_camera(1, 0) == pytest.approx(
        np.array([0.0125, 0.0, 1.25])
    )
