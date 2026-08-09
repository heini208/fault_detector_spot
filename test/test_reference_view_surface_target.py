"""Tests for object-frame surface-facing target pose generation."""

import math

import numpy as np
import pytest

from fault_detector_spot.inspection.models import (
    ImagePoint,
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.reference_view_approach_direction import (
    APPROACH_SOURCE_SURFACE_FIT,
    ReferenceApproachDirection,
)
from fault_detector_spot.inspection.reference_view_depth_projection import (
    ProjectedReferencePoint,
)
from fault_detector_spot.inspection.reference_view_surface_target import (
    quaternion_to_rpy,
    resolve_reference_surface_target,
)


def projected_point(point):
    return ProjectedReferencePoint(
        requested_pixel=ImagePoint(u=5, v=5),
        mapped_pixel=ImagePoint(u=5, v=5),
        sampled_pixel=ImagePoint(u=5, v=5),
        point_camera=Vector3Data(*point),
        frame_id="hand_color_image_sensor",
        depth_m=float(point[2]),
    )


def approach(point, direction):
    return ReferenceApproachDirection(
        projected_point=projected_point(point),
        direction_camera=Vector3Data(*direction),
        source=APPROACH_SOURCE_SURFACE_FIT,
    )


def quaternion_matrix(quaternion):
    x, y, z, w = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    )


def test_identity_tag_geometry_produces_identity_orientation():
    result = resolve_reference_surface_target(
        approach((1.0, 2.0, 3.0), (1.0, 0.0, 0.0)),
        PoseData.identity(),
        target_surface_distance_m=0.03,
        aligned_preapproach_distance_m=0.20,
    )

    assert result.surface_point_object == Vector3Data(1.0, 2.0, 3.0)
    assert result.target_pose_object.position.x == pytest.approx(1.03)
    assert result.target_pose_object.position.y == pytest.approx(2.0)
    assert result.target_pose_object.position.z == pytest.approx(3.0)
    assert result.aligned_preapproach_pose_object.position.x == pytest.approx(
        1.20
    )
    assert result.target_pose_object.orientation == QuaternionData.identity()
    assert (
        result.aligned_preapproach_pose_object.orientation
        == result.target_pose_object.orientation
    )


def test_camera_pose_transforms_surface_and_direction_into_object_frame():
    half_sqrt = math.sqrt(0.5)
    camera_pose = PoseData(
        position=Vector3Data(10.0, 0.0, 0.0),
        orientation=QuaternionData(
            x=0.0,
            y=0.0,
            z=half_sqrt,
            w=half_sqrt,
        ),
    )

    result = resolve_reference_surface_target(
        approach((1.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
        camera_pose,
        target_surface_distance_m=0.05,
        aligned_preapproach_distance_m=0.25,
    )

    assert result.surface_point_object.x == pytest.approx(10.0)
    assert result.surface_point_object.y == pytest.approx(1.0)
    assert result.surface_point_object.z == pytest.approx(1.0)
    assert result.target_pose_object.position.x == pytest.approx(10.0)
    assert result.target_pose_object.position.y == pytest.approx(1.05)
    assert result.aligned_preapproach_pose_object.position.y == pytest.approx(
        1.25
    )
    rotation = quaternion_matrix(result.target_pose_object.orientation)
    assert rotation[:, 0] == pytest.approx([0.0, 1.0, 0.0])


def test_orientation_local_x_matches_tilted_surface_outward_axis():
    outward = np.array([1.0, 0.0, 1.0])
    outward = outward / np.linalg.norm(outward)
    result = resolve_reference_surface_target(
        approach((0.0, 0.0, 1.0), tuple(outward)),
        PoseData.identity(),
        target_surface_distance_m=0.04,
        aligned_preapproach_distance_m=0.18,
    )

    rotation = quaternion_matrix(result.target_pose_object.orientation)
    assert rotation[:, 0] == pytest.approx(outward)
    assert np.linalg.det(rotation) == pytest.approx(1.0)
    assert rotation.T @ rotation == pytest.approx(np.eye(3))


def test_target_and_aligned_positions_use_their_own_distances():
    result = resolve_reference_surface_target(
        approach((0.0, 0.0, 1.0), (0.0, 0.0, -1.0)),
        PoseData.identity(),
        target_surface_distance_m=0.03,
        aligned_preapproach_distance_m=0.15,
    )

    assert result.target_pose_object.position.z == pytest.approx(0.97)
    assert result.aligned_preapproach_pose_object.position.z == pytest.approx(
        0.85
    )


@pytest.mark.parametrize(
    "target_distance,preapproach_distance,message",
    [
        (0.0, 0.15, "Target surface distance"),
        (0.03, 0.0, "Aligned pre-approach distance"),
        (0.03, 0.03, "at least 0.05 m"),
        (0.03, 0.079, "at least 0.05 m"),
        (0.10, 0.05, "at least 0.05 m"),
    ],
)
def test_invalid_distances_are_rejected(
    target_distance,
    preapproach_distance,
    message,
):
    with pytest.raises(ValueError, match=message):
        resolve_reference_surface_target(
            approach((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
            PoseData.identity(),
            target_surface_distance_m=target_distance,
            aligned_preapproach_distance_m=preapproach_distance,
        )


def test_exact_minimum_distance_separation_is_accepted():
    result = resolve_reference_surface_target(
        approach((0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
        PoseData.identity(),
        target_surface_distance_m=0.10,
        aligned_preapproach_distance_m=0.15,
    )

    assert result.aligned_preapproach_distance_m == pytest.approx(0.15)


def test_quaternion_to_rpy_reports_generated_yaw():
    half_sqrt = math.sqrt(0.5)
    roll, pitch, yaw = quaternion_to_rpy(
        QuaternionData(0.0, 0.0, half_sqrt, half_sqrt)
    )

    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(math.pi / 2.0)
