"""Tests for resolving outward reference-view approach directions."""

import math

import pytest

from fault_detector_spot.inspection.models import (
    ImagePoint,
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.reference_view_approach_direction import (
    APPROACH_MODE_AUTOMATIC,
    APPROACH_MODE_SURFACE_FIT,
    APPROACH_MODE_TAG_X,
    APPROACH_SOURCE_SURFACE_FIT,
    APPROACH_SOURCE_TAG_X_SELECTED,
    resolve_reference_approach_direction,
)
from fault_detector_spot.inspection.reference_view_depth_projection import (
    ProjectedReferencePoint,
)
from fault_detector_spot.inspection.reference_view_surface_normal import (
    ReferenceSurfaceNormal,
)


def make_projected_point():
    """Create one selected point in the reference camera frame."""
    pixel = ImagePoint(u=12, v=8)
    return ProjectedReferencePoint(
        requested_pixel=pixel,
        sampled_pixel=ImagePoint(u=12, v=8),
        point_camera=Vector3Data(x=0.1, y=-0.02, z=0.8),
        frame_id="hand_color_image_sensor",
        depth_m=0.8,
    )


def make_pose(orientation=None):
    """Create the saved camera pose in the object/tag frame."""
    return PoseData(
        position=Vector3Data.zero(),
        orientation=orientation or QuaternionData.identity(),
    )


def make_surface_normal(projected_point, x, y, z):
    """Create a valid local surface-normal estimate."""
    return ReferenceSurfaceNormal(
        projected_point=projected_point,
        normal_camera=Vector3Data(x=x, y=y, z=z),
        sample_count=40,
        plane_rmse_m=0.001,
    )


def test_surface_fit_is_aligned_toward_the_reference_camera():
    """A fitted normal points toward the camera viewing the surface."""
    point = make_projected_point()
    normal = make_surface_normal(point, 0.0, 0.0, 1.0)

    result = resolve_reference_approach_direction(
        point,
        normal,
        make_pose(),
        mode=APPROACH_MODE_AUTOMATIC,
    )

    assert result.source == APPROACH_SOURCE_SURFACE_FIT
    assert result.direction_camera.x == pytest.approx(0.0)
    assert result.direction_camera.y == pytest.approx(0.0)
    assert result.direction_camera.z == pytest.approx(-1.0)
    assert result.surface_normal is normal


def test_automatic_mode_rejects_missing_surface_fit():
    """Automatic mode never assumes a tag mounting direction."""
    point = make_projected_point()

    with pytest.raises(ValueError, match="surface is uneven"):
        resolve_reference_approach_direction(
            point,
            None,
            make_pose(),
            mode=APPROACH_MODE_AUTOMATIC,
            surface_normal_unavailable_reason="surface is uneven",
        )


def test_tag_x_is_transformed_into_the_camera_frame():
    """Object-frame +X is rotated through the saved reference pose."""
    point = make_projected_point()
    half_sqrt = math.sqrt(0.5)
    pose = make_pose(
        QuaternionData(
            x=0.0,
            y=-half_sqrt,
            z=0.0,
            w=half_sqrt,
        )
    )

    result = resolve_reference_approach_direction(
        point,
        None,
        pose,
        mode=APPROACH_MODE_TAG_X,
    )

    assert result.source == APPROACH_SOURCE_TAG_X_SELECTED
    assert result.direction_camera.x == pytest.approx(0.0, abs=1e-9)
    assert result.direction_camera.y == pytest.approx(0.0, abs=1e-9)
    assert result.direction_camera.z == pytest.approx(-1.0)


def test_surface_fit_only_rejects_missing_normal():
    """Surface-only mode never silently falls back to tag orientation."""
    with pytest.raises(ValueError, match="Surface-fit approach"):
        resolve_reference_approach_direction(
            make_projected_point(),
            None,
            make_pose(),
            mode=APPROACH_MODE_SURFACE_FIT,
            surface_normal_unavailable_reason="too few samples",
        )


def test_surface_normal_must_belong_to_the_selected_point():
    """A stale normal cannot be attached to a different selected pixel."""
    point = make_projected_point()
    other = ProjectedReferencePoint(
        requested_pixel=ImagePoint(u=13, v=8),
        sampled_pixel=ImagePoint(u=13, v=8),
        point_camera=Vector3Data(x=0.11, y=-0.02, z=0.8),
        frame_id=point.frame_id,
        depth_m=0.8,
    )
    normal = make_surface_normal(other, 1.0, 0.0, 0.0)

    with pytest.raises(ValueError, match="different reference pixel"):
        resolve_reference_approach_direction(
            point,
            normal,
            make_pose(),
        )


def test_invalid_mode_is_rejected():
    """Unknown direction-source modes fail explicitly."""
    with pytest.raises(ValueError, match="Unsupported"):
        resolve_reference_approach_direction(
            make_projected_point(),
            None,
            make_pose(),
            mode="unknown",
        )
