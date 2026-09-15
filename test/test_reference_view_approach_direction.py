"""Tests for selecting the outward vector used by probe setup geometry."""

import math

import pytest

from fault_detector_spot.inspection.geometry.surface_normal import (
    SurfaceNormalEstimate,
)
from fault_detector_spot.inspection.model.models import (
    ImagePoint,
    PoseData,
    QuaternionData,
    Vector3Data,
)
from fault_detector_spot.inspection.setup.probe_setup_geometry import (
    ProbeSetupGeometry,
)
from fault_detector_spot.inspection.setup.reference_view_approach_direction import (
    APPROACH_MODE_AUTOMATIC,
    APPROACH_MODE_SURFACE_FIT,
    APPROACH_MODE_TAG_X,
    APPROACH_SOURCE_SURFACE_FIT,
    APPROACH_SOURCE_TAG_X_SELECTED,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ProjectedReferencePoint,
)


def make_projected_point():
    pixel = ImagePoint(u=12, v=8)
    return ProjectedReferencePoint(
        requested_pixel=pixel,
        sampled_pixel=ImagePoint(u=12, v=8),
        point_camera=Vector3Data(x=0.1, y=-0.02, z=0.8),
        frame_id="hand_color_image_sensor",
        depth_m=0.8,
    )


def make_pose(orientation=None):
    return PoseData(
        position=Vector3Data.zero(),
        orientation=orientation or QuaternionData.identity(),
    )


def make_surface_normal(projected_point, x, y, z):
    return SurfaceNormalEstimate(
        projected_point=projected_point,
        normal_camera=Vector3Data(x=x, y=y, z=z),
        sample_count=40,
        plane_rmse_m=0.001,
    )


def resolve(point, normal, pose, mode, reason=""):
    return ProbeSetupGeometry._resolve_outward_camera_direction(
        projected_point=point,
        surface_normal=normal,
        controlled_frame_pose_object=pose,
        mode=mode,
        surface_normal_unavailable_reason=reason,
    )


def test_surface_fit_uses_the_existing_camera_facing_surface_normal():
    point = make_projected_point()
    normal = make_surface_normal(point, 0.0, 0.0, -1.0)

    direction, source = resolve(
        point,
        normal,
        make_pose(),
        APPROACH_MODE_AUTOMATIC,
    )

    assert source == APPROACH_SOURCE_SURFACE_FIT
    assert direction.x == pytest.approx(0.0)
    assert direction.y == pytest.approx(0.0)
    assert direction.z == pytest.approx(-1.0)


def test_surface_fit_does_not_reorient_the_surface_normal_again():
    point = make_projected_point()
    normal = make_surface_normal(point, 0.25, 0.0, -0.75)

    direction, _source = resolve(
        point,
        normal,
        make_pose(),
        APPROACH_MODE_SURFACE_FIT,
    )

    length = math.sqrt(0.25 ** 2 + 0.75 ** 2)
    assert direction.x == pytest.approx(0.25 / length)
    assert direction.z == pytest.approx(-0.75 / length)


def test_automatic_mode_rejects_missing_surface_fit():
    with pytest.raises(ValueError, match="surface is uneven"):
        resolve(
            make_projected_point(),
            None,
            make_pose(),
            APPROACH_MODE_AUTOMATIC,
            "surface is uneven",
        )


def test_tag_x_is_transformed_into_the_camera_frame():
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

    direction, source = resolve(
        point,
        None,
        pose,
        APPROACH_MODE_TAG_X,
    )

    assert source == APPROACH_SOURCE_TAG_X_SELECTED
    assert direction.x == pytest.approx(0.0, abs=1e-9)
    assert direction.y == pytest.approx(0.0, abs=1e-9)
    assert direction.z == pytest.approx(-1.0)


def test_surface_normal_must_belong_to_the_selected_point():
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
        resolve(
            point,
            normal,
            make_pose(),
            APPROACH_MODE_AUTOMATIC,
        )


def test_invalid_mode_is_rejected():
    with pytest.raises(ValueError, match="Unsupported"):
        resolve(
            make_projected_point(),
            None,
            make_pose(),
            "unknown",
        )
