"""Saved-image candidates retain the selected ray and local measured surface."""

from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from test_probe_reference_preview import FakeRepository, image
from test_reference_view_rgb_depth_mapping import make_camera_info, make_depth

from fault_detector_spot.inspection.geometry.surface_plane import SurfacePlane
from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.setup.probe_setup_geometry import ProbeSetupGeometry
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ProjectedReferencePoint,
    project_reference_pixel,
    refine_reference_surface_point,
)
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data


def capture_with_surface(*, slope_x=0.0, slope_y=0.0):
    depth_info = make_camera_info(41, 41, 300.0)
    rgb_info = make_camera_info(82, 82, 600.0)
    v, u = np.indices((41, 41))
    depths = 1.0 / (1.0 - slope_x * (u - 20.0) / 300.0
                    - slope_y * (v - 20.0) / 300.0)
    return SimpleNamespace(
        reference_view=SimpleNamespace(
            view_id="slot1_hand", controlled_frame="hand_color_image_sensor",
            controlled_frame_pose_object=PoseData.identity(),
        ),
        rgb_image=image(82, 82), depth_image=make_depth(depths.ravel(), 41, 41),
        rgb_camera_info=rgb_info, depth_camera_info=depth_info,
    ), depths


def resolve(capture, pixel=ImagePoint(u=59, v=45), mode="surface_fit"):
    return ProbeSetupGeometry(FakeRepository(capture)).resolve(
        "motor", "scan", "slot1_hand", pixel, mode, 0.01, 0.25,
        PoseData.identity(),
    )


def raw_projection(capture, pixel):
    return project_reference_pixel(
        pixel, capture.depth_image, capture.depth_camera_info,
        rgb_size=(82, 82), rgb_camera_info=capture.rgb_camera_info,
    )


@pytest.mark.parametrize("depth_noise_m", [-0.010, 0.010])
def test_candidate_uses_fitted_surface_instead_of_one_noisy_depth(depth_noise_m):
    capture, depths = capture_with_surface()
    pixel = ImagePoint(u=59, v=45)
    mapped = raw_projection(capture, pixel).mapped_pixel
    depths[mapped.v, mapped.u] += depth_noise_m
    capture.depth_image = make_depth(depths.ravel(), 41, 41)
    raw = raw_projection(capture, pixel)

    result = resolve(capture, pixel)

    # The fitted position uses all accepted neighbors, while diagnostics still
    # describe the actual measurement and pixel used to seed that fit.
    assert result.projected_point == raw
    assert result.surface_normal.projected_point == raw
    assert raw.depth_m == pytest.approx(1.0 + depth_noise_m)
    surface = result.surface_target.surface_point_object
    assert surface.z == pytest.approx(1.0, abs=0.001)
    assert surface.x / surface.z == pytest.approx((pixel.u - 40.5) / 600.0)
    assert surface.y / surface.z == pytest.approx((pixel.v - 40.5) / 600.0)


def test_depth_hole_keeps_original_rgb_ray_on_sloped_surface():
    slope_x, slope_y = 0.3, -0.2
    capture, depths = capture_with_surface(slope_x=slope_x, slope_y=slope_y)
    pixel = ImagePoint(u=59, v=45)
    mapped = raw_projection(capture, pixel).mapped_pixel
    depths[mapped.v, mapped.u] = 0.0
    capture.depth_image = make_depth(depths.ravel(), 41, 41)
    saved = deepcopy(capture)
    camera_pose = capture.reference_view.controlled_frame_pose_object
    camera_pose.position = Vector3Data(0.3, -0.2, 0.4)
    camera_pose.orientation.z = camera_pose.orientation.w = 2 ** -0.5

    result = resolve(capture, pixel)

    assert result.projected_point.sampled_pixel != mapped
    ray = np.array([(pixel.u - 40.5) / 600.0, (pixel.v - 40.5) / 600.0, 1.0])
    expected_camera = ray / (1.0 - slope_x * ray[0] - slope_y * ray[1])
    expected_object = [0.3 - expected_camera[1], -0.2 + expected_camera[0],
                       0.4 + expected_camera[2]]
    surface = result.surface_target.surface_point_object
    assert [surface.x, surface.y, surface.z] == pytest.approx(expected_object, abs=1e-6)
    assert capture.depth_image == saved.depth_image
    assert capture.rgb_camera_info == saved.rgb_camera_info


@pytest.mark.parametrize("feature_depth", [0.96, 1.04])
def test_candidate_does_not_erase_feature_with_dominant_neighbor_plane(feature_depth):
    capture, depths = capture_with_surface()
    pixel = ImagePoint(u=59, v=45)
    mapped = raw_projection(capture, pixel).mapped_pixel
    depths[mapped.v - 1:mapped.v + 2, mapped.u - 1:mapped.u + 2] = feature_depth
    capture.depth_image = make_depth(depths.ravel(), 41, 41)

    with pytest.raises(ValueError, match="disagrees with the fitted surface"):
        resolve(capture, pixel)


def plane(point=(0.0, 0.0, 1.0), normal=(0.0, 0.0, -1.0)):
    return SurfacePlane(
        Vector3Data(*point), Vector3Data(*normal), "hand_color_image_sensor",
        40, 49, 40 / 49, 0.001,
    )


@pytest.mark.parametrize("surface,reason", [
    (plane(point=(0.0, 0.0, 0.90)), "too far"),
    (plane(point=(0.0, 0.0, -1.0)), "behind"),
    (plane(point=(0.0, 0.0, 1.0), normal=(1.0, 0.0, 0.0)), "parallel"),
])
def test_unreliable_plane_intersections_are_rejected(surface, reason):
    pixel = ImagePoint(u=20, v=20)
    projected = ProjectedReferencePoint(
        pixel, pixel, Vector3Data(0.0, 0.0, 1.0),
        "hand_color_image_sensor", 1.0,
    )

    with pytest.raises(ValueError, match=reason):
        refine_reference_surface_point(
            projected, surface, make_camera_info(41, 41, 300.0), (41, 41),
        )


def test_surface_fit_requires_local_support_and_tag_axis_retains_raw_fallback():
    capture, depths = capture_with_surface()
    depths[:] = 0.0
    depths[22, 29] = 1.0
    capture.depth_image = make_depth(depths.ravel(), 41, 41)

    with pytest.raises(ValueError, match="Too few consistent depth samples"):
        resolve(capture)
    result = resolve(capture, mode="tag_x")
    assert result.surface_normal is None
    assert result.surface_target.surface_point_object == result.projected_point.point_camera
