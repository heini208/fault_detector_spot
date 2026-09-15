"""Offline geometry checks for NumPy surface orientation."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry import surface_plane
from fault_detector_spot.inspection.geometry.depth_point_cloud import (
    create_organized_depth_point_cloud,
)
from fault_detector_spot.inspection.geometry.surface_normal import (
    estimate_surface_normal,
)
from fault_detector_spot.inspection.sensing.probe_surface_source import (
    ProbeSurfaceSource,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    project_reference_pixel,
)
from fault_detector_spot.inspection.model.models import ImagePoint, Vector3Data
from test_surface_normal import make_camera_info, make_32fc1, plane_depth_values


def fit(points):
    return surface_plane.fit_surface_plane(
        points, 'camera', 0.005, 12, 0.60, 100, 0.0005,
    )


@pytest.mark.parametrize('seed', range(20))
def test_noisy_tilted_plane_with_outliers_matches_known_plane(seed):
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-0.05, 0.05, (797, 2))
    slopes = rng.uniform(-0.5, 0.5, 2)
    z = 0.6 + xy @ slopes + rng.normal(0, 0.0005, len(xy))
    points = np.column_stack((xy, z))
    # Thirty percent gross outliers, still within the depth-delta gate.
    bad = rng.choice(len(points), int(0.3 * len(points)), replace=False)
    points[bad, 2] += rng.choice([-1, 1], len(bad)) * 0.035
    expected = np.r_[-slopes, 1.0]
    expected /= np.linalg.norm(expected)
    result = fit(points)
    cosine = np.clip(abs(result.normal_array() @ expected), 0, 1)
    assert math.degrees(math.acos(cosine)) < 0.5
    assert result.inlier_ratio == pytest.approx(0.70, abs=0.01)
    assert result.rmse_m < 0.001
    assert abs(result.signed_distance(Vector3Data(0.0, 0.0, 0.6))) < 0.001


def test_degenerate_and_nonplanar_samples_rejected():
    x = np.linspace(-0.05, 0.05, 100)
    with pytest.raises(ValueError):
        fit(np.column_stack((x, x * 0, x * 0 + 0.6)))
    with pytest.raises(ValueError):
        fit(np.full((100, 3), 0.6))
    rng = np.random.default_rng(92)
    with pytest.raises(ValueError, match='not planar enough'):
        fit(rng.uniform(-0.05, 0.05, (797, 3)))


@pytest.mark.parametrize('encoding', ['32FC1', '16UC1'])
@pytest.mark.parametrize('bigendian', [False, True])
def test_projection_matches_pinhole_geometry_with_padding_and_invalid_depth(
    encoding, bigendian,
):
    camera = make_camera_info(11, 11)
    image = make_32fc1([0.6] * 121)
    values = np.full((11, 13), 600 if encoding == '16UC1' else 0.6)
    values[0, 0] = 0
    if encoding == '32FC1':
        values[0, 1:4] = [np.nan, np.inf, -1.0]
    dtype = ('>' if bigendian else '<') + ('u2' if encoding == '16UC1' else 'f4')
    image.encoding = encoding
    image.is_bigendian = bigendian
    image.step = 13 * np.dtype(dtype).itemsize
    image.data = values.astype(dtype).tobytes()
    cloud = create_organized_depth_point_cloud(image, camera)
    depth = values[:, :11].astype(dtype).astype(float)
    if encoding == '16UC1':
        depth *= 0.001
    valid = np.isfinite(depth) & (depth > 0)
    np.testing.assert_array_equal(cloud.valid_mask, valid)
    for v, u in np.argwhere(valid):
        z = depth[v, u]
        np.testing.assert_allclose(
            cloud.points_camera[v, u],
            [(u - 5) * z / 100, (v - 5) * z / 100, z],
            atol=1e-7,
        )
    assert np.isnan(cloud.points_camera[~valid]).all()


def test_surface_source_preserves_known_normal():
    camera = make_camera_info(33, 33, 300.0)
    expected = (0.2, -0.1, -math.sqrt(0.95))
    depth = make_32fc1(plane_depth_values(33, 33, camera, expected, 0.6), 33, 33)
    source = SimpleNamespace(latest_hand_depth=lambda _age: (depth, camera))
    result = ProbeSurfaceSource.surface_normal(source)
    np.testing.assert_allclose(
        [result.normal_camera.x, result.normal_camera.y, result.normal_camera.z],
        expected, atol=1e-5,
    )
    assert result.sample_count == 797


def test_depth_edge_and_missing_center_preserve_selection():
    camera = make_camera_info(33, 33, 300.0)
    values = np.full((33, 33), 0.6)
    values[:, 20:] = 1.2
    values[16, 16] = np.nan
    depth = make_32fc1(values.ravel(), 33, 33)
    cloud = create_organized_depth_point_cloud(depth, camera)
    projected = project_reference_pixel(
        ImagePoint(u=16, v=16), depth, camera,
        search_radius_px=16, point_cloud=cloud,
    )
    result = estimate_surface_normal(
        projected, depth, camera,
        neighborhood_radius_px=16,
        maximum_neighborhood_radius_px=16,
        point_cloud=cloud,
    )
    assert result.normal_camera.z == pytest.approx(-1, abs=1e-6)
    assert result.plane_rmse_m < 1e-7
