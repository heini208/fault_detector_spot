"""Offline quality comparisons for the NumPy orientation backend."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry import open3d_depth, surface_plane
from fault_detector_spot.inspection.geometry.open3d_depth import create_organized_depth_point_cloud
from fault_detector_spot.inspection.geometry.surface_normal import estimate_surface_normal
from fault_detector_spot.inspection.sensing.probe_surface_source import ProbeSurfaceSource
from fault_detector_spot.inspection.setup.reference_view_depth_projection import project_reference_pixel
from fault_detector_spot.inspection.model.models import ImagePoint
from test_surface_normal import make_camera_info, make_32fc1, plane_depth_values


def fit(points, use_open3d):
    return surface_plane.fit_surface_plane(
        points, 'camera', 0.005, 12, 0.60, 100, 0.0005,
        use_open3d=use_open3d,
    )


@pytest.mark.parametrize('seed', range(20))
def test_noisy_tilted_plane_with_outliers_matches_open3d(seed):
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
    old, new = fit(points, True), fit(points, False)
    def angle(normal):
        return math.degrees(math.acos(np.clip(abs(normal @ expected), 0, 1)))
    assert angle(new.normal_array()) < 0.5
    assert angle(new.normal_array()) <= angle(old.normal_array()) + 0.1
    assert new.inlier_ratio == pytest.approx(old.inlier_ratio, abs=0.01)
    assert new.rmse_m <= old.rmse_m + 0.0001


@pytest.mark.parametrize('use_open3d', [True, False])
def test_degenerate_and_nonplanar_samples_rejected(use_open3d):
    x = np.linspace(-0.05, 0.05, 100)
    with pytest.raises(ValueError):
        fit(np.column_stack((x, x * 0, x * 0 + 0.6)), use_open3d)
    with pytest.raises(ValueError):
        fit(np.full((100, 3), 0.6), use_open3d)
    rng = np.random.default_rng(92)
    with pytest.raises(ValueError, match='not planar enough'):
        fit(rng.uniform(-0.05, 0.05, (797, 3)), use_open3d)


@pytest.mark.parametrize('encoding', ['32FC1', '16UC1'])
@pytest.mark.parametrize('bigendian', [False, True])
def test_projection_matches_open3d_with_padding_and_invalid_depth(encoding, bigendian):
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
    old = create_organized_depth_point_cloud(image, camera)
    new = create_organized_depth_point_cloud(image, camera, use_open3d=False)
    np.testing.assert_array_equal(new.valid_mask, old.valid_mask)
    np.testing.assert_allclose(new.points_camera, old.points_camera, atol=1e-7)


def test_orientation_never_loads_open3d_and_preserves_normal(monkeypatch):
    def forbidden():
        raise AssertionError('Orientation must not initialize Open3D')
    monkeypatch.setattr(open3d_depth, 'require_open3d', forbidden)
    monkeypatch.setattr(surface_plane, 'require_open3d', forbidden)
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


@pytest.mark.parametrize('use_open3d', [True, False])
def test_depth_edge_and_missing_center_preserve_selection(use_open3d):
    camera = make_camera_info(33, 33, 300.0)
    values = np.full((33, 33), 0.6)
    values[:, 20:] = 1.2
    values[16, 16] = np.nan
    depth = make_32fc1(values.ravel(), 33, 33)
    cloud = create_organized_depth_point_cloud(depth, camera, use_open3d=use_open3d)
    projected = project_reference_pixel(ImagePoint(u=16, v=16), depth, camera,
                                        search_radius_px=16, point_cloud=cloud)
    result = estimate_surface_normal(projected, depth, camera,
                                    neighborhood_radius_px=16,
                                    maximum_neighborhood_radius_px=16,
                                    point_cloud=cloud, use_open3d=use_open3d)
    assert result.normal_camera.z == pytest.approx(-1, abs=1e-6)
    assert result.plane_rmse_m < 1e-7
