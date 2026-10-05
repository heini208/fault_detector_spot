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
from fault_detector_spot.inspection.model.models import (
    ImagePoint,
)
from fault_detector_spot.shared.geometry.models import (
    Vector3Data,
)
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
    assert result.sample_count >= 797


def test_surface_source_accepts_noisy_planar_depth():
    camera = make_camera_info(65, 65, 100.0)
    expected = (0.2, -0.1, -math.sqrt(0.95))
    values = np.array(plane_depth_values(65, 65, camera, expected, 0.6))
    values += np.random.default_rng(7).normal(0.0, 0.009, len(values))
    depth = make_32fc1(values, 65, 65)
    source = SimpleNamespace(latest_hand_depth=lambda _age: (depth, camera))
    result = ProbeSurfaceSource.surface_normal(source)
    normal = np.array([result.normal_camera.x, result.normal_camera.y,
                       result.normal_camera.z])
    assert math.degrees(math.acos(np.clip(normal @ expected, -1, 1))) < 1.0
    assert result.plane_rmse_m < 0.015


def test_latest_hand_depth_can_require_frame_received_after_operation_start(monkeypatch):
    from collections import deque
    from threading import RLock
    from fault_detector_spot.inspection.sensing import probe_surface_source

    source = ProbeSurfaceSource.__new__(ProbeSurfaceSource)
    source._lock = RLock()
    source._hand_depth_camera_info = make_camera_info()
    old = make_32fc1([0.5] * 121)
    fresh = make_32fc1([0.7] * 121)
    source._hand_depth_history = deque([(1.0, old), (2.0, fresh)])
    source.node = SimpleNamespace(count_publishers=lambda topic: 1)
    monkeypatch.setattr(probe_surface_source.time, 'monotonic', lambda: 2.1)

    image, _ = source.latest_hand_depth(
        receipt_not_before=1.5,
    )
    assert image.data == fresh.data

    with pytest.raises(ValueError, match="after the required start time"):
        source.latest_hand_depth(receipt_not_before=2.5)


@pytest.mark.parametrize('publishers', [0, 1])
def test_stale_depth_is_rejected_with_stream_diagnostics(monkeypatch, publishers):
    from collections import deque
    from threading import RLock
    from fault_detector_spot.inspection.sensing import probe_surface_source
    source = ProbeSurfaceSource.__new__(ProbeSurfaceSource)
    source._lock = RLock()
    source._hand_depth_camera_info = make_camera_info()
    source._hand_depth_history = deque([(1.0, make_32fc1([0.6] * 121))])
    source.node = SimpleNamespace(count_publishers=lambda topic: publishers)
    monkeypatch.setattr(probe_surface_source.time, 'monotonic', lambda: 9.0)
    with pytest.raises(ValueError, match=f'discovered publishers={publishers}'):
        source.latest_hand_depth()


def test_surface_source_ignores_five_isolated_center_outliers():
    camera = make_camera_info(81, 81, 300.0)
    values = np.full((81, 81), 0.6)
    for v, u in ((40, 40), (39, 40), (41, 40), (40, 39), (40, 41)):
        values[v, u] = 0.8
    depth = make_32fc1(values.ravel(), 81, 81)
    depth.header.stamp.sec = 12
    depth.header.stamp.nanosec = 34
    source = SimpleNamespace(latest_hand_depth=lambda _age: (depth, camera))
    result = ProbeSurfaceSource.surface_normal(source)
    assert result.sample_count > 700
    assert result.normal_camera.z == pytest.approx(-1.0, abs=1e-6)
    assert result.stamp_nanoseconds == 12000000034


def test_surface_source_rejects_two_equally_supported_depth_layers():
    camera = make_camera_info(81, 81, 300.0)
    values = np.full((81, 81), 0.6)
    values[:, 41:] = 0.8
    values[:, 40] = np.nan
    depth = make_32fc1(values.ravel(), 81, 81)
    source = SimpleNamespace(latest_hand_depth=lambda _age: (depth, camera))
    with pytest.raises(ValueError, match="Ambiguous"):
        ProbeSurfaceSource.surface_normal(source)


def test_surface_source_centers_fit_on_nearest_valid_surface_sample():
    width = height = 41
    camera = make_camera_info(width, height, 100.0)
    values = np.full((height, width), np.nan)
    values[31:37, 31:37] = 0.6
    depth = make_32fc1(values.ravel(), width, height)
    source = SimpleNamespace(latest_hand_depth=lambda _age: (depth, camera))

    result = ProbeSurfaceSource.surface_normal(source)

    assert result.projected_point.mapped_pixel == ImagePoint(u=20, v=20)
    assert result.projected_point.sampled_pixel == ImagePoint(u=31, v=31)
    assert result.sample_count == 36
    assert result.normal_camera.z == pytest.approx(-1.0, abs=1e-6)


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
