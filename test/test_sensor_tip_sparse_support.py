"""Sparse registered pixels must support the target without hiding obstacles."""

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry.depth_point_cloud import (
    create_organized_depth_point_cloud,
)
from fault_detector_spot.inspection.geometry.surface_plane import SurfacePlane
from fault_detector_spot.inspection.model.models import ImagePoint
from fault_detector_spot.inspection.sensing.sensor_tip_distance import (
    _target_patch,
    measure_sensor_tip_distance,
)
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from test_live_surface_distance import planar_depth
from test_sensor_tip_distance_support import edge_target_scene
from test_sensor_tip_observation import camera_pose, target_image


def sparse_image(image, row_phase=0, column_phase=0):
    source = np.frombuffer(bytes(image.data), dtype="<f4").reshape(
        image.height, image.width,
    )
    values = np.zeros_like(source)
    values[row_phase::3, column_phase::3] = source[row_phase::3, column_phase::3]
    image.data = values.tobytes()
    return values


def local_cloud(samples):
    image, info = planar_depth(width=11, height=11, depth_m=0.35)
    values = np.zeros((11, 11), dtype="<f4")
    for u, v, depth in samples:
        values[v, u] = depth
    image.data = values.tobytes()
    return create_organized_depth_point_cloud(image, info)


def camera_plane():
    return SurfacePlane(
        point=Vector3Data(0.0, 0.0, 0.35),
        normal=Vector3Data(0.0, 0.0, -1.0),
        frame_id="hand_depth",
        inlier_count=30,
        sample_count=30,
        inlier_ratio=1.0,
        rmse_m=0.0,
    )


@pytest.mark.parametrize("row_phase", range(3))
@pytest.mark.parametrize("column_phase", range(3))
def test_sparse_registered_wall_measures_across_raster_phases(row_phase, column_phase):
    image, info = target_image()
    # The geometric target lies between integer pixels, as in real TF projection.
    info.k[2] += 0.4
    info.k[5] -= 0.4
    sparse_image(image, row_phase, column_phase)
    sample = measure_sensor_tip_distance(
        image, info, PoseData.identity(), camera_pose(), 0.25,
    )
    assert sample.distance_m == pytest.approx(0.25, abs=1e-6)
    assert sample.sample_count >= 30
    assert sample.valid_pixel_ratio == pytest.approx(1.0)


@pytest.mark.parametrize("pixels", [
    [(3, 3), (7, 3), (3, 7)],
    [(2, 5), (4, 5), (6, 5), (8, 5)],
    [(2, 2), (4, 4), (6, 6), (8, 8)],
    [(6, 3), (8, 3), (6, 7), (8, 7)],
])
def test_sparse_returns_require_enough_two_dimensional_support_around_target(pixels):
    cloud = local_cloud([(u, v, 0.35) for u, v in pixels])
    with pytest.raises(ValueError):
        _target_patch(cloud, ImagePoint(5, 5))


def test_hull_checks_exact_projected_target_instead_of_rounded_pixel():
    cloud = local_cloud([(u, v, 0.35) for u, v in [(5, 3), (7, 3), (5, 7), (7, 7)]])
    with pytest.raises(ValueError):
        _target_patch(cloud, ImagePoint(5, 5), projected_pixel=(4.6, 5.0))


def test_one_foreground_return_rejects_otherwise_supported_wall():
    wall = [(u, v, 0.35) for u in (3, 5, 7) for v in (3, 5, 7)]
    wall[4] = (5, 5, 0.30)
    cloud = local_cloud(wall)
    # Eight of nine returns support the wall; an occluder must still reject it.
    assert _target_patch(cloud, ImagePoint(5, 5)) == pytest.approx(1.0)
    with pytest.raises(ValueError):
        _target_patch(cloud, ImagePoint(5, 5), camera_plane=camera_plane())


def test_plane_consistent_returns_must_surround_target_themselves():
    cloud = local_cloud([
        (6, 3, 0.35), (8, 3, 0.35), (6, 7, 0.35), (8, 7, 0.35),
        (2, 3, 0.40), (2, 7, 0.40),
    ])
    assert _target_patch(cloud, ImagePoint(5, 5)) == pytest.approx(1.0)
    # The four wall returns pass the 60% ratio but all lie right of the target.
    with pytest.raises(ValueError):
        _target_patch(cloud, ImagePoint(5, 5), camera_plane=camera_plane())


def test_four_local_returns_do_not_replace_required_larger_plane_support():
    image, info = target_image()
    values = np.zeros((image.height, image.width), dtype="<f4")
    for u, v in [(62, 46), (66, 46), (62, 50), (66, 50)]:
        values[v, u] = 0.35
    image.data = values.tobytes()
    with pytest.raises(ValueError, match="Too few consistent depth samples"):
        measure_sensor_tip_distance(
            image, info, PoseData.identity(), camera_pose(), 0.25,
        )


def test_sparse_nominal_support_cannot_replace_missing_actual_intersection():
    image, info, camera = edge_target_scene()
    values = sparse_image(image)
    # Actual intersection at row422 differs from the nominal target at row443.
    values[419:426, 317:324] = 0.0
    image.data = values.tobytes()
    with pytest.raises(ValueError, match="pixel \\(320, 422\\)"):
        measure_sensor_tip_distance(
            image, info, PoseData.identity(), camera, 0.25,
        )
