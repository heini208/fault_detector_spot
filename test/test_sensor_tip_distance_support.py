"""Fit actual target support when a short observation move leaves parallax."""

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry.sensor_tip_observation import (
    project_camera_point,
    target_camera_point,
)
from fault_detector_spot.inspection.sensing.sensor_tip_distance import (
    measure_sensor_tip_distance,
)
from fault_detector_spot.shared.geometry.models import PoseData
from test_live_surface_distance import planar_depth
from test_sensor_tip_observation import camera_pose


def edge_target_scene():
    image, info = planar_depth(width=640, height=480, depth_m=0.39)
    info.k[0] = info.k[4] = 552.0
    # Nominal 250 mm stand-off appears 36 px inside the bottom image edge.
    # The actual 290 mm surface shifts the original probe axis by 20.8 px.
    camera = camera_pose(height=(443.0 - 240.0) * 0.35 / 552.0)
    return image, info, camera


def test_off_center_observation_fits_depth_through_actual_probe_intersection():
    image, info, camera = edge_target_scene()
    sample = measure_sensor_tip_distance(
        image, info, PoseData.identity(), camera, 0.25,
    )
    assert sample.distance_m == pytest.approx(0.29, abs=1e-6)
    assert sample.surface_plane_probe.normal.x == pytest.approx(-1.0)
    assert sample.valid_pixel_ratio == pytest.approx(1.0)
    _, intersection_v = project_camera_point(
        target_camera_point(PoseData.identity(), camera, 0.29),
        (552.0, 552.0, 320.0, 240.0),
    )
    # The reported fitted region includes both the expected and measured
    # target patches, rather than extrapolating a fit that stops before it.
    region = sample.source_region
    assert region.y <= intersection_v - 3
    assert region.y + region.height > 443 + 3
    assert region.height <= 65


@pytest.mark.parametrize("target_depth,valid_count", [(0.0, 0), (0.12, 49)])
def test_expanding_fit_still_rejects_missing_actual_probe_target_depth(target_depth, valid_count):
    image, info, camera = edge_target_scene()
    depth = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    # The nominal target at row443 remains visible; its measured intersection
    # at row422 has no depth. Surrounding wall depth cannot replace this patch.
    depth[419:426, 317:324] = target_depth
    image.data = depth.tobytes()
    reason = "insufficient unoccluded depth support" if target_depth == 0 else "foreground depth closer"
    with pytest.raises(ValueError, match=reason) as error:
        measure_sensor_tip_distance(
            image, info, PoseData.identity(), camera, 0.25,
        )
    assert "pixel (320, 422)" in str(error.value)
    assert f"valid depth={valid_count}/49" in str(error.value)
    assert ("need at least 4 measured samples" if target_depth == 0 else "foreground depth closer") in str(error.value)
