"""Discover and fit actual target support without an assumed wall distance."""

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
    # The reported region fits the measured intersection directly; it need
    # not include the unrelated pixel implied by an assumed distance.
    region = sample.source_region
    assert region.y <= intersection_v - 3
    assert region.y + region.height > intersection_v + 3
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


@pytest.mark.parametrize("distance", [0.379, 0.381])
def test_standalone_discovers_reported_distances_without_an_estimate(distance):
    image, info, camera = edge_target_scene()
    image.data = np.full((480, 640), distance + 0.10, dtype="<f4").tobytes()
    sample = measure_sensor_tip_distance(
        image, info, PoseData.identity(), camera,
    )
    assert sample.distance_m == pytest.approx(distance, abs=1e-6)
    u, v = project_camera_point(
        target_camera_point(PoseData.identity(), camera, distance),
        (552, 552, 320, 240),
    )
    region = sample.source_region
    assert region.x <= u - 3 < u + 3 < region.x + region.width
    assert region.y <= v - 3 < v + 3 < region.y + region.height
    # The final normal is fitted at the actual intersection, over 50 pixels
    # from where a 250 mm assumption would have put the target.
    assert region.y + region.height < 443 - 3
    assert region.height <= 65
    with pytest.raises(ValueError, match="does not match the intended"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera, 0.25)


@pytest.mark.parametrize("depth", [0.0, 0.12, 0.40, 0.55])
def test_discovery_cannot_skip_missing_occluded_or_different_actual_surface(depth):
    image, info, camera = edge_target_scene()
    values = np.full((480, 640), 0.48, dtype="<f4")
    u, v = project_camera_point(
        target_camera_point(PoseData.identity(), camera, 0.38),
        (552, 552, 320, 240),
    )
    u, v = round(u), round(v)
    values[v-3:v+4, u-3:u+4] = depth
    image.data = values.tobytes()
    with pytest.raises(ValueError, match="insufficient unoccluded|foreground depth closer|two-dimensional surface"):
        measure_sensor_tip_distance(
            image, info, PoseData.identity(), camera,
        )


def test_discovery_keeps_camera_minimum_and_probe_maximum_distance_limits():
    image, info, camera = edge_target_scene()
    for optical_depth, reason in [(0.28, "too close"), (0.61, "0.003–0.500 m range")]:
        image.data = np.full((480, 640), optical_depth, dtype="<f4").tobytes()
        with pytest.raises(ValueError, match=reason):
            measure_sensor_tip_distance(
                image, info, PoseData.identity(), camera,
            )
