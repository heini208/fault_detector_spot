"""Side surfaces cannot replace a measured target along the probe ray."""

import numpy as np
import pytest

from fault_detector_spot.inspection.sensing.sensor_tip_distance import measure_sensor_tip_distance
from fault_detector_spot.shared.geometry.models import PoseData
from test_sensor_tip_automatic_discovery import wall_image
from test_sensor_tip_observation import camera_pose


def wall_with_parallel_side_plane(side=1):
    camera = camera_pose(0.0)
    image, info = wall_image(camera, 0.30)
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    # A coherent side plane is 10 mm from the probe ray and inside its
    # broad 25 mm discovery corridor, but never intersects that ray.
    for offset in range(16, 26):
        values[220:261, 320 + side * offset] = info.k[0] * 0.010 / offset
    image.data = values.tobytes()
    return image, info, camera


@pytest.mark.parametrize("side", [-1, 1])
def test_parallel_off_axis_plane_does_not_hide_supported_wall_target(side):
    image, info, camera = wall_with_parallel_side_plane(side)

    sample = measure_sensor_tip_distance(image, info, PoseData.identity(), camera)

    assert sample.distance_m == pytest.approx(0.30, abs=1e-6)
    assert sample.surface_plane_probe.normal_array() == pytest.approx([-1, 0, 0], abs=1e-6)


def test_parallel_side_plane_does_not_remove_actual_probe_ray_foreground():
    image, info, camera = wall_with_parallel_side_plane()
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    values[240, 320] = 0.178
    image.data = values.tobytes()

    with pytest.raises(ValueError, match="too close.*0.178 m"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera)


def test_parallel_side_plane_continuation_preserves_expected_target_correspondence():
    image, info, camera = wall_with_parallel_side_plane()

    with pytest.raises(ValueError, match="does not match the intended sensor-tip target"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera, 0.10)
