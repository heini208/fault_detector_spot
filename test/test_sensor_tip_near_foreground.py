"""Near gripper-like pixels may not replace a visible probe-axis target."""

import numpy as np
import pytest

from fault_detector_spot.inspection.sensing.live_surface_plane import estimate_live_surface_normal
from fault_detector_spot.inspection.sensing.sensor_tip_distance import measure_sensor_tip_distance
from fault_detector_spot.shared.geometry.models import PoseData
from test_sensor_tip_automatic_discovery import wall_image
from test_sensor_tip_observation import camera_pose, target_image


def depth_values(image):
    return np.frombuffer(bytes(image.data), dtype="<f4").reshape(
        image.height, image.width,
    ).copy()


@pytest.mark.parametrize("wall_depth", [0.270, 0.370])
@pytest.mark.parametrize("foreground_size", [1, 17])
@pytest.mark.parametrize("sparse", [False, True])
@pytest.mark.parametrize("height", [0.0, 0.03])
def test_off_axis_near_foreground_keeps_visible_wall_measurement(
    wall_depth, foreground_size, sparse, height,
):
    image, info = target_image(wall_depth)
    values = depth_values(image)
    if sparse:
        values[:] = 0.0
        values[::3, ::3] = wall_depth
    # At 178 mm, column 84 is 17.8 mm off the probe ray: inside the
    # discovery corridor but outside the target footprint at column 64.
    half = foreground_size // 2
    near_row = round(48 + 200 * height / 0.178)
    values[near_row-half:near_row+half+1, 84-half:85+half] = 0.178
    image.data = values.tobytes()

    orientation = estimate_live_surface_normal(image, info)
    sample = measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(height))

    assert orientation.projected_point.depth_m == pytest.approx(wall_depth)
    assert sample.distance_m == pytest.approx(wall_depth - 0.10, abs=1e-6)
    assert sample.valid_pixel_ratio == pytest.approx(1.0)


@pytest.mark.parametrize("height", [0.0, 0.03])
def test_near_foreground_on_probe_ray_still_blocks_with_camera_parallax(height):
    image, info = target_image(0.370)
    values = depth_values(image)
    # With parallax, the nearer intersection is at row 82 and the far
    # target at row 64. Checking only the final wall patch would miss it.
    near_row = round(48 + 200 * height / 0.178)
    values[near_row, 64] = 0.178
    image.data = values.tobytes()

    with pytest.raises(ValueError, match="too close.*0.178 m.*pixel"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(height))


@pytest.mark.parametrize("support", ["missing", "collinear", "foreground"])
def test_unsupported_off_axis_speck_does_not_require_unrelated_near_ray_support(support):
    image, info = target_image(0.370)
    values = depth_values(image)
    values[82, 84] = 0.178
    # The nearer ray footprint is separate from the visible wall target.
    if support == "foreground":
        values[80, 66] = 0.170
    else:
        values[79:86, 61:68] = 0.0
        if support == "collinear":
            values[82, 61:68] = 0.370
    image.data = values.tobytes()

    if support == "foreground":
        with pytest.raises(ValueError, match="too close"):
            measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(0.03))
    else:
        # Unknown support at a different camera ray is not an observed
        # obstruction. The wall contact itself still has enclosing depth.
        sample = measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(0.03))
        assert sample.distance_m == pytest.approx(0.270, abs=1e-6)


def test_near_surface_with_probe_intersection_outside_image_stays_blocking():
    camera = camera_pose(0.08)
    image, info = wall_image(camera, 0.38)
    values = depth_values(image)
    # A near surface at the bottom of the image could meet the probe ray
    # below the image. Visible farther depth cannot establish clearance.
    values[450:, :] = 0.178
    image.data = values.tobytes()

    with pytest.raises(ValueError, match="too close.*0.178 m"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera)


@pytest.mark.parametrize("height,near_depth,speck_depth", [(0.15, 0.320, 0.290), (0.08, 0.178, 0.158)])
def test_off_axis_speck_cannot_discard_a_coherent_nearer_plane(height, near_depth, speck_depth):
    camera = camera_pose(height)
    image, info = wall_image(camera, 0.38)
    values = depth_values(image)
    values[450:, 290:351] = near_depth
    values[479, 320] = speck_depth
    image.data = values.tobytes()

    with pytest.raises(ValueError, match="foreground of the fitted plane|too close|outside the camera image"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera)


@pytest.mark.parametrize("target_depth", [0.0, 0.178])
def test_ignoring_off_axis_foreground_does_not_fill_occluded_wall_target(target_depth):
    image, info = target_image(0.370)
    values = depth_values(image)
    values[82, 84] = 0.178
    # This foreground projects behind the near probe footprint, but covers
    # the farther target itself. The original image must still reject it.
    values[61:68, 61:68] = target_depth
    image.data = values.tobytes()

    with pytest.raises(ValueError, match="insufficient unoccluded|foreground depth closer"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(0.03))
