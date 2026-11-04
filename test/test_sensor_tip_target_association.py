"""An observation retains its measured wall without hiding nearer geometry."""

import numpy as np
import pytest

from fault_detector_spot.inspection.sensing import sensor_tip_distance as sensing
from fault_detector_spot.shared.geometry.models import PoseData
from test_sensor_tip_automatic_discovery import wall_image
from test_sensor_tip_observation import camera_pose
from test_sensor_tip_side_planes import wall_with_parallel_side_plane


def observed_plane(camera, distance=0.30):
    image, info = wall_image(camera, distance)
    return sensing.measure_sensor_tip_distance(
        image, info, PoseData.identity(), camera,
    ).surface_plane_probe


@pytest.mark.parametrize("side", [-1, 1])
def test_observed_wall_remains_associated_through_off_axis_side_geometry(side):
    image, info, camera = wall_with_parallel_side_plane(side)
    plane = observed_plane(camera)

    sample = sensing.measure_sensor_tip_distance(
        image, info, PoseData.identity(), camera, target_plane_probe=plane,
    )

    assert sample.distance_m == pytest.approx(0.30, abs=1e-6)
    assert sample.surface_plane_probe.normal_array() == pytest.approx([-1, 0, 0], abs=1e-6)


def test_observed_wall_is_not_replaced_by_unsupported_forward_side_plane():
    camera = camera_pose(0.03)
    image, info = wall_image(camera, 0.30)
    plane = observed_plane(camera)
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    # This coherent grazing plane lies beside the probe and extrapolates to
    # a 160 mm ray contact. The real wall is visible at that image location,
    # so no measured depth supports that hypothetical side-plane contact.
    for column in range(324, 329):
        values[300:341, column] = 0.026 / ((column-320)/552 + 0.10)
    image.data = values.tobytes()

    with pytest.raises(ValueError, match="insufficient unoccluded depth support"):
        sensing.measure_sensor_tip_distance(image, info, PoseData.identity(), camera)

    sample = sensing.measure_sensor_tip_distance(
        image, info, PoseData.identity(), camera, target_plane_probe=plane,
    )

    assert sample.distance_m == pytest.approx(0.30, abs=1e-6)
    assert sample.surface_plane_probe.normal_array() == pytest.approx([-1, 0, 0], abs=1e-6)


@pytest.mark.parametrize("height,near_depth", [(0.0, 0.178), (0.03, 0.250)])
def test_observed_wall_does_not_hide_actual_nearer_probe_ray_returns(height, near_depth):
    camera = camera_pose(height)
    image, info = wall_image(camera, 0.30)
    plane = observed_plane(camera)
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    near_row = round(240 + 552 * height / near_depth)
    values[near_row, 320] = near_depth
    image.data = values.tobytes()

    with pytest.raises(ValueError, match="foreground lies on probe"):
        sensing.measure_sensor_tip_distance(
            image, info, PoseData.identity(), camera, target_plane_probe=plane,
        )


def test_observed_wall_does_not_hide_coherent_nearer_surface_outside_image():
    camera = camera_pose(0.15)
    image, info = wall_image(camera, 0.38)
    plane = observed_plane(camera, 0.38)
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    values[450:, :] = 0.32
    image.data = values.tobytes()

    with pytest.raises(sensing.SensorTipTargetOutsideView) as error:
        sensing.measure_sensor_tip_distance(
            image, info, PoseData.identity(), camera, target_plane_probe=plane,
        )

    assert error.value.axis_distance_m == pytest.approx(0.22, abs=1e-6)
    assert error.value.target_plane_probe.normal.x == pytest.approx(-1.0, abs=1e-6)


def test_observed_wall_does_not_bypass_coherent_camera_minimum_blocker():
    camera = camera_pose(0.08)
    image, info = wall_image(camera, 0.38)
    plane = observed_plane(camera, 0.38)
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    values[450:, :] = 0.178
    image.data = values.tobytes()

    with pytest.raises(ValueError, match="too close.*0.178 m.*0.210 m"):
        sensing.measure_sensor_tip_distance(
            image, info, PoseData.identity(), camera, target_plane_probe=plane,
        )


def test_observed_wall_does_not_accept_a_different_local_surface_normal():
    camera = camera_pose()
    plane = observed_plane(camera)
    image, info = wall_image(camera, 0.30, normal=(-1.0, 0.5, 0.0))

    with pytest.raises(ValueError, match="normal does not agree.*26.6 deg"):
        sensing.measure_sensor_tip_distance(
            image, info, PoseData.identity(), camera, target_plane_probe=plane,
        )


def test_observed_wall_does_not_relocate_target_to_another_plane():
    camera = camera_pose()
    plane = observed_plane(camera)
    image, info = wall_image(camera, 0.33)

    with pytest.raises(ValueError, match="surface does not agree.*0.330 m.*0.300 m"):
        sensing.measure_sensor_tip_distance(
            image, info, PoseData.identity(), camera, target_plane_probe=plane,
        )


def test_observed_wall_still_requires_actual_target_depth():
    camera = camera_pose()
    plane = observed_plane(camera)
    image, info = wall_image(camera, 0.30)
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(480, 640).copy()
    values[237:244, 317:324] = 0.0
    image.data = values.tobytes()

    with pytest.raises(ValueError, match="insufficient unoccluded depth support"):
        sensing.measure_sensor_tip_distance(
            image, info, PoseData.identity(), camera, target_plane_probe=plane,
        )


def test_failed_observed_target_fit_cannot_fall_back_to_another_seed(monkeypatch):
    camera = camera_pose()
    plane = observed_plane(camera)
    image, info = wall_image(camera, 0.30)
    values = np.zeros((480, 640), dtype="<f4")
    values[[237, 240, 240, 243], [320, 317, 323, 320]] = 0.40
    image.data = values.tobytes()

    def forbidden_fallback(*args, **kwargs):
        pytest.fail("An observed target fit must not select a different seed")

    monkeypatch.setattr(sensing, "_next_axis_depth_seed", forbidden_fallback)
    with pytest.raises(ValueError, match="Too few consistent depth samples"):
        sensing.measure_sensor_tip_distance(
            image, info, PoseData.identity(), camera, target_plane_probe=plane,
        )
