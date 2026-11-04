"""Observation uses calibrated probe geometry before measuring any surface."""

from copy import deepcopy

import numpy as np
import pytest

from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from test_sensor_tip_automatic_discovery import wall_image
import test_sensor_tip_observation as observation_fixtures
from test_sensor_tip_observation import camera_pose


source = observation_fixtures.source


def set_image(source, image, info, camera):
    source.source._receive_hand_depth_camera_info(info)
    source.source._receive_hand_depth(image)
    source.source._lookup_pose = lambda *args, **kwargs: deepcopy(camera)


def test_empty_initial_depth_can_align_camera_without_measuring_a_surface(source):
    camera = camera_pose(0.08)
    image, info = wall_image(camera, 0.218)
    image.data = np.zeros((480, 640), dtype="<f4").tobytes()
    set_image(source, image, info, camera)

    view = source.source.observation_view(PoseData.identity())

    assert not view.visible
    assert view.hand_translation_odom == Vector3Data(0.0, 0.0, -0.08)
    assert view.target_plane_probe is None
    assert source.fits == 0
    # Geometry is only a movement proposal and cannot become a measurement.
    with pytest.raises(ValueError):
        source.source.sensor_tip_distance_samples(PoseData.identity())


@pytest.mark.parametrize("distance", [0.10, 0.218, 0.45])
@pytest.mark.parametrize("expected", [None, 0.04, 0.45])
def test_initial_background_and_saved_distance_do_not_choose_observation(source, distance, expected):
    camera = camera_pose(0.08)
    image, info = wall_image(camera, distance)
    set_image(source, image, info, camera)

    view = source.source.observation_view(PoseData.identity(), expected)

    assert not view.visible
    assert view.hand_translation_odom == Vector3Data(0.0, 0.0, -0.08)
    assert view.target_plane_probe is None
    assert source.fits == 0


def test_already_aligned_camera_measures_actual_target_without_motion(source):
    camera = camera_pose()
    image, info = wall_image(camera, 0.218)
    set_image(source, image, info, camera)

    view = source.source.observation_view(PoseData.identity())

    assert view.visible
    assert view.hand_translation_odom == Vector3Data.zero()
    assert view.target_plane_probe.signed_distance(Vector3Data.zero()) == pytest.approx(0.218, abs=1e-6)
    assert source.fits == 1


def test_sparse_target_after_move_does_not_trigger_another_excursion(source):
    # Deliberately leave a small lateral positioning error. Post-move retries
    # must inspect actual depth rather than issue another alignment movement.
    camera = camera_pose(0.005)
    image, info = wall_image(camera, 0.218)
    values = np.zeros((480, 640), dtype="<f4")
    values[249, 320] = 0.318
    image.data = values.tobytes()
    set_image(source, image, info, camera)

    view = source.source.observation_view(PoseData.identity(), allow_translation=False)

    assert not view.visible
    assert view.hand_translation_odom == Vector3Data.zero()
    assert view.target_plane_probe is None
    assert source.fits == 1
    with pytest.raises(ValueError):
        source.source.sensor_tip_distance_samples(PoseData.identity())


@pytest.mark.parametrize("camera_depth,valid", [(0.209, False), (0.210, True)])
def test_camera_minimum_distance_still_applies_after_geometry_only_move(source, camera_depth, valid):
    camera = camera_pose()
    image, info = wall_image(camera, camera_depth - 0.10)
    set_image(source, image, info, camera)

    view = source.source.observation_view(PoseData.identity(), allow_translation=False)

    assert view.visible == valid
    assert view.hand_translation_odom == Vector3Data.zero()
    if valid:
        assert view.target_plane_probe.signed_distance(Vector3Data.zero()) == pytest.approx(0.11, abs=1e-6)
    else:
        assert view.target_plane_probe is None
        assert "too close" in view.detail
        assert "minimum 0.210 m" in view.detail


def test_late_received_pre_move_acquisition_is_rejected_before_tf_or_measurement(source):
    source.now = 100.2
    source.ros_now = 10.2
    image, info = wall_image(camera_pose(), 0.218)
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = 0
    source.source._receive_hand_depth_camera_info(info)
    source.source._receive_hand_depth(image)

    with pytest.raises(ValueError, match="acquisition precedes"):
        source.source.observation_view(
            PoseData.identity(), receipt_not_before=100.1,
            acquisition_not_before=10.1, allow_translation=False,
        )

    assert source.poses == []
    assert source.fits == 0


@pytest.mark.parametrize("acquisition_nanoseconds", [100_000_000, 200_000_000])
def test_current_acquisition_at_or_after_move_cutoff_can_be_measured(source, acquisition_nanoseconds):
    source.now = 100.2
    source.ros_now = 10.2
    image, info = wall_image(camera_pose(), 0.218)
    image.header.stamp.sec = 10
    image.header.stamp.nanosec = acquisition_nanoseconds
    source.source._receive_hand_depth_camera_info(info)
    source.source._receive_hand_depth(image)

    view = source.source.observation_view(
        PoseData.identity(), receipt_not_before=100.1,
        acquisition_not_before=10.1, allow_translation=False,
    )

    assert view.visible
    assert view.hand_translation_odom == Vector3Data.zero()
    assert view.target_plane_probe.signed_distance(Vector3Data.zero()) == pytest.approx(0.218, abs=1e-6)
    assert source.poses == [("odom", "hand_depth", 10_000_000_000 + acquisition_nanoseconds)]
    assert source.fits == 1
