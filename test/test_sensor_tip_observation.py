"""Stationary target depth and lateral observation geometry contracts."""

from types import SimpleNamespace

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry.pose import compose_poses, inverse_pose
from fault_detector_spot.inspection.geometry.sensor_tip_observation import (
    transform_surface_plane,
)
from fault_detector_spot.inspection.sensing import probe_surface_source
from fault_detector_spot.inspection.sensing.probe_surface_source import (
    ProbeSurfaceSource, NoNewSensorTipPoseSample,
)
from fault_detector_spot.inspection.sensing.sensor_tip_distance import measure_sensor_tip_distance
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.geometry.rotation import quaternion_from_matrix
from test_live_surface_distance import fitted_plane, planar_depth


def camera_pose(height=0.0):
    # Optical +Z faces probe +X; optical +Y points below probe -Z.
    return PoseData(
        Vector3Data(-0.10, 0.0, height),
        quaternion_from_matrix(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
    )


def target_image(depth_m=0.35):
    image, info = planar_depth(width=128, height=96, depth_m=depth_m)
    info.k[0] = info.k[4] = 200.0
    return image, info


def test_plane_transform_preserves_normal_sign_and_round_trips():
    plane = fitted_plane(0.25)
    transformed = transform_surface_plane(plane, camera_pose(), "odom")
    assert transformed.normal.z == pytest.approx(0.0)
    restored = transform_surface_plane(transformed, inverse_pose(camera_pose()), "probe")
    assert restored.point_array() == pytest.approx(plane.point_array())
    assert restored.normal_array() == pytest.approx(plane.normal_array())


def test_measurement_refers_to_original_tip_after_camera_excursion():
    image, info = target_image()
    reference = PoseData.identity()
    observation_camera = camera_pose(height=0.01)
    sample = measure_sensor_tip_distance(image, info, reference, observation_camera, 0.25)
    assert sample.distance_m == pytest.approx(0.25, abs=1e-6)
    assert sample.surface_plane_probe.normal.x == pytest.approx(-1.0)
    assert sample.surface_plane_probe.frame_id == "probe"
    # An odom translation and rotation do not alter the original tip distance.
    anchor = camera_pose(height=1.0)
    shifted = measure_sensor_tip_distance(
        image, info, compose_poses(anchor, reference),
        compose_poses(anchor, observation_camera), 0.25,
    )
    assert shifted.distance_m == pytest.approx(sample.distance_m)
    assert shifted.surface_plane_probe.normal_array() == pytest.approx(
        sample.surface_plane_probe.normal_array(),
    )


def test_measurement_does_not_extrapolate_plane_when_target_is_outside_image():
    image, info = target_image()
    with pytest.raises(ValueError, match="outside the camera image"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(0.10), 0.25)


@pytest.mark.parametrize("depth", [0.0, 0.12])
def test_missing_or_occluded_target_cannot_be_replaced_by_surrounding_wall(depth):
    image, info = target_image()
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(96, 128).copy()
    values[45:52, 61:68] = depth
    image.data = values.tobytes()
    with pytest.raises(ValueError, match="insufficient unoccluded|too close"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(), 0.25)


def test_foreground_plane_must_match_expected_target_distance():
    image, info = target_image(depth_m=0.31)
    with pytest.raises(ValueError, match="does not match the intended"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(), 0.40)


def test_saved_distance_checks_correspondence_without_selecting_the_fitted_patch():
    image, info = planar_depth(width=128, height=256, depth_m=0.35)
    info.k[0] = info.k[4] = 200.0
    # A different saved distance must not shift the fitted patch away from
    # the actual intersection or stop its otherwise supported measurement.
    sample = measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(0.15), 0.16)
    assert sample.distance_m == pytest.approx(0.25, abs=1e-6)


@pytest.fixture
def source(monkeypatch):
    state = SimpleNamespace(now=100.0, ros_now=10.0, poses=[], fits=0)
    monkeypatch.setattr(probe_surface_source.time, "monotonic", lambda: state.now)
    state.source = ProbeSurfaceSource(SimpleNamespace(
        create_subscription=lambda *args, **kwargs: object(),
        destroy_subscription=lambda subscription: None,
        count_publishers=lambda topic: 1,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(state.ros_now * 1e9),
        )),
    ), None)
    image, info = target_image()
    state.source._receive_hand_depth_camera_info(info)
    state.source._receive_hand_depth(image)

    def lookup(target, source, lookup_time):
        state.poses.append((target, source, lookup_time.nanoseconds))
        return camera_pose()

    state.source._lookup_pose = lookup
    original_measure = probe_surface_source.measure_sensor_tip_distance

    def measure(*args, **kwargs):
        state.fits += 1
        return original_measure(*args, **kwargs)

    monkeypatch.setattr(probe_surface_source, "measure_sensor_tip_distance", measure)
    return state


def test_target_sample_uses_acquisition_tf_on_every_poll_and_caches_fit(source):
    for _ in range(2):
        samples = source.source.sensor_tip_distance_samples(PoseData.identity(), 0.25)
        assert samples[0].distance_m == pytest.approx(0.25, abs=1e-6)
    assert source.fits == 1
    assert source.poses == [("odom", "hand_depth", 10_000_000_000)] * 2


def test_new_saved_reference_invalidates_target_fit(source):
    reference = PoseData.identity()
    source.source.sensor_tip_distance_samples(reference, 0.25)
    reference.position.x += 0.01
    sample = source.source.sensor_tip_distance_samples(reference, 0.24)[0]
    assert source.fits == 2
    assert sample.distance_m == pytest.approx(0.24, abs=1e-6)


def test_changed_observed_target_invalidates_fit_without_becoming_the_measurement(source):
    reference = PoseData.identity()
    for target, fit_count in ((None, 1), (0.25, 2), (0.25, 2), (0.254, 3)):
        sample = source.source.sensor_tip_distance_samples(
            reference, target_plane_probe=None if target is None else fitted_plane(target),
        )[0]
        assert source.fits == fit_count
        assert sample.distance_m == pytest.approx(0.25, abs=1e-6)


def test_failed_anchored_view_cannot_reselect_an_observation_target(source, monkeypatch):
    from fault_detector_spot.inspection.sensing.sensor_tip_distance import SensorTipTargetUnavailable

    target = fitted_plane(0.25)

    def missing_target(*args, **kwargs):
        assert kwargs["target_plane_probe"] == target
        raise SensorTipTargetUnavailable("target depth missing")

    monkeypatch.setattr(probe_surface_source, "measure_sensor_tip_distance", missing_target)
    view = source.source.observation_view(PoseData.identity(), target_plane_probe=target)
    assert not view.visible
    assert view.target_plane_probe == target
    assert view.hand_translation_odom == Vector3Data.zero()
    assert "target depth missing" in view.detail


def test_observation_and_sample_cache_distinguish_discovery_from_saved_correspondence(source):
    image, _ = target_image(depth_m=0.48)
    image.header.stamp.nanosec = 100_000_000
    source.now += 0.1
    source.ros_now += 0.1
    source.source._receive_hand_depth(image)
    reference = PoseData.identity()
    assert not source.source.observation_view(reference, 0.25).visible
    view = source.source.observation_view(reference)
    assert view.visible
    assert view.hand_translation_odom == Vector3Data.zero()
    assert view.target_plane_probe.signed_distance(Vector3Data.zero()) == pytest.approx(0.38, abs=1e-6)
    for discover in (False, True, False, True):
        if discover:
            samples = source.source.sensor_tip_distance_samples(
                reference,
                receipt_not_before=source.now,
            )
            assert samples[0].distance_m == pytest.approx(0.38, abs=1e-6)
        else:
            with pytest.raises(ValueError, match="does not match the intended"):
                source.source.sensor_tip_distance_samples(
                    reference, 0.25, receipt_not_before=source.now,
                )


def test_target_samples_preserve_receipt_freshness_and_count_contract(source):
    with pytest.raises(ValueError, match="post_start=0"):
        source.source.sensor_tip_distance_samples(PoseData.identity(), 0.25, receipt_not_before=100.1)
    with pytest.raises(ValueError, match="at least 2 fresh"):
        source.source.sensor_tip_distance_samples(PoseData.identity(), 0.25, minimum_samples=2)
    source.now += 0.6
    with pytest.raises(ValueError, match="within_age=0"):
        source.source.sensor_tip_distance_samples(PoseData.identity(), 0.25)
    with pytest.raises(ValueError, match="Latest valid.*stale"):
        source.source.sensor_tip_distance_samples(
            PoseData.identity(), 0.25, maximum_age_sec=6.0, maximum_latest_age_sec=0.5,
        )


def test_postmove_sampling_excludes_pre_settle_acquisitions_even_when_cached(source):
    reference = PoseData.identity()
    source.source.sensor_tip_distance_samples(reference)
    source.now, source.ros_now = 100.2, 10.2
    # A previously fitted pre-move frame may be delivered again after settling.
    old_image, _ = target_image()
    source.source._receive_hand_depth(old_image)
    with pytest.raises(ValueError, match="acquisition precedes the required measurement"):
        source.source.sensor_tip_distance_samples(reference, acquisition_not_before=10.1)
    assert source.fits == 1

    image, _ = target_image()
    image.header.stamp.nanosec = 200_000_000
    source.source._receive_hand_depth(image)
    samples = source.source.sensor_tip_distance_samples(reference, acquisition_not_before=10.1)
    assert len(samples) == 1
    assert samples[0].stamp_seconds == pytest.approx(10.2)
    assert source.fits == 2


def test_observation_view_and_snapshot_use_same_timestamped_frames(source):
    view = source.source.observation_view(PoseData.identity(), 0.25)
    assert view.visible
    snapshot = source.source.sensor_tip_pose_snapshot("test_sensor")
    assert len(snapshot) == 3
    assert source.poses == [
        ("odom", "hand_depth", 10_000_000_000),
        ("odom", "hand", 10_000_000_000),
        ("odom", "test_sensor_probe", 10_000_000_000),
        ("odom", "body", 10_000_000_000),
    ]


def test_pose_snapshot_refuses_pre_return_and_empty_timestamp_frames(source):
    with pytest.raises(ValueError, match="after the required start"):
        source.source.sensor_tip_pose_snapshot("test_sensor", receipt_not_before=100.1)
    image, _ = target_image()
    image.header.stamp.sec = 0
    source.source._receive_hand_depth(image)
    with pytest.raises(ValueError, match="timestamp is empty"):
        source.source.sensor_tip_pose_snapshot("test_sensor")


def test_pose_sample_rejects_delayed_acquisition_even_after_new_receipt(source):
    source.now = 100.2
    source.ros_now = 10.2
    image, _ = target_image()  # Acquired at ROS 10.0, delivered after movement.
    source.source._receive_hand_depth(image)
    with pytest.raises(ValueError, match="acquisition precedes"):
        source.source.sensor_tip_pose_sample(
            "test_sensor", receipt_not_before=100.1, acquisition_not_before=10.1,
        )
    assert source.poses == []


def test_pose_sample_exposes_acquisition_identity_for_all_three_transforms(source):
    source.now = 100.2
    source.ros_now = 10.3
    image, _ = target_image()
    image.header.stamp.nanosec = 200_000_000
    source.source._receive_hand_depth(image)
    sample = source.source.sensor_tip_pose_sample(
        "test_sensor", receipt_not_before=100.1, acquisition_not_before=10.1,
    )
    assert sample.stamp_seconds == pytest.approx(10.2)
    assert sample.hand == camera_pose()
    assert sample.probe == camera_pose()
    assert sample.body == camera_pose()
    assert source.poses == [
        ("odom", "hand", 10_200_000_000),
        ("odom", "test_sensor_probe", 10_200_000_000),
        ("odom", "body", 10_200_000_000),
    ]


@pytest.mark.parametrize("ros_now,match", [
    (10.6, "acquisition is stale"),
    (9.99, "timestamp is in the future"),
])
def test_pose_sample_rejects_stale_or_future_acquisition_before_tf(source, ros_now, match):
    source.ros_now = ros_now
    with pytest.raises(ValueError, match=match):
        source.source.sensor_tip_pose_sample("test_sensor")
    assert source.poses == []


def test_pose_sample_freshness_is_rechecked_after_tf_resolution(source):
    lookup = source.source._lookup_pose

    def delayed_lookup(*args, **kwargs):
        result = lookup(*args, **kwargs)
        source.ros_now += .2
        return result

    source.source._lookup_pose = delayed_lookup
    with pytest.raises(ValueError, match="acquisition is stale"):
        source.source.sensor_tip_pose_sample("test_sensor")
    assert len(source.poses) == 3


@pytest.mark.parametrize("error", ["extrapolation into the past", "extrapolation into the future"])
def test_pose_sample_does_not_fall_back_to_latest_tf(source, error):
    requested = []

    def unavailable(target, frame, lookup_time):
        requested.append((target, frame, lookup_time.nanoseconds))
        raise RuntimeError(error)

    source.source._lookup_pose = unavailable
    with pytest.raises(RuntimeError, match=error):
        source.source.sensor_tip_pose_sample("test_sensor")
    assert requested == [("odom", "hand", 10_000_000_000)]


def test_pose_sample_rejects_nonfinite_header_before_tf(source):
    _, info = target_image()
    invalid = SimpleNamespace(header=SimpleNamespace(
        frame_id="hand_depth", stamp=SimpleNamespace(sec=float("nan"), nanosec=0),
    ))
    source.source.latest_hand_depth = lambda **kwargs: (invalid, info)
    with pytest.raises(ValueError, match="timestamp is empty or nonfinite"):
        source.source.sensor_tip_pose_sample("test_sensor")
    assert source.poses == []


def test_pose_samples_keep_duplicate_and_out_of_order_acquisition_identity(source):
    first = source.source.sensor_tip_pose_sample("test_sensor")
    second = source.source.sensor_tip_pose_sample("test_sensor")
    assert first.stamp_seconds == second.stamp_seconds
    source.now += .1
    source.ros_now += .1
    older, _ = target_image()
    older.header.stamp.sec = 9
    older.header.stamp.nanosec = 900_000_000
    source.source._receive_hand_depth(older)
    latest_receipt = source.source.sensor_tip_pose_sample("test_sensor")
    assert latest_receipt.stamp_seconds == pytest.approx(9.9)
    assert latest_receipt.stamp_seconds < first.stamp_seconds


def test_consumed_pose_frame_aging_between_updates_is_no_new_evidence(source):
    accepted = source.source.sensor_tip_pose_sample("test_sensor")
    source.now += 0.7
    source.ros_now += 0.7
    source.poses.clear()
    with pytest.raises(NoNewSensorTipPoseSample, match="distinct, advancing"):
        source.source.sensor_tip_pose_sample("test_sensor", after_stamp_seconds=accepted.stamp_seconds)
    assert source.poses == []
    image, _ = target_image()
    image.header.stamp.nanosec = 400_000_000
    source.source._receive_hand_depth(image)
    sample = source.source.sensor_tip_pose_sample("test_sensor", after_stamp_seconds=accepted.stamp_seconds)
    assert sample.stamp_seconds == pytest.approx(10.4)
    assert len(source.poses) == 3


def test_unseen_stale_pose_frame_remains_invalid(source):
    source.now += 0.7
    source.ros_now += 0.7
    image, _ = target_image()
    image.header.stamp.nanosec = 100_000_000
    source.source._receive_hand_depth(image)
    with pytest.raises(ValueError, match="acquisition is stale") as failure:
        source.source.sensor_tip_pose_sample("test_sensor", after_stamp_seconds=10.0)
    assert not isinstance(failure.value, NoNewSensorTipPoseSample)
    assert source.poses == []


@pytest.mark.parametrize("threshold", [-0.1, float("nan"), float("inf")])
def test_pose_sample_rejects_invalid_previous_stamp(source, threshold):
    with pytest.raises(ValueError, match="Previous pose acquisition timestamp"):
        source.source.sensor_tip_pose_sample("test_sensor", after_stamp_seconds=threshold)


def test_pose_cutoff_uses_ros_clock_and_explicit_settle_delay(source):
    assert source.source.pose_acquisition_cutoff(.2) == pytest.approx(10.2)
    assert source.now == 100.0


@pytest.mark.parametrize("delay", [-.1, float("nan"), float("inf")])
def test_pose_cutoff_rejects_invalid_delays(source, delay):
    with pytest.raises(ValueError, match="delay must be finite and nonnegative"):
        source.source.pose_acquisition_cutoff(delay)


@pytest.mark.parametrize("threshold", [-.1, float("nan"), float("inf")])
def test_pose_sample_rejects_invalid_acquisition_threshold(source, threshold):
    with pytest.raises(ValueError, match="threshold must be finite and nonnegative"):
        source.source.sensor_tip_pose_sample("test_sensor", acquisition_not_before=threshold)
